#include "./localization.hpp"
#include "imu_transform.hpp"

#include <sstream>
#include <fstream>
#include <stdio.h>
#include <iomanip>

#ifdef HAS_CPUID
#include <cpuid.h>
#endif

#include <boost/algorithm/string.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>


using namespace util::geom::cvt::ops;


PerceptionNode::PerceptionNode() :
    Node("cardinal_perception"),
    tf_buffer{ std::make_shared<rclcpp::Clock>(RCL_ROS_TIME) },
    tf_listener{ tf_buffer },
    tf_broadcaster{ *this },
    mt_callback_group{ this->create_callback_group(rclcpp::CallbackGroupType::Reentrant) },
    metrics_pub{ this, "/perception_debug/", 1 },
    pose_pub{ this, "/perception_debug/", 1 },
    lidar_odom{ this },
    trajectory_filter{}
{
    this->getParams();
    this->initMetrics();

#if USE_GTSAM_PGO > 0
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = 0.1;
    params.relinearizeSkip = 1;
    this->pgo.isam = std::make_shared<gtsam::ISAM2>(params);
#endif

    std::string scan_topic, imu_topic;
    util::declare_param(this, "scan_topic", scan_topic, "scan");
    util::declare_param(this, "imu_topic", imu_topic, "imu");

    rclcpp::SubscriptionOptions ops{};
    ops.callback_group = this->mt_callback_group;

    this->detections_sub = this->create_subscription<cardinal_perception::msg::TagsDetection>(
        "tags_detections", rclcpp::SensorDataQoS{},
        [this](const cardinal_perception::msg::TagsDetection::ConstSharedPtr& det){ this->detection_callback(det); }, ops);
    this->scan_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        scan_topic, rclcpp::SensorDataQoS{},
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan){ this->scan_callback(scan); }, ops);
    this->imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, rclcpp::SensorDataQoS{},
        [this](const sensor_msgs::msg::Imu::SharedPtr imu){ this->imu_callback(imu); }, ops);

    this->filtered_scan_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_scan", rclcpp::SensorDataQoS{});
#if USE_GTSAM_PGO > 0
    this->path_pub = this->create_publisher<nav_msgs::msg::Path>("path", rclcpp::SensorDataQoS{});
#endif
}


void PerceptionNode::getParams()
{
    util::declare_param(this, "map_frame_id", this->map_frame, "map");
    util::declare_param(this, "odom_frame_id", this->odom_frame, "odom");
    util::declare_param(this, "base_frame_id", this->base_frame, "base_link");

    util::declare_param(this, "debug.status_max_print_freq", this->param.status_max_print_freq, 10.);
    util::declare_param(this, "debug.img_max_pub_freq", this->param.img_debug_max_pub_freq, 30.);
}

void PerceptionNode::initMetrics()
{
    char CPUBrandString[0x40];
    memset(CPUBrandString, 0, sizeof(CPUBrandString));
    this->metrics.cpu_type = "";

#ifdef HAS_CPUID
    unsigned int CPUInfo[4] = {0, 0, 0, 0};
    __cpuid(0x80000000, CPUInfo[0], CPUInfo[1], CPUInfo[2], CPUInfo[3]);
    unsigned int nExIds = CPUInfo[0];
    for(unsigned int i = 0x80000000; i <= nExIds; ++i)
    {
        __cpuid(i, CPUInfo[0], CPUInfo[1], CPUInfo[2], CPUInfo[3]);
        if(i == 0x80000002)
            memcpy(CPUBrandString, CPUInfo, sizeof(CPUInfo));
        else if(i == 0x80000003)
            memcpy(CPUBrandString + 16, CPUInfo, sizeof(CPUInfo));
        else if(i == 0x80000004)
            memcpy(CPUBrandString + 32, CPUInfo, sizeof(CPUInfo));
    }

    this->metrics.cpu_type = CPUBrandString;
    boost::trim(this->metrics.cpu_type);
#endif

    FILE * file;
    struct tms timeSample;
    char line[128];

    this->metrics.last_cpu = times(&timeSample);
    this->metrics.last_sys_cpu = timeSample.tms_stime;
    this->metrics.last_user_cpu = timeSample.tms_utime;

    file = fopen("/proc/cpuinfo", "r");
    this->metrics.num_processors = 0;
    while(fgets(line, 128, file) != NULL)
    {
        if(strncmp(line, "processor", 9) == 0)
            this->metrics.num_processors++;
    }
    fclose(file);
}

void PerceptionNode::sendTf(const builtin_interfaces::msg::Time& stamp, bool needs_lock)
{
    if(needs_lock) this->state.tf_mtx.lock();

    geometry_msgs::msg::TransformStamped _tf;
    _tf.header.stamp = stamp;
    // map to odom
    _tf.header.frame_id = this->map_frame;
    _tf.child_frame_id = this->odom_frame;
    _tf.transform << this->state.map_tf.pose;
    this->tf_broadcaster.sendTransform(_tf);
    // odom to base
    _tf.header.frame_id = this->odom_frame;
    _tf.child_frame_id = this->base_frame;
    _tf.transform << this->state.odom_tf.pose;
    // _tf.transform << full_tf;
    this->tf_broadcaster.sendTransform(_tf);

    if(needs_lock) this->state.tf_mtx.unlock();
}

void PerceptionNode::handleStatusUpdate()
{
    // try lock mutex
    if(this->state.print_mtx.try_lock())
    {
        // check frequency
        auto _tp = std::chrono::system_clock::now();
        const double _dt = util::toFloatSeconds(_tp - this->state.last_print_time);
        if(_dt > (1. / this->param.status_max_print_freq))
        {
            this->state.last_print_time = _tp;
            std::ostringstream msg;

            // read stats from /proc
            double /*vm_usage = 0.,*/ resident_set = 0., cpu_percent = -1.;
            size_t num_threads = 0;
            {
                std::string pid, comm, state, ppid, pgrp, session, tty_nr;
                std::string tpgid, flags, minflt, cminflt, majflt, cmajflt;
                std::string utime, stime, cutime, cstime, priority, nice;
                std::string itrealvalue, starttime;
                unsigned long vsize;
                long rss;

                std::ifstream stat_stream("/proc/self/stat", std::ios_base::in);
                stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr >> tpgid >> flags >> minflt >> cminflt >>
                    majflt >> cmajflt >> utime >> stime >> cutime >> cstime >> priority >> nice >> num_threads >> itrealvalue >>
                    starttime >> vsize >> rss; // don't care about the rest
                stat_stream.close();

                long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024;
                // vm_usage = vsize / 1024.;
                resident_set = rss * page_size_kb;

                struct tms _sample;
                clock_t now = times(&_sample);
                if( now > this->metrics.last_cpu &&
                    _sample.tms_stime >= this->metrics.last_sys_cpu &&
                    _sample.tms_utime >= this->metrics.last_user_cpu )
                {
                    cpu_percent =
                        ( (_sample.tms_stime - this->metrics.last_sys_cpu) + (_sample.tms_utime - this->metrics.last_user_cpu) ) *
                        ( 100. / (now - this->metrics.last_cpu) / this->metrics.num_processors );
                }
                this->metrics.last_cpu = now;
                this->metrics.last_sys_cpu = _sample.tms_stime;
                this->metrics.last_user_cpu = _sample.tms_utime;

                this->metrics.avg_cpu_percent = (this->metrics.avg_cpu_percent * this->metrics.avg_cpu_samples + cpu_percent) / (this->metrics.avg_cpu_samples + 1);
                this->metrics.avg_cpu_samples++;
                if(cpu_percent > this->metrics.max_cpu_percent) this->metrics.max_cpu_percent = cpu_percent;
            }

            msg << std::setprecision(2) << std::fixed << std::right << std::setfill(' ') << std::endl;
            msg << "+-------------------------------------------------------------------+\n"
                   "| =================== Cardinal Perception v0.0.1 ================== |\n"
                   "+- RESOURCES -------------------------------------------------------+\n"
                   "|                      ::  Current  |  Average  |  Maximum          |\n";
            msg << "|      CPU Utilization ::  " << std::setw(6) << cpu_percent
                                                << " %  |  " << std::setw(6) << this->metrics.avg_cpu_percent
                                                            << " %  |  " << std::setw(5) << this->metrics.max_cpu_percent
                                                                         << " %         |\n";
            msg << "|       RAM Allocation :: " << std::setw(6) << resident_set / 1000.
                                                << " MB |                               |\n";
            msg << "|        Total Threads ::  " << std::setw(5) << num_threads
                                                << "    |                               |\n";
            msg << "|                                                                   |\n"
                << "+- CALLBACKS -------------------------------------------------------+\n"
                << "|                        Comp. Time | Avg. Time | Max Time | Total  |\n";
            msg << std::setprecision(1) << std::fixed << std::right << std::setfill(' ');
            // this->metrics.info_thread.mtx.lock();
            // msg << "|  Info CB (" << std::setw(5) << 1. / this->metrics.info_thread.avg_call_delta
            //                      << " Hz) ::  " << std::setw(5) << this->metrics.info_thread.last_comp_time * 1e6
            //                                    << " us  | " << std::setw(5) << this->metrics.info_thread.avg_comp_time * 1e6
            //                                                << " us  | " << std::setw(5) << this->metrics.info_thread.max_comp_time * 1e3
            //                                                            << " ms | " << std::setw(6) << this->metrics.info_thread.samples
            //                                                                        << " |\n";
            // this->metrics.info_thread.mtx.unlock();
            this->metrics.img_thread.mtx.lock();
            msg << "|   DET CB (" << std::setw(5) << 1. / this->metrics.img_thread.avg_call_delta
                                 << " Hz) ::  " << std::setw(5) << this->metrics.img_thread.last_comp_time * 1e6
                                               << " us  | " << std::setw(5) << this->metrics.img_thread.avg_comp_time * 1e6
                                                           << " us  | " << std::setw(5) << this->metrics.img_thread.max_comp_time * 1e3
                                                                       << " ms | " << std::setw(6) << this->metrics.img_thread.samples
                                                                                   << " |\n";
            this->metrics.img_thread.mtx.unlock();
            this->metrics.imu_thread.mtx.lock();
            msg << "|   IMU CB (" << std::setw(5) << 1. / this->metrics.imu_thread.avg_call_delta
                                 << " Hz) ::  " << std::setw(5) << this->metrics.imu_thread.last_comp_time * 1e6
                                               << " us  | " << std::setw(5) << this->metrics.imu_thread.avg_comp_time * 1e6
                                                           << " us  | " << std::setw(5) << this->metrics.imu_thread.max_comp_time * 1e3
                                                                       << " ms | " << std::setw(6) << this->metrics.imu_thread.samples
                                                                                   << " |\n";
            this->metrics.imu_thread.mtx.unlock();
            this->metrics.scan_thread.mtx.lock();
            msg << "|  Scan CB (" << std::setw(5) << 1. / this->metrics.scan_thread.avg_call_delta
                                 << " Hz) ::  " << std::setw(5) << this->metrics.scan_thread.last_comp_time * 1e3
                                               << " ms  | " << std::setw(5) << this->metrics.scan_thread.avg_comp_time * 1e3
                                                           << " ms  | " << std::setw(5) << this->metrics.scan_thread.max_comp_time * 1e3
                                                                       << " ms | " << std::setw(6) << this->metrics.scan_thread.samples
                                                                                   << " |\n";
            this->metrics.scan_thread.mtx.unlock();
            msg << "|                                                                   |\n"
                   "+- THREAD UTILIZATION ----------------------------------------------+\n"
                   "|                                                                   |\n";

            this->metrics.thread_procs_mtx.lock();
            size_t idx = 0;
            for(auto& p : this->metrics.thread_proc_times)
            {
                static constexpr char const* CHAR_VARS = "DSuFMx"; // Det, Scan, imu, debug Frame, Metrics, misc
                static constexpr char const* COLORS[7] =
                {
                    "\033[38;5;49m",
                    // "\033[38;5;11m",
                    "\033[38;5;45m",
                    "\033[38;5;198m",
                    "\033[38;5;228m",
                    "\033[38;5;99m",
                    "\033[37m"
                };
                msg << "| " << std::setw(3) << idx++ << ": [";    // start -- 8 chars
                // fill -- 58 chars
                size_t avail_chars = 58;
                for(size_t x = 0; x < (size_t)ProcType::NUM_ITEMS; x++)
                {
                    size_t n_chars = static_cast<size_t>(p.second[x] / _dt * 58.);
                    n_chars = std::min(n_chars, avail_chars);
                    avail_chars -= n_chars;
                    p.second[x] = 0.;
                    if(n_chars > 0)
                    {
                        msg << COLORS[x] << std::setfill('=') << std::setw(n_chars) << CHAR_VARS[x];
                    }
                }
                msg << "\033[0m" << std::setfill(' ');
                if(avail_chars > 0)
                {
                    msg << std::setw(avail_chars) << ' ';
                }
                msg << "] |\n"; // end -- 3 chars
            }
            this->metrics.thread_procs_mtx.unlock();

            msg << "+-------------------------------------------------------------------+" << std::endl;

            // print
            // printf("\033[2J\033[1;1H");
            std::cout << "\033[2J\033[1;1H" << std::endl;
            RCLCPP_INFO(this->get_logger(), msg.str().c_str());
        }
        this->appendThreadProcTime(ProcType::HANDLE_METRICS, util::toFloatSeconds(std::chrono::system_clock::now() - _tp));
        this->state.print_mtx.unlock();
    }
}


void PerceptionNode::detection_callback(const cardinal_perception::msg::TagsDetection::ConstSharedPtr& detection_group)
{
    auto _start = std::chrono::system_clock::now();

    const geometry_msgs::msg::TransformStamped& tf = detection_group->estimated_tf;
    if(tf.header.frame_id == this->map_frame && tf.child_frame_id == this->base_frame)
    {
        TagDetection::Ptr td = std::make_shared<TagDetection>();
        td->pose << tf.transform;
        td->time_point = util::toFloatSeconds(tf.header.stamp);
        td->pix_area = detection_group->pix_area;
        td->avg_range = detection_group->avg_range;
        td->rms = detection_group->rms;
        td->num_tags = detection_group->num_tags;
        this->trajectory_filter.addMeasurement(td, td->time_point);
    }

    auto _end = std::chrono::system_clock::now();
    const double _dt = this->metrics.img_thread.addSample(_start, _end);
    this->appendThreadProcTime(ProcType::DET_CB, _dt);

    this->handleStatusUpdate();
}

void PerceptionNode::scan_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan)
{
    auto _start = std::chrono::system_clock::now();

    thread_local pcl::PointCloud<DLOdom::PointType>::Ptr filtered_scan = std::make_shared<pcl::PointCloud<DLOdom::PointType>>();
    thread_local util::geom::PoseTf3d new_odom_tf;
    int64_t dlo_status = 0;

    this->state.dlo_in_progress = true;
    try
    {
        sensor_msgs::msg::PointCloud2::SharedPtr scan_ = std::make_shared<sensor_msgs::msg::PointCloud2>();

        auto tf = this->tf_buffer.lookupTransform(
            this->base_frame,
            scan->header.frame_id,
            util::toTf2TimePoint(scan->header.stamp));

        tf2::doTransform(*scan, *scan_, tf);

        dlo_status = this->lidar_odom.processScan(scan_, filtered_scan, new_odom_tf);   // TODO: add mtx lock timeout so we don't hang here
    }
    catch(const std::exception& e)
    {
        RCLCPP_INFO(this->get_logger(), "SCAN CALLBACK: [dlo] failed to process scan.\n\twhat(): %s", e.what());
        dlo_status = 0;
    }
    this->state.dlo_in_progress = false;

    if(dlo_status)
    {
        const double new_odom_stamp = util::toFloatSeconds(scan->header.stamp);

        geometry_msgs::msg::PoseStamped _pose;
        _pose.header.stamp = scan->header.stamp;

        this->trajectory_filter.addOdom(new_odom_tf.pose, new_odom_stamp);

        const bool stable = this->trajectory_filter.lastFilterStatus();
        const size_t
            odom_q_sz = this->trajectory_filter.odomQueueSize(),
            meas_q_sz = this->trajectory_filter.measurementQueueSize(),
            trajectory_sz = this->trajectory_filter.trajectoryQueueSize();

        this->metrics_pub.publish("trajectory_filter/stable", stable ? 1. : 0.);
        this->metrics_pub.publish("trajectory_filter/odom_queue_size", (double)odom_q_sz);
        this->metrics_pub.publish("trajectory_filter/measurement_queue_size", (double)meas_q_sz);
        this->metrics_pub.publish("trajectory_filter/trajectory_len", (double)trajectory_sz);
        this->metrics_pub.publish("trajectory_filter/filter_dt", this->trajectory_filter.lastFilterWindow());
        this->metrics_pub.publish("trajectory_filter/avg_linear_err", this->trajectory_filter.lastAvgLinearError());
        this->metrics_pub.publish("trajectory_filter/avg_angular_err", this->trajectory_filter.lastAvgAngularError());
        this->metrics_pub.publish("trajectory_filter/linear_variance", this->trajectory_filter.lastLinearVariance());
        this->metrics_pub.publish("trajectory_filter/angular_variance", this->trajectory_filter.lastAngularVariance());
        this->metrics_pub.publish("trajectory_filter/linear_error", this->trajectory_filter.lastLinearDelta());
        this->metrics_pub.publish("trajectory_filter/angular_error", this->trajectory_filter.lastAngularDelta());

    #if USE_GTSAM_PGO > 0
        size_t run_isam = 0;
    #endif

        this->state.tf_mtx.lock();  // TODO: timeout

        if(!(dlo_status & (1 << 1)))
        {
        #if USE_GTSAM_PGO > 0
            const bool need_keyframe_update = dlo_status & (1 << 2);

            static gtsam::noiseModel::Diagonal::shared_ptr odom_noise =    // shared for multiple branches >>>
                gtsam::noiseModel::Diagonal::Variances( (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 25e-6, 25e-6, 25e-6).finished() );
        #endif

            if(stable)
            {
                auto keypose = this->trajectory_filter.getFiltered();
                const TagDetection::Ptr& detection = keypose.second.measurement;

                _pose.header.frame_id = this->odom_frame;
                _pose.pose << keypose.second.odometry;
                this->pose_pub.publish("filtered_odom_pose", _pose);

                _pose.header.frame_id = this->map_frame;
                _pose.pose << detection->pose;
                this->pose_pub.publish("filtered_aruco_pose", _pose);

            #if USE_GTSAM_PGO > 0
                gtsam::Pose3 aruco_pose;
                aruco_pose << detection->pose;

                const double
                    rms_per_tag = detection->rms / detection->num_tags,
                    linear_variance = this->tag_filtering.covariance_linear_base_coeff +
                        this->tag_filtering.covariance_linear_range_coeff * detection->avg_range +
                        this->tag_filtering.covariance_linear_rms_per_tag_coeff * rms_per_tag,
                    angular_variance = this->tag_filtering.covariance_angular_base_coeff +
                        this->tag_filtering.covariance_angular_range_coeff * detection->avg_range +
                        this->tag_filtering.covariance_angular_rms_per_tag_coeff * rms_per_tag;

                RCLCPP_INFO(this->get_logger(), "ARUCO VARIANCE -- linear: %f -- angular: %f", linear_variance, angular_variance);

                gtsam::noiseModel::Diagonal::shared_ptr aruco_noise =
                    gtsam::noiseModel::Diagonal::Variances(
                        (gtsam::Vector(6) <<
                            angular_variance, angular_variance, angular_variance,
                            linear_variance, linear_variance, linear_variance ).finished() );

                gtsam::Pose3 odom_to;
                odom_to << keypose.second.odometry;

                this->pgo.factor_graph.add(
                    gtsam::BetweenFactor<gtsam::Pose3>(
                        this->pgo.next_state_idx - 1,
                        this->pgo.next_state_idx,
                        this->pgo.last_odom.between(odom_to),
                        odom_noise) );
                this->pgo.factor_graph.add(gtsam::PriorFactor<gtsam::Pose3>(this->pgo.next_state_idx, aruco_pose, aruco_noise));
                Eigen::Isometry3d _absolute;
                gtsam::Pose3 absolute_to;
                absolute_to << (_absolute << keypose.second.odometry) * this->state.map_tf.tf;  // << TODO: this is royally screwed!
                this->pgo.init_estimate.insert(this->pgo.next_state_idx, aruco_pose);

                _pose.pose << absolute_to;
                _pose.header.frame_id = this->map_frame;
                this->pose_pub.publish("absolute_matched_odom", _pose);

                this->pgo.last_odom = odom_to;
                this->pgo.next_state_idx++;

                run_isam = 2;
            }

            if(need_keyframe_update)
            {
                // RCLCPP_INFO(this->get_logger(), "SCAN CALLBACK: Creating factor graph -- adding independant keyframe...");
                gtsam::Pose3 odom_to;
                odom_to << new_odom_tf.pose;

                this->pgo.factor_graph.add(
                    gtsam::BetweenFactor<gtsam::Pose3>(
                        this->pgo.next_state_idx - 1,
                        this->pgo.next_state_idx,
                        this->pgo.last_odom.between(odom_to),
                        odom_noise) );

                // TODO: make sure this stays relative to the PGO values when/if DLO is updated using these estimates
                gtsam::Pose3 absolute_to;
                this->pgo.init_estimate.insert(this->pgo.next_state_idx, (absolute_to << new_odom_tf.tf * this->state.map_tf.tf));

                this->pgo.keyframe_state_indices.push_back(this->pgo.next_state_idx);
                this->pgo.last_odom = odom_to;
                this->pgo.next_state_idx++;

                run_isam = 2;
            #else
                {
                    Eigen::Isometry3d _absolute, _match;
                    this->state.map_tf.tf = (_absolute << detection->pose) * (_match << keypose.second.odometry).inverse();
                    this->state.map_tf.pose << this->state.map_tf.tf;
                }
            #endif
            }

        }
    #if USE_GTSAM_PGO > 0
        else    // first keyframe added -- initialize PGO
        {
            // RCLCPP_INFO(this->get_logger(), "SCAN CALLBACK: Creating factor graph -- adding initial pose...");
            static gtsam::noiseModel::Diagonal::shared_ptr init_noise =
                gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished());
                // gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1., 1., 1., 1., 1., 1.).finished());

            this->pgo.last_odom << new_odom_tf.pose;     // from dlo -- contains preset pose

            this->pgo.factor_graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, this->pgo.last_odom, init_noise));
            this->pgo.init_estimate.insert(0, this->pgo.last_odom);

            this->pgo.keyframe_state_indices.push_back(0);
            this->pgo.next_state_idx++;

            run_isam = 1;
        }

        if(run_isam)
        {
            try
            {
                this->pgo.isam->update(this->pgo.factor_graph, this->pgo.init_estimate);
                for(size_t i = 1; i < run_isam; i++)
                {
                    this->pgo.isam->update();
                }
            }
            catch(const std::exception& e)
            {
                RCLCPP_INFO(this->get_logger(), "SCAN CALLBACK: Caught ISAM::update() exception: %s", e.what());
                this->pgo.isam->print();
                throw e;
            }

            this->pgo.factor_graph.resize(0);
            this->pgo.init_estimate.clear();

            try
            {
                this->pgo.isam_estimate = this->pgo.isam->calculateEstimate();
            }
            catch(const std::exception& e)
            {
                RCLCPP_INFO(this->get_logger(), "SCAN CALLBACK: Caught ISAM::calculateEstimate() exception: %s", e.what());
                this->pgo.isam->print();
                throw e;
            }

            const size_t
                last_len = this->pgo.trajectory_buff.poses.size(),
                path_len = this->pgo.isam_estimate.size();

            // don't touch odom frame, update map so that the full transform is equivalent to the PGO solution
            Eigen::Isometry3d pgo_full, current_odom;
            pgo_full << this->pgo.isam_estimate.at<gtsam::Pose3>(path_len - 1);
            current_odom << this->pgo.last_odom;

            // Eigen::MatrixXd cov = this->pgo.isam->marginalCovariance(path_len - 1);
            // RCLCPP_INFO(this->get_logger(), "PGO Marginal Covariance Size: (%d, %d)", cov.rows(), cov.cols());
            // std::cout << cov << std::endl;

            this->pgo.trajectory_buff.poses.resize(path_len);
            for(size_t i = last_len/*(path_len > 100 ? path_len - 100 : 0)*/; i < path_len; i++)
            {
                geometry_msgs::msg::PoseStamped& _p = this->pgo.trajectory_buff.poses[i];
                _p.header.stamp = scan->header.stamp;
                _p.header.frame_id = this->map_frame;
                _p.pose << this->pgo.isam_estimate.at<gtsam::Pose3>(i);
            }
            if(this->path_pub->get_subscription_count() > 0)
            {
                this->pgo.trajectory_buff.header.stamp = scan->header.stamp;
                this->pgo.trajectory_buff.header.frame_id = this->map_frame;
                this->path_pub->publish(this->pgo.trajectory_buff);
            }

            // this->state.map_tf.tf = (new_odom_tf.pose.quat.inverse() * Eigen::Translation3d{ -new_odom_tf.pose.vec }) * pgo_full;
            this->state.map_tf.tf = pgo_full * current_odom.inverse();
            this->state.map_tf.pose << this->state.map_tf.tf;

            // TODO: export path
            RCLCPP_INFO(this->get_logger(), "SCAN CALLBACK: Successfully ran ISAM with graph of length %lu and %lu keyframes", this->pgo.isam_estimate.size(), this->pgo.keyframe_state_indices.size());
        }
    #endif

        // TODO: use PGO??? >>
        this->state.odom_tf = new_odom_tf;
        this->state.last_odom_stamp = new_odom_stamp;

        // publish tf
        this->sendTf(scan->header.stamp, false);
        this->state.tf_mtx.unlock();

        // mapping -- or send to another thread

        try     // TODO: thread_local may cause this to republish old scans?
        {
            sensor_msgs::msg::PointCloud2 filtered_pc;
            pcl::toROSMsg(*filtered_scan, filtered_pc);
            filtered_pc.header.stamp = scan->header.stamp;
            filtered_pc.header.frame_id = this->base_frame;

            this->filtered_scan_pub->publish(filtered_pc);
        }
        catch(const std::exception& e)
        {
            RCLCPP_INFO(this->get_logger(), "SCAN CALLBACK: failed to publish filtered scan.\n\twhat(): %s", e.what());
        }
    }

    auto _end = std::chrono::system_clock::now();
    const double _dt = this->metrics.scan_thread.addSample(_start, _end);
    this->appendThreadProcTime(ProcType::SCAN_CB, _dt);

    this->handleStatusUpdate();
    // this->handleDebugFrame();

    // RCLCPP_INFO(this->get_logger(), "SCAN_CALLBACK EXIT -- %s", (std::stringstream{} << std::this_thread::get_id()).str().c_str());
}

void PerceptionNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu)
{
    // RCLCPP_INFO(this->get_logger(), "IMU_CALLBACK");
    auto _start = std::chrono::system_clock::now();

    try
    {
        auto tf = this->tf_buffer.lookupTransform(
            this->base_frame,
            imu->header.frame_id,
            util::toTf2TimePoint(imu->header.stamp));

        tf2::doTransform(*imu, *imu, tf);

        this->lidar_odom.processImu(*imu);
    }
    catch(const std::exception& e)
    {
        RCLCPP_INFO(this->get_logger(), "IMU CALLBACK: [dlo] failed to process imu measurment.\n\twhat(): %s", e.what());
    }

    auto _end = std::chrono::system_clock::now();
    const double _dt = this->metrics.imu_thread.addSample(_start, _end);
    this->appendThreadProcTime(ProcType::IMU_CB, _dt);

    this->handleStatusUpdate();
    // this->handleDebugFrame();

    // RCLCPP_INFO(this->get_logger(), "IMU_CALLBACK EXIT");
}


double PerceptionNode::ThreadMetrics::addSample(
    const std::chrono::system_clock::time_point& start,
    const std::chrono::system_clock::time_point& end)
{
    std::lock_guard<std::mutex> _lock{ this->mtx };

    const double
        call_diff = util::toFloatSeconds(start - this->last_call_time),
        comp_time = util::toFloatSeconds(end - start);
    this->last_call_time = start;
    this->last_comp_time = comp_time;

    constexpr size_t ROLLING_SAMPLE_MAX = 250;
    const size_t sample_weight = std::min(ROLLING_SAMPLE_MAX, this->samples);

    this->avg_comp_time = (this->avg_comp_time * sample_weight + comp_time) / (sample_weight + 1);
    if(this->samples != 0)
    {
        this->avg_call_delta = (this->avg_call_delta * sample_weight + call_diff) / (sample_weight + 1);
    }
    this->samples++;

    if(comp_time > this->max_comp_time) this->max_comp_time = comp_time;

    return comp_time;
}

void PerceptionNode::appendThreadProcTime(ProcType type, double dt)
{
    if(type == ProcType::NUM_ITEMS) return;
    std::lock_guard<std::mutex> _lock{ this->metrics.thread_procs_mtx };

    std::thread::id _id = std::this_thread::get_id();
    auto ptr = this->metrics.thread_proc_times.find(_id);
    if(ptr == this->metrics.thread_proc_times.end())
    {
        auto x = this->metrics.thread_proc_times.insert({_id, {}});
        if(x.second)
        {
            memset(x.first->second.data(), 0, sizeof(double) * (size_t)ProcType::NUM_ITEMS);
            ptr = x.first;
        }
        else return;
    }
    ptr->second[(size_t)type] += dt;
}

/** template
+------------------------------------------------------------------+
| =================  Cardinal Perception v0.0.1  ================= |
+- RESOURCES ------------------------------------------------------+
|                    ::  Current   |  Average  |  Maximum          |
|    CPU Utilization ::  0.000%    |  0.000%   |  0.000%           |
|     RAM Allocation ::  0.000 MB  |                               |
|      Total Threads ::  0000      |                               |
|                                                                  |
+- CALLBACKS ------------------------------------------------------+
|                       Comp. Time | Avg. Time | Max Time | Total  |
|  Info CB (0.00 Hz) ::  0.000 ms  | 0.000 ms  | 0.000 ms | 0000   |
| Image CB (0.00 Hz) ::  0.000 ms  | 0.000 ms  | 0.000 ms | 0000   |
|   IMU CB (0.00 Hz) ::  0.000 ms  | 0.000 ms  | 0.000 ms | 0000   |
|  Scan CB (0.00 Hz) ::  0.000 ms  | 0.000 ms  | 0.000 ms | 0000   |
|                                                                  |
+- THREAD UTILIZATION ----------------------------------------------+ << updated width
| 01: [##+++++=======--%%%                                        ] |
| 02: [+======---------------                                     ] |
| 03: [+++-------*@                                               ] |
| 04: [=--*****                                                   ] |
+-------------------------------------------------------------------+
**/
