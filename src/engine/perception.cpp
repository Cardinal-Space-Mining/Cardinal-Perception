#include "./perception.hpp"
#include "imu_transform.hpp"

#include <sstream>
#include <fstream>
#include <stdio.h>
#include <iomanip>

#ifdef HAS_CPUID
#include <cpuid.h>
#endif

#include <boost/algorithm/string.hpp>

#ifdef USE_CV_BRIDGE_HPP
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <opencv2/imgproc.hpp>


using namespace util::geom::cvt::ops;


PerceptionNode::PerceptionNode() :
    Node("cardinal_perception"),
    tf_buffer{ std::make_shared<rclcpp::Clock>(RCL_ROS_TIME) },
    tf_listener{ tf_buffer },
    tf_broadcaster{ *this },
    img_transport{ std::shared_ptr<PerceptionNode>(this, [](auto*){}) },
    mt_callback_group{ this->create_callback_group(rclcpp::CallbackGroupType::Reentrant) },
    metrics_pub{ this, "/perception_debug/", 1 },
    pose_pub{ this, "/perception_debug/", 1 },
    lidar_odom{ this },
    tag_detection{ this },
    trajectory_filter{}
{
    // RCLCPP_INFO(this->get_logger(), "CONSTRUCTOR INIT");

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

    this->scan_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        scan_topic, rclcpp::SensorDataQoS{},
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan){ this->scan_callback(scan); }, ops);
    this->imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, rclcpp::SensorDataQoS{},
        [this](const sensor_msgs::msg::Imu::SharedPtr imu){ this->imu_callback(imu); }, ops);

    std::vector<std::string> img_topics, info_topics;
    util::declare_param(this, "img_topics", img_topics, {});
    util::declare_param(this, "info_topics", info_topics, {});

    const size_t n_img = img_topics.size(), n_info = info_topics.size();
    if(n_img > 0 && n_img <= n_info)
    {
        this->camera_subs.resize(n_img);    // DO NOT CALL RESERVE AND SUBSEQUENTLY PUSH BACK INSTANCES!!!
        for(size_t i = 0; i < n_img; i++)
        {
            this->camera_subs[i].initialize(this, img_topics[i], info_topics[i]);
        }
    }

    this->debug_img_pub = this->img_transport.advertise("debug_img", rmw_qos_profile_sensor_data.depth);
    this->filtered_scan_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_scan", rclcpp::SensorDataQoS{});
    this->path_pub = this->create_publisher<nav_msgs::msg::Path>("path", rclcpp::SensorDataQoS{});

    double trajectory_filter_params[6];
    util::declare_param(this, "trajectory_filter.window_duration", trajectory_filter_params[0], 0.5);
    util::declare_param(this, "trajectory_filter.min_sample_duration", trajectory_filter_params[1], 0.3);
    util::declare_param(this, "trajectory_filter.thresh.avg_linear_error", trajectory_filter_params[2], 2e-2);
    util::declare_param(this, "trajectory_filter.thresh.avg_angular_error", trajectory_filter_params[3], 5e-2);
    util::declare_param(this, "trajectory_filter.thresh.min_linear_variance", trajectory_filter_params[4], 1e-5);
    util::declare_param(this, "trajectory_filter.thresh.min_angular_variance", trajectory_filter_params[5], 1e-5);
    this->trajectory_filter.applyParams(
        trajectory_filter_params[0],
        trajectory_filter_params[1],
        trajectory_filter_params[2],
        trajectory_filter_params[3],
        trajectory_filter_params[4],
        trajectory_filter_params[5] );

    // RCLCPP_INFO(this->get_logger(), "CONSTRUCTOR EXIT");
}


void PerceptionNode::CameraSubscriber::initialize(
    PerceptionNode* inst,
    const std::string& img_topic,
    const std::string& info_topic)
{
    if(!inst) return;
    this->pnode = inst;

    rclcpp::SubscriptionOptions ops{};
    ops.callback_group = this->pnode->mt_callback_group;

    this->image_sub = this->pnode->img_transport.subscribe(
        img_topic, rmw_qos_profile_sensor_data.depth,
        [this](const image_transport::ImageTransport::ImageConstPtr& img){ this->img_callback(img); },
        image_transport::ImageTransport::VoidPtr(), nullptr, ops);
    this->info_sub = this->pnode->create_subscription<sensor_msgs::msg::CameraInfo>(
        info_topic, rclcpp::SensorDataQoS{},
        [this](const image_transport::ImageTransport::CameraInfoConstPtr& info){ this->info_callback(info); }, ops);
}
PerceptionNode::CameraSubscriber::CameraSubscriber(
    const CameraSubscriber& ref
) :
    pnode{ ref.pnode },
    // image_sub{ ref.image_sub },
    // info_sub{ ref.info_sub },
    dbg_frame{ ref.dbg_frame },
    calibration{ ref.calibration.clone() },
    distortion{ ref.distortion.clone() },
    valid_calib{ ref.valid_calib.load() }
{}

void PerceptionNode::CameraSubscriber::img_callback(const sensor_msgs::msg::Image::ConstSharedPtr& img)
{
    // RCLCPP_INFO(this->pnode->get_logger(), "IMG_CALLBACK -- %s", (std::stringstream{} << std::this_thread::get_id()).str().c_str());
    auto _start = std::chrono::system_clock::now();

    std::vector<TagDetection::Ptr> detections;
    this->pnode->tag_detection.processImg(img, *this, detections);
    this->pnode->state.any_new_frames = true;

    // if(detections.size() > 0)
    // {
    //     this->pnode->trajectory_filter.addMeasurement(detections, detections[0]->time_point);
    // }

    // RCLCPP_INFO(this->pnode->get_logger(), "IMG_CALLBACK DONE PROCESSING -- %s", (std::stringstream{} << std::this_thread::get_id()).str().c_str());

    // filter
    TagDetection::Ptr best_detection = nullptr;
    double best_score = 0.;
    {
        for(auto& detection : detections)
        {
            const double
                tags_per_range = detection->num_tags / detection->avg_range,
                rms_per_tag = detection->rms / detection->num_tags;
            const bool
                in_bounds = this->pnode->tag_filtering.filter_bbox.isEmpty() ||
                    this->pnode->tag_filtering.filter_bbox.contains(detection->pose.vec),
                tags_per_range_ok = tags_per_range >= this->pnode->tag_filtering.thresh_min_tags_per_range,
                rms_per_tag_ok = rms_per_tag <= this->pnode->tag_filtering.thresh_max_rms_per_tag,
                pix_area_ok = detection->pix_area >= this->pnode->tag_filtering.thresh_min_pix_area;

            if(in_bounds && tags_per_range_ok && rms_per_tag_ok && pix_area_ok)
            {
                const double score = detection->rms;

                // update best
                if(!best_detection || score < best_score)
                {
                    best_detection = detection;
                    best_score = score;
                }
            }
            else
            {
                detection = nullptr;
            }
        }
    }
    if(best_detection)
    {
        this->pnode->trajectory_filter.addMeasurement(best_detection, best_detection->time_point);
    }

    // TODO: handle initialization case where we don't have odometry yet
    // if(!this->pnode->state.dlo_in_progress && best_detection)
    // {
    //     // hard realign + publish tf

    //     this->pnode->state.tf_mtx.lock();

    //     this->pnode->state.map_tf.tf = (Eigen::Translation3d{ best_detection->translation } * best_detection->rotation) *
    //         (this->pnode->state.odom_tf.pose.quat.inverse() * Eigen::Translation3d{ -this->pnode->state.odom_tf.pose.vec });
    //     this->pnode->state.map_tf.pose << this->pnode->state.map_tf.tf;

    //     this->pnode->sendTf(img->header.stamp, false);

    //     this->pnode->state.tf_mtx.unlock();
    // }

    // cache detections for DLO refinement
    // this->pnode->state.alignment_mtx.lock();
    // for(const auto& detection : detections)
    // {
    //     if(detection) this->pnode->alignment_queue.push_front(detection);
    // }
    // this->pnode->state.alignment_mtx.unlock();
    // if(best_detection)
    // {
    //     // this->pnode->state.alignment_mtx.lock();
    //     // this->pnode->alignment_queue.push_front(best_detection);
    //     // this->pnode->state.alignment_mtx.unlock();
    //     // util::geom::Pose3d p;
    //     // p.vec = best_detection->translation;
    //     // p.quat = best_detection->rotation;

    //     this->pnode->trajectory_filter.addMeasurement(best_detection->pose, best_detection->time_point);
    // }

    auto _end = std::chrono::system_clock::now();
    const double _dt = this->pnode->metrics.img_thread.addSample(_start, _end);
    this->pnode->appendThreadProcTime(ProcType::IMG_CB, _dt);

    this->pnode->handleStatusUpdate();
    this->pnode->handleDebugFrame();

    // RCLCPP_INFO(this->pnode->get_logger(), "IMG_CALLBACK EXIT -- %s", (std::stringstream{} << std::this_thread::get_id()).str().c_str());
}

void PerceptionNode::CameraSubscriber::info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info)
{
    // RCLCPP_INFO(this->pnode->get_logger(), "INFO_CALLBACK");
    auto _start = std::chrono::system_clock::now();

    if( !this->valid_calib &&
        info->k.size() == 9 &&
        info->d.size() >= 5 )
    {
        this->calibration = cv::Mat(info->k, true).reshape(0, 3);
        this->distortion = cv::Mat(info->d, true).reshape(0, 1);

        this->valid_calib = true;

        // log
    }

    auto _end = std::chrono::system_clock::now();
    const double _dt = this->pnode->metrics.info_thread.addSample(_start, _end);
    this->pnode->appendThreadProcTime(ProcType::INFO_CB, _dt);

    this->pnode->handleStatusUpdate();
    this->pnode->handleDebugFrame();

    // RCLCPP_INFO(this->pnode->get_logger(), "INFO_CALLBACK EXIT");
}


void PerceptionNode::getParams()
{
    this->declare_parameter("image_transport", "raw");

    util::declare_param(this, "map_frame_id", this->map_frame, "map");
    util::declare_param(this, "odom_frame_id", this->odom_frame, "odom");
    util::declare_param(this, "base_frame_id", this->base_frame, "base_link");

    util::declare_param(this, "debug.status_max_print_freq", this->param.status_max_print_freq, 10.);
    util::declare_param(this, "debug.img_max_pub_freq", this->param.img_debug_max_pub_freq, 30.);

    std::vector<double> min, max;
    util::declare_param(this, "tag_filtering.bounds_min", min, {});
    util::declare_param(this, "tag_filtering.bounds_max", max, {});
    if(min.size() > 2 && max.size() > 2)
    {
        this->tag_filtering.filter_bbox = Eigen::AlignedBox3d{ *reinterpret_cast<Eigen::Vector3d*>(min.data()), *reinterpret_cast<Eigen::Vector3d*>(max.data()) };
    }

    util::declare_param(this, "tag_filtering.fitness.oob_weight", this->tag_filtering.fitness_oob_weight, 100.);
    util::declare_param(this, "tag_filtering.fitness.rms_weight", this->tag_filtering.fitness_rms_weight, 10.0);
    util::declare_param(this, "tag_filtering.thresh.max_linear_diff_velocity", this->tag_filtering.thresh_max_linear_diff_velocity, 5.);
    util::declare_param(this, "tag_filtering.thresh.max_angular_diff_velocity", this->tag_filtering.thresh_max_angular_diff_velocity, 5.);
    util::declare_param(this, "tag_filtering.thresh.min_tags_per_range", this->tag_filtering.thresh_min_tags_per_range, 0.5);
    util::declare_param(this, "tag_filtering.thresh.max_rms_per_tag", this->tag_filtering.thresh_max_rms_per_tag, 0.1);
    util::declare_param(this, "tag_filtering.thresh.min_sum_pix_area", this->tag_filtering.thresh_min_pix_area, 10000.);

    util::declare_param(this, "tag_filtering.covariance.linear_base_coeff", this->tag_filtering.covariance_linear_base_coeff, 0.001);
    util::declare_param(this, "tag_filtering.covariance.linear_range_coeff", this->tag_filtering.covariance_linear_range_coeff, 0.001);
    util::declare_param(this, "tag_filtering.covariance.angular_base_coeff", this->tag_filtering.covariance_angular_base_coeff, 0.001);
    util::declare_param(this, "tag_filtering.covariance.angular_range_coeff", this->tag_filtering.covariance_angular_range_coeff, 0.001);
    util::declare_param(this, "tag_filtering.covariance.linear_rms_per_tag_coeff", this->tag_filtering.covariance_linear_rms_per_tag_coeff, 0.01);
    util::declare_param(this, "tag_filtering.covariance.angular_rms_per_tag_coeff", this->tag_filtering.covariance_angular_rms_per_tag_coeff, 0.01);
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

    // Eigen::Isometry3d full_tf = this->state.map_tf.pose.vec_trl() * (this->state.odom_tf.tf * this->state.map_tf.pose.quat);    // rotate then add translation

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
            this->metrics.info_thread.mtx.lock();
            msg << "|  Info CB (" << std::setw(5) << 1. / this->metrics.info_thread.avg_call_delta
                                 << " Hz) ::  " << std::setw(5) << this->metrics.info_thread.last_comp_time * 1e6
                                               << " us  | " << std::setw(5) << this->metrics.info_thread.avg_comp_time * 1e6
                                                           << " us  | " << std::setw(5) << this->metrics.info_thread.max_comp_time * 1e3
                                                                       << " ms | " << std::setw(6) << this->metrics.info_thread.samples
                                                                                   << " |\n";
            this->metrics.info_thread.mtx.unlock();
            this->metrics.img_thread.mtx.lock();
            msg << "| Image CB (" << std::setw(5) << 1. / this->metrics.img_thread.avg_call_delta
                                 << " Hz) ::  " << std::setw(5) << this->metrics.img_thread.last_comp_time * 1e3
                                               << " ms  | " << std::setw(5) << this->metrics.img_thread.avg_comp_time * 1e3
                                                           << " ms  | " << std::setw(5) << this->metrics.img_thread.max_comp_time * 1e3
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
                static constexpr char const* CHAR_VARS = "IiSuFMx"; // Img, info, Scan, imu, debug Frame, Metrics, misc
                static constexpr char const* COLORS[7] =
                {
                    "\033[38;5;49m",
                    "\033[38;5;11m",
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

void PerceptionNode::handleDebugFrame()
{
    // RCLCPP_INFO(this->get_logger(), "DEBUG FRAME INIT");
    // try lock mutex
    if(this->state.any_new_frames && this->state.frames_mtx.try_lock())
    {
        // check frequency
        auto _tp = std::chrono::system_clock::now();
        const double _dt = util::toFloatSeconds(_tp - this->state.last_frames_time);
        if(_dt > (1. / this->param.img_debug_max_pub_freq))
        {
            this->state.last_frames_time = _tp;
            this->state.any_new_frames = false;

            // RCLCPP_INFO(this->get_logger(), "DEBUG FRAME RUNNING...");

            // lock & collect frames
            std::vector<cv::Mat> frames;
            std::vector<std::unique_lock<std::mutex>> locks;
            frames.reserve(this->camera_subs.size());
            locks.reserve(this->camera_subs.size());
            int max_height{ 0 };

            for(CameraSubscriber& s : this->camera_subs)
            {
                std::unique_lock<std::mutex> _lock;
                // cv::Mat& _frame = s.dbg_frame.B(_lock);
                cv::Mat& _frame = s.dbg_frame.lock(_lock);
                // s.dbg_frame.try_spin();     // hint a spin since we are done with all the current frames?
                if(_frame.size().area() > 0)
                {
                    if(_frame.size().height > max_height) max_height = _frame.size().height;
                    frames.push_back(_frame);
                    locks.emplace_back(std::move(_lock));
                }
            }

            for(size_t i = 0; i < frames.size(); i++)
            {
                // resize frames, unlock on dereferences
                if(frames[i].size().height < max_height)
                {
                    double ratio = max_height / frames[i].size().height;
                    cv::Mat _resized;
                    cv::resize(frames[i], _resized, cv::Size{}, ratio, ratio, cv::INTER_AREA);
                    frames[i] = _resized;
                    locks[i].unlock();
                }
            }

            // combine & output
            if(frames.size() > 0)
            {
                try
                {
                    thread_local cv::Mat pub;
                    if(frames.size() > 1)
                    {
                        cv::hconcat(frames, pub);
                    }
                    else
                    {
                        pub = frames[0];
                    }

                    std_msgs::msg::Header hdr;
                    hdr.frame_id = this->base_frame;
                    hdr.stamp = this->get_clock()->now();

                    this->debug_img_pub.publish(cv_bridge::CvImage(hdr, "bgr8", pub).toImageMsg());
                }
                catch(const std::exception& e)
                {
                    // TODO
                }
            }
        }
        this->appendThreadProcTime(ProcType::HANDLE_DBG_FRAME, util::toFloatSeconds(std::chrono::system_clock::now() - _tp));
        this->state.frames_mtx.unlock();
    }
    // RCLCPP_INFO(this->get_logger(), "DEBUG FRAME EXIT");
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
    this->handleDebugFrame();

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
    this->handleDebugFrame();

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
