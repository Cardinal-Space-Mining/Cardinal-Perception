#include "perception.hpp"
#include "imu_transform.hpp"

#include <sstream>
#include <fstream>
#include <stdio.h>
#include <iomanip>

#ifdef HAS_CPUID
#include <cpuid.h>
#endif

#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <opencv2/imgproc.hpp>


PerceptionNode::PerceptionNode() :
    Node("cardinal_perception"),
    tf_buffer{ std::make_shared<rclcpp::Clock>(RCL_ROS_TIME) },
    tf_linstener{ tf_buffer },
    tf_broadcaster{ *this },
    img_transport{ std::shared_ptr<PerceptionNode>(this, [](auto*){}) },
    lidar_odom{ this },
    tag_detection{ this }
{
    this->getParams();
    this->initMetrics();

    this->scan_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "pointcloud", 1, std::bind(&PerceptionNode::scan_callback, this, std::placeholders::_1));
    this->imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu", 1, std::bind(&PerceptionNode::imu_callback, this, std::placeholders::_1));

    std::vector<std::string> img_topics, info_topics;
    util::declare_param(this, "img_topics", img_topics, {});
    util::declare_param(this, "info_topics", info_topics, {});

    const size_t n_img = img_topics.size(), n_info = info_topics.size();
    if(n_img > 0 && n_img <= n_info)
    {
        this->camera_subs.reserve(n_img);
        for(size_t i = 0; i < n_img; i++)
        {
            this->camera_subs.emplace_back(this, img_topics[i], info_topics[i]);
        }
    }

    // std::string dbg_img_topic;
    // util::declare_param(this, "debug_img_topic", dbg_img_topic, "cardinal_perception/debug/image");
    this->debug_img_pub = this->img_transport.advertise("cardinal_perception/debug/image", 1);
    this->filtered_scan_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_scan");
}


PerceptionNode::CameraSubscriber::CameraSubscriber(
    PerceptionNode* inst,
    const std::string& img_topic,
    const std::string& info_topic
) :
    pnode{ inst }
{
    if(!this->pnode) return;

    this->image_sub = this->pnode->img_transport.subscribe( img_topic, 1,
        std::bind(&PerceptionNode::CameraSubscriber::img_callback, this, std::placeholders::_1) );
    this->info_sub = this->pnode->create_subscription<sensor_msgs::msg::CameraInfo>( info_topic, 10,
        std::bind(&PerceptionNode::CameraSubscriber::info_callback, this, std::placeholders::_1) );
}

void PerceptionNode::CameraSubscriber::img_callback(const sensor_msgs::msg::Image::ConstPtr& img)
{
    std::vector<TagDetection::Ptr> detections;
    this->pnode->tag_detection.processImg(img, *this, detections);
    this->state.any_new_frames = true;

    // filter + hard realign
    // publish tf
    // cache detections for DLO refinement

    this->handleStatusUpdate();
    this->handleDebugFrame();
}

void PerceptionNode::CameraSubscriber::info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info)
{
    if( !this->valid_calib &&
        info->k.size() == 9 &&
        info->d.size() >= 5 )
    {
        this->calibration = cv::Mat(info->k, true).reshape(0, 3);
        this->distortion = cv::Mat(info->d, true).reshape(0, 1);

        this->valid_calib_data = true;

        // log
    }

    this->handleStatusUpdate();
    this->handleDebugFrame();
}


void PerceptionNode::getParams()
{
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
    util::declare_param(this, "tag_filtering.max_linear_diff_velocity", this->tag_filtering.thresh_max_linear_diff_velocity, 5.);
    util::declare_param(this, "tag_filtering.max_angular_diff_velocity", this->tag_filtering.thresh_max_angular_diff_velocity, 5.);
    util::declare_param(this, "tag_filtering.min_tags_per_range", this->tag_filtering.thresh_min_tags_per_range, 0.5);
    util::declare_param(this, "tag_filtering.max_rms_per_tag", this->tag_filtering.thresh_max_rms_per_tag, 0.1);
    util::declare_param(this, "tag_filtering.min_sum_pix_area", this->tag_filtering.thresh_min_pix_area, 10000.);

    util::declare_param(this, "tag_filtering.covariance.linear_base_coeff", this->tag_filtering.covariance_linear_base_coeff, 0.001);
    util::declare_param(this, "tag_filtering.covariance.linear_range_coeff", this->tag_filtering.covariance_linear_range_coeff, 0.001);
    util::declare_param(this, "tag_filtering.covariance.angular_base_coeff", this->tag_filtering.covariance_angular_base_coeff, 0.001);
    util::declare_param(this, "tag_filtering.covariance.angular_range_coeff", this->tag_filtering.covariance_angular_range_coeff, 0.001);
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

void PerceptionNode::handleStatusUpdate()
{
    // try lock mutex
    if(this->state.print_mtx.try_lock())
    {
        // check frequency
        auto _tp = std::chrono::system_clock::now();
        if(util::toFloatSeconds(_tp - this->state.last_print_time) > (1. / this->param.status_max_print_freq))
        {
            this->state.last_print_time = _tp;
            std::ostringstream msg;

            // read stats from /proc
            double vm_usage = 0., resident_set = 0., cpu_percent = -1.;
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
                vm_usage = vsize / 1024.;
                resident_set = rss * page_size_kg;

                struct tms _sample;
                clock_t now = times(&_sample);
                if( now > this->metrics.last_cpu &&
                    _sample.tms_stime > this->metrics.last_sys_cpu &&
                    _sample.tms_utime > this->metrics.last_user_cpu )
                {
                    cpu_percent =
                        ( (_sample.tms_stime - this->metrics.last_sys_cpu) + (_sample.tms_utime - this->last_user_cpu) ) *
                        ( 100. / (now - this->metrics.last_cpu) / this->metrics.numProcessors );
                }
                this->metrics.last_cpu = now;
                this->metrics.last_sys_cpu = _sample.tms_stime;
                this->metrics.last_user_cpu = _sample.tms_utime;

                this->metrics.avg_cpu_percent = (this->metrics.avg_cpu_percent * this->metrics.avg_cpu_samples + cpu_percent) / (this->metrics.avg_cpu_samples + 1);
                this->metrics.avg_cpu_samples++;
                if(cpu_percent > this->metrics.max_cpu_percent) this->metrics.max_cpu_percent = cpu_percent;
            }

            msg << std::setprecision(2) << std::fixed << std::right << std::setfill(' ');
            msg << "+-------------------------------------------------------------------+\n"
                   "| =================== Cardinal Perception v0.0.1 ================== |\n"
                   "+- RESOURCES -------------------------------------------------------+\n"
                   "|                      ::  Current  |  Average  |  Maximum          |\n";
            msg << "|      CPU Utilization ::  " << std::setw(5) << cpu_percent
                                                << " %  |  " << std::setw(5) << this->metrics.avg_cpu_percent
                                                            << " %  |  " << std::setw(5) << this->metrics.max_cpu_percent
                                                                         << " %         |\n";
            msg << "|       RAM Allocation ::  " << std::setfill(' ') << std::setw(5) << resident_set
                                                << " MB |                               |\n";
            msg << "|        Total Threads ::  " << std::setfill(' ') << std::setw(5) << num_threads
                                                << "    |                               |\n";
            msg << "|                                                                   |\n"
                << "+- CALLBACKS -------------------------------------------------------+\n"
                << "|                        Comp. Time | Avg. Time | Max Time | Total  |\n";
            msg << std::setprecision(1) << std::fixed << std::right << std::setfill(' ');
            this->metrics.info_thread.mtx.lock();
            msg << "|  Info CB (" << std::setw(5) << this->metrics.info_thread.avg_call_freq
                                 << " Hz) ::  " << std::setw(5) << this->metrics.info_thread.last_comp_time * 1000.
                                               << " ms  | " << std::setw(5) << this->metrics.info_thread.avg_comp_time * 1000.
                                                           << " ms  | " << std::setw(5) << this->metrics.info_thread.max_comp_time * 1000.
                                                                       << " ms | " << std::setw(6) << this->metrics.info_thread.avg_com_samples
                                                                                   << " |\n";
            this->metrics.info_thread.mtx.unlock();
            this->metrics.img_thread.mtx.lock();
            msg << "| Image CB (" << std::setw(5) << this->metrics.img_thread.avg_call_freq
                                 << " Hz) ::  " << std::setw(5) << this->metrics.img_thread.last_comp_time * 1000.
                                               << " ms  | " << std::setw(5) << this->metrics.img_thread.avg_comp_time * 1000.
                                                           << " ms  | " << std::setw(5) << this->metrics.img_thread.max_comp_time * 1000.
                                                                       << " ms | " << std::setw(6) << this->metrics.img_thread.avg_com_samples
                                                                                   << " |\n";
            this->metrics.img_thread.mtx.unlock();
            this->metrics.imu_thread.mtx.lock();
            msg << "|   IMU CB (" << std::setw(5) << this->metrics.imu_thread.avg_call_freq
                                 << " Hz) ::  " << std::setw(5) << this->metrics.imu_thread.last_comp_time * 1000.
                                               << " ms  | " << std::setw(5) << this->metrics.imu_thread.avg_comp_time * 1000.
                                                           << " ms  | " << std::setw(5) << this->metrics.imu_thread.max_comp_time * 1000.
                                                                       << " ms | " << std::setw(6) << this->metrics.imu_thread.avg_com_samples
                                                                                   << " |\n";
            this->metrics.imu_thread.mtx.unlock();
            this->metrics.scan_thread.mtx.lock();
            msg << "|  Scan CB (" << std::setw(5) << this->metrics.scan_thread.avg_call_freq
                                 << " Hz) ::  " << std::setw(5) << this->metrics.scan_thread.last_comp_time * 1000.
                                               << " ms  | " << std::setw(5) << this->metrics.scan_thread.avg_comp_time * 1000.
                                                           << " ms  | " << std::setw(5) << this->metrics.scan_thread.max_comp_time * 1000.
                                                                       << " ms | " << std::setw(6) << this->metrics.scan_thread.avg_com_samples
                                                                                   << " |\n";
            this->metrics.scan_thread.mtx.unlock();
            msg << "+-------------------------------------------------------------------+" << std::endl;

            // print
            printf("\033[2J\033[1;1H");
            RCLCPP_INFO(this->get_logger(), msg.str());
        }
        this->state.print_mtx.unlock();
    }
}

void PerceptionNode::handleDebugFrame()
{
    // try lock mutex
    if(this->state.any_new_frames && this->state.frames_mtx.try_lock())
    {
        // check frequency
        auto _tp = std::chrono::system_clock::now();
        if(util::toFloatSeconds(_tp - this->state.last_frames_time) > (1. / this->param.img_debug_max_pub_freq))
        {
            this->state.last_frames_time = _tp;

            // handle singular callback?
            // lock & collect frames
            std::vector<cv::Mat> frames;
            std::vector<std::unique_lock> locks;
            frames.reserve(this->camera_subs.size());
            locks.reserve(this->camera_subs.size());
            size_t max_height{ 0 };

            for(CameraSubscriber& s : this->camera_subs)
            {
                std::unique_lock _lock;
                cv::Mat& _frame = s.dbg_frame.B(_lock);
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
        this->state.frames_mtx.unlock();
    }
}


void PerceptionNode::scan_callback(const sensor_msgs::msg::ConstSharedPtr& scan)
{
    try
    {
        sensor_msgs::msg::PointCloud2::SharedPtr scan_ = std::make_shared<sensor_msgs::msg::PointCloud2>();

        auto tf = this->tf_buffer.lookupTransform(
            this->base_frame,
            scan->header.frame_id,
            util::toTf2TimePoint(scan->header.stamp));

        tf2::doTransform(*scan, *scan_, tf);

        this->lidar_odom.processScan(scan_);
    }
    catch(const std::exception& e)
    {
        // fail
    }

    // refine cached tag detections
    // publish tf
    // mapping -- or send to other thread

    this->handleStatusUpdate();
    this->handleDebugFrame();
}

void PerceptionNode::imu_callback(const sensor_msgs::msg::SharedPtr imu)
{
    try
    {
        auto tf = this->tf_buffer.lookupTransform(
            this->base_frame,
            imu->header.frame_id,
            util::toTf2TimePoint(imu->header.stamp));

        tf2::doTransform(*imu, *imu, tf);

        this->lidar_odom.processImu(imu);
    }
    catch(const std::exception& e)
    {
        // fail
    }

    this->handleStatusUpdate();
    this->handleDebugFrame();
}


void PerceptionNode::ThreadMetrics::addSample(
    const std::chrono::system_clock::time_point& start,
    const std::chrono::system_clock::time_point& end)
{
    this->mtx.lock();
    {
        const double
            call_diff = util::toFloatSeconds(start - this->last_call_time),
            comp_time = util::toFloatSeconds(start - end);
        this->last_call_time = start;
        this->last_comp_time = comp_time;

        this->avg_comp_time = (this->avg_comp_time * this->avg_comp_samples + comp_time) / (this->avg_comp_samples + 1);
        this->avg_comp_samples++;
        this->avg_call_freq = (this->avg_call_freq * this->avg_freq_samples + (1. / call_diff)) / (this->avg_freq_samples + 1);
        this->avg_freq_samples++;

        if(comp_time > this->max_comp_time) this->max_comp_time = comp_time;
    }
    this->mtx.unlock();
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
+------------------------------------------------------------------+
**/
