#include "perception.hpp"
#include "imu_transform.hpp"

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

    this->metrics.lastCPU = times(&timeSample);
    this->metrics.lastSysCPU = timeSample.tms_stime;
    this->metrics.lastUserCPU = timeSample.tms_utime;

    file = fopen("/proc/cpuinfo", "r");
    this->metrics.numProcessors = 0;
    while(fgets(line, 128, file) != NULL)
    {
        if(strncmp(line, "processor", 9) == 0)
            this->metrics.numProcessors++;
    }
    fclose(file);
}

void PerceptionNode::handleStatusUpdate()
{
    // try lock mutex
    {
        // check frequency
        {
            // read stats from /proc
            // lock per-callback stats buffers (cyclebuffer?) -- read and swap?
            // print
        }
    }

}

void PerceptionNode::handleDebugFrame()
{
    // try lock mutex
    {
        // check frequency
        {
            // lock & collect frames
            // combine & output
        }
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
