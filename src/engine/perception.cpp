#include "perception.hpp"

#include <fstream>
#include <stdio.h>

#ifdef HAS_CPUID
#include <cpuid.h>
#endif


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
    // util::declare_param(this, "debug_img_topic", dbg_img_topic, "perception/debug/image");
    this->debug_img_pub = this->img_transport.advertise("debug_img_topic", 1);
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

}

void PerceptionNode::CameraSubscriber::info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info)
{

}


void PerceptionNode::getParams()
{
    util::declare_param(this, "map_frame_id", this->map_frame, "map");
    util::declare_param(this, "odom_frame_id", this->odom_frame, "odom");
    util::declare_param(this, "base_frame_id", this->base_frame, "base_link");

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


void PerceptionNode::scan_callback(const sensor_msgs::msg::ConstSharedPtr& scan)
{

}

void PerceptionNode::imu_callback(const sensor_msgs::msg::SharedPtr imu)
{

}