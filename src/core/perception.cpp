/*******************************************************************************
*   Copyright (C) 2024-2025 Cardinal Space Mining Club                         *
*                                                                              *
*                                 ;xxxxxxx:                                    *
*                                ;$$$$$$$$$       ...::..                      *
*                                $$$$$$$$$$x   .:::::::::::..                  *
*                             x$$$$$$$$$$$$$$::::::::::::::::.                 *
*                         :$$$$$&X;      .xX:::::::::::::.::...                *
*                 .$$Xx++$$$$+  :::.     :;:   .::::::.  ....  :               *
*                :$$$$$$$$$  ;:      ;xXXXXXXXx  .::.  .::::. .:.              *
*               :$$$$$$$$: ;      ;xXXXXXXXXXXXXx: ..::::::  .::.              *
*              ;$$$$$$$$ ::   :;XXXXXXXXXXXXXXXXXX+ .::::.  .:::               *
*               X$$$$$X : +XXXXXXXXXXXXXXXXXXXXXXXX; .::  .::::.               *
*                .$$$$ :xXXXXXXXXXXXXXXXXXXXXXXXXXXX.   .:::::.                *
*                 X$$X XXXXXXXXXXXXXXXXXXXXXXXXXXXXx:  .::::.                  *
*                 $$$:.XXXXXXXXXXXXXXXXXXXXXXXXXXX  ;; ..:.                    *
*                 $$& :XXXXXXXXXXXXXXXXXXXXXXXX;  +XX; X$$;                    *
*                 $$$: XXXXXXXXXXXXXXXXXXXXXX; :XXXXX; X$$;                    *
*                 X$$X XXXXXXXXXXXXXXXXXXX; .+XXXXXXX; $$$                     *
*                 $$$$ ;XXXXXXXXXXXXXXX+  +XXXXXXXXx+ X$$$+                    *
*               x$$$$$X ;XXXXXXXXXXX+ :xXXXXXXXX+   .;$$$$$$                   *
*              +$$$$$$$$ ;XXXXXXx;;+XXXXXXXXX+    : +$$$$$$$$                  *
*               +$$$$$$$$: xXXXXXXXXXXXXXX+      ; X$$$$$$$$                   *
*                :$$$$$$$$$. +XXXXXXXXX;      ;: x$$$$$$$$$                    *
*                ;x$$$$XX$$$$+ .;+X+      :;: :$$$$$xX$$$X                     *
*               ;;;;;;;;;;X$$$$$$$+      :X$$$$$$&.                            *
*               ;;;;;;;:;;;;;x$$$$$$$$$$$$$$$$x.                               *
*               :;;;;;;;;;;;;.  :$$$$$$$$$$X                                   *
*                .;;;;;;;;:;;    +$$$$$$$$$                                    *
*                  .;;;;;;.       X$$$$$$$:                                    *
*                                                                              *
*   Unless required by applicable law or agreed to in writing, software        *
*   distributed under the License is distributed on an "AS IS" BASIS,          *
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
*   See the License for the specific language governing permissions and        *
*   limitations under the License.                                             *
*                                                                              *
*******************************************************************************/

#include "perception.hpp"

#include <sstream>
#include <iostream>

#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <imu_transform.hpp>
#include <cloud_ops.hpp>

#ifndef ENABLE_PRINT_STATUS
#define ENABLE_PRINT_STATUS 1
#endif

#ifndef LFD_PUBLISH_DEBUG
#define LFD_PUBLISH_DEBUG 1
#endif

#ifndef PUBLISH_FULL_MAP
#define PUBLISH_FULL_MAP 1
#endif

#if LFD_ENABLED
#define PERCEPTION_THREADS 4
#else
#define PERCEPTION_THREADS 3
#endif


using namespace util::geom::cvt::ops;

namespace csm
{
namespace perception
{

PerceptionNode::PerceptionNode() :
    Node("cardinal_perception_localization"),
    tf_buffer{ std::make_shared<rclcpp::Clock>(RCL_ROS_TIME) },
    tf_listener{ tf_buffer },
    tf_broadcaster{ *this },
    lidar_odom{ *this },
    transform_sync{ this->tf_broadcaster },
    metrics_pub{ this, "/cardinal_perception/" },
    pose_pub{ this, "/poses/" },
    scan_pub{ this, "/cardinal_perception/" },
    thread_metrics_pub{ this, "/cardinal_perception/" }
{
    this->getParams();
    this->initPubSubs();
    this->transform_sync.setFrameIds(
        this->map_frame,
        this->odom_frame,
        this->base_frame );

    this->mt.threads.reserve(PERCEPTION_THREADS);
    this->mt.threads.emplace_back(&PerceptionNode::odometry_worker, this);
    this->mt.threads.emplace_back(&PerceptionNode::mapping_worker, this);
IF_LFD_ENABLED(
    this->mt.threads.emplace_back(&PerceptionNode::fiducial_worker, this); )
    this->mt.threads.emplace_back(&PerceptionNode::traversibility_worker, this);
}

PerceptionNode::~PerceptionNode()
{
    this->shutdown();
}


void PerceptionNode::shutdown()
{
    this->state.threads_running = false;

    this->mt.odometry_resources.notifyExit();
    this->mt.mapping_resources.notifyExit();
IF_LFD_ENABLED(
    this->mt.fiducial_resources.notifyExit(); )
    this->mt.traversibility_resources.notifyExit();

    for(auto& x : this->mt.threads) if(x.joinable()) x.join();
}



void PerceptionNode::getParams()
{
    util::declare_param(this, "map_frame_id", this->map_frame, "map");
    util::declare_param(this, "odom_frame_id", this->odom_frame, "odom");
    util::declare_param(this, "base_frame_id", this->base_frame, "base_link");

IF_TAG_DETECTION_ENABLED(
    util::declare_param(this, "use_tag_detections", this->param.use_tag_detections, -1); )
    // util::declare_param(this, "require_rebias_before_tf_pub", this->param.rebias_tf_pub_prereq, false);
    // util::declare_param(this, "require_rebias_before_scan_pub", this->param.rebias_scan_pub_prereq, false);
    util::declare_param(this, "metrics_pub_freq", this->param.metrics_pub_freq, 10.);

    util::declare_param(this, "publish_odometry_debug", this->param.publish_odom_debug, false);

    std::vector<double> _min, _max;
    util::declare_param(this, "cropbox_filter.min", _min, { 0., 0., 0. });
    util::declare_param(this, "cropbox_filter.max", _max, { 0., 0., 0. });
    this->param.crop_min = Eigen::Vector3f{ static_cast<float>(_min[0]), static_cast<float>(_min[1]), static_cast<float>(_min[2]) };
    this->param.crop_max = Eigen::Vector3f{ static_cast<float>(_max[0]), static_cast<float>(_max[1]), static_cast<float>(_max[2]) };
    this->param.use_crop_filter = this->param.crop_min != this->param.crop_max;

    double sample_window_s, filter_window_s, avg_linear_err_thresh, avg_angular_err_thresh,
        max_linear_deviation_thresh, max_angular_deviation_thresh;
    util::declare_param(this, "trajectory_filter.sampling_window_s", sample_window_s, 0.5);
    util::declare_param(this, "trajectory_filter.min_filter_window_s", filter_window_s, 0.3);
    util::declare_param(this, "trajectory_filter.thresh.avg_linear_error", avg_linear_err_thresh, 0.2);
    util::declare_param(this, "trajectory_filter.thresh.avg_angular_error", avg_angular_err_thresh, 0.1);
    util::declare_param(this, "trajectory_filter.thresh.max_linear_deviation", max_linear_deviation_thresh, 4e-2);
    util::declare_param(this, "trajectory_filter.thresh.max_angular_deviation", max_angular_deviation_thresh, 4e-2);
    this->transform_sync.trajectoryFilter().applyParams(
        sample_window_s,
        filter_window_s,
        avg_linear_err_thresh,
        avg_angular_err_thresh,
        max_linear_deviation_thresh,
        max_angular_deviation_thresh );

    double frustum_search_radius, radial_dist_thresh, delete_delta_coeff, delete_max_range, add_max_range, voxel_size;
    util::declare_param(this, "mapping.frustum_search_radius", frustum_search_radius, 0.01);
    util::declare_param(this, "mapping.radial_distance_thresh", radial_dist_thresh, 0.01);
    util::declare_param(this, "mapping.delete_delta_coeff", delete_delta_coeff, 0.1);
    util::declare_param(this, "mapping.delete_max_range", delete_max_range, 4.);
    util::declare_param(this, "mapping.add_max_range", add_max_range, 4.);
    util::declare_param(this, "mapping.voxel_size", voxel_size, 0.1);
    this->environment_map.applyParams(
        frustum_search_radius,
        radial_dist_thresh,
        delete_delta_coeff,
        0.,
        delete_max_range,
        add_max_range,
        voxel_size );

#if LFD_ENABLED
    double lfd_range_thresh, plane_distance, eps_angle, vox_res, max_remaining_proportion;
    int min_points_thresh{ 0 }, min_seg_points_thresh{ 0 };
    util::declare_param(this, "fiducial_detection.max_range", lfd_range_thresh, 2.);
    util::declare_param(this, "fiducial_detection.plane_distance_threshold", plane_distance, 0.005);
    util::declare_param(this, "fiducial_detection.plane_eps_thresh", eps_angle, 0.1);
    util::declare_param(this, "fiducial_detection.vox_resolution", vox_res, 0.03);
    util::declare_param(this, "fiducial_detection.remaining_points_thresh", max_remaining_proportion, 0.05);
    util::declare_param(this, "fiducial_detection.minimum_input_points", min_points_thresh, 100);
    util::declare_param(this, "fiducial_detection.minimum_segmented_points", min_seg_points_thresh, 15);
    this->fiducial_detector.applyParams(
        lfd_range_thresh,
        plane_distance,
        eps_angle,
        vox_res,
        static_cast<size_t>(min_points_thresh),
        static_cast<size_t>(min_seg_points_thresh),
        max_remaining_proportion );
#endif

    util::declare_param(this, "traversibility.chunk_horizontal_range", this->param.map_export_horizontal_range, 4.);
    util::declare_param(this, "traversibility.chunk_vertical_range", this->param.map_export_vertical_range, 1.);
}



void PerceptionNode::initPubSubs()
{
    std::string scan_topic, imu_topic;
    util::declare_param(this, "scan_topic", scan_topic, "scan");
    util::declare_param(this, "imu_topic", imu_topic, "imu");

    this->imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic,
        rclcpp::SensorDataQoS{},
        [this](sensor_msgs::msg::Imu::SharedPtr imu)
        {
            this->imu_worker(imu);
        }
    );
    this->scan_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        scan_topic,
        rclcpp::SensorDataQoS{},
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan)
        {
            this->mt.odometry_resources.updateAndNotify(scan);
        }
    );
#if TAG_DETECTION_ENABLED
    this->detections_sub = this->create_subscription<cardinal_perception::msg::TagsTransform>(
        "tags_detections",
        rclcpp::SensorDataQoS{},
        [this](const cardinal_perception::msg::TagsTransform::ConstSharedPtr& det)
        {
            this->detection_worker(det);
        }
    );
#endif

    this->filtered_scan_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                                            "filtered_scan", rclcpp::SensorDataQoS{} );
    this->map_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                                            "map_cloud", rclcpp::SensorDataQoS{} );
    this->velocity_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>(
                                            "odom_velocity", rclcpp::SensorDataQoS{});

    this->proc_metrics_pub = this->create_publisher<cardinal_perception::msg::ProcessMetrics>(
                                        "/cardinal_perception/process_metrics", rclcpp::SensorDataQoS{} );

    this->traj_filter_debug_pub = this->create_publisher<cardinal_perception::msg::TrajectoryFilterDebug>(
                                                "/cardinal_perception/trajectory_filter", rclcpp::SensorDataQoS{} );
}




void PerceptionNode::handleStatusUpdate()
{
    // try lock mutex
    if(!this->state.print_mtx.try_lock())
    {
        return;
    }

    // check frequency
    auto _tp = this->appendMetricStartTime(ProcType::HANDLE_METRICS);
    const double _dt = util::toFloatSeconds(_tp - this->state.last_print_time);
    if(_dt <= (1. / this->param.metrics_pub_freq))
    {
        this->appendMetricStopTime(ProcType::HANDLE_METRICS);
        this->state.print_mtx.unlock();
        return;
    }

    this->state.last_print_time = _tp;

    double resident_set_mb = 0.;
    size_t num_threads = 0;
    this->metrics.process_utilization.update();
    util::proc::getProcessStats(resident_set_mb, num_threads);

#if ENABLE_PRINT_STATUS
    std::ostringstream msg;

    msg << std::setprecision(2) << std::fixed << std::right << std::setfill(' ') << '\n';
    msg << "+-------------------------------------------------------------------+\n"
           "| =================== Cardinal Perception v0.5.0 ================== |\n"
           "+- RESOURCES -------------------------------------------------------+\n"
           "|                      ::  Current  |  Average  |  Maximum          |\n";
    msg << "|      CPU Utilization :: " << std::setw(6) << (this->metrics.process_utilization.last_cpu_percent)
                                        << " %  | " << std::setw(6) << this->metrics.process_utilization.avg_cpu_percent
                                                    << " %  |   " << std::setw(5) << this->metrics.process_utilization.max_cpu_percent
                                                                 << " %         |\n";
    msg << "|       RAM Allocation :: " << std::setw(6) << resident_set_mb
                                        << " MB |                               |\n";
    msg << "|        Total Threads ::  " << std::setw(5) << num_threads
                                        << "    |                               |\n";
    msg << "|                                                                   |\n"
        << "+- CALLBACKS -------------------------------------------------------+\n"
        << "|                        Comp. Time | Avg. Time | Max Time | Total  |\n";
    msg << std::setprecision(1) << std::fixed << std::right << std::setfill(' ');
    // this->metrics.imu_thread.mtx.lock();
    msg << "|   IMU CB (" << std::setw(5) << 1. / this->metrics.imu_thread.avg_call_delta
                          << " Hz) ::  " << std::setw(5) << this->metrics.imu_thread.last_comp_time * 1e6
                                       << " us  | " << std::setw(5) << this->metrics.imu_thread.avg_comp_time * 1e6
                                                   << " us  | " << std::setw(5) << this->metrics.imu_thread.max_comp_time * 1e3
                                                               << " ms | " << std::setw(6) << this->metrics.imu_thread.samples
                                                                           << " |\n";
    // this->metrics.imu_thread.mtx.unlock();
    // this->metrics.scan_thread.mtx.lock();
    msg << "|  SCAN CB (" << std::setw(5) << 1. / this->metrics.scan_thread.avg_call_delta
                          << " Hz) ::  " << std::setw(5) << this->metrics.scan_thread.last_comp_time * 1e3
                                       << " ms  | " << std::setw(5) << this->metrics.scan_thread.avg_comp_time * 1e3
                                                   << " ms  | " << std::setw(5) << this->metrics.scan_thread.max_comp_time * 1e3
                                                               << " ms | " << std::setw(6) << this->metrics.scan_thread.samples
                                                                           << " |\n";
    // this->metrics.scan_thread.mtx.unlock();
#if TAG_DETECTION_ENABLED
    // this->metrics.det_thread.mtx.lock();
    msg << "|   DET CB (" << std::setw(5) << 1. / this->metrics.det_thread.avg_call_delta
                          << " Hz) ::  " << std::setw(5) << this->metrics.det_thread.last_comp_time * 1e6
                                       << " us  | " << std::setw(5) << this->metrics.det_thread.avg_comp_time * 1e6
                                                   << " us  | " << std::setw(5) << this->metrics.det_thread.max_comp_time * 1e3
                                                               << " ms | " << std::setw(6) << this->metrics.det_thread.samples
                                                                           << " |\n";
    // this->metrics.det_thread.mtx.unlock();
#endif
    // this->metrics.mapping_thread.mtx.lock();
    msg << "|   MAP CB (" << std::setw(5) << 1. / this->metrics.mapping_thread.avg_call_delta
                          << " Hz) ::  " << std::setw(5) << this->metrics.mapping_thread.last_comp_time * 1e3
                                       << " ms  | " << std::setw(5) << this->metrics.mapping_thread.avg_comp_time * 1e3
                                                   << " ms  | " << std::setw(5) << this->metrics.mapping_thread.max_comp_time * 1e3
                                                               << " ms | " << std::setw(6) << this->metrics.mapping_thread.samples
                                                                           << " |\n";
    // this->metrics.mapping_thread.mtx.unlock();
#if LFD_ENABLED
    // this->metrics.fiducial_thread.mtx.lock();
    msg << "|   FID CB (" << std::setw(5) << 1. / this->metrics.fiducial_thread.avg_call_delta
                          << " Hz) ::  " << std::setw(5) << this->metrics.fiducial_thread.last_comp_time * 1e3
                                       << " ms  | " << std::setw(5) << this->metrics.fiducial_thread.avg_comp_time * 1e3
                                                   << " ms  | " << std::setw(5) << this->metrics.fiducial_thread.max_comp_time * 1e3
                                                               << " ms | " << std::setw(6) << this->metrics.fiducial_thread.samples
                                                                           << " |\n";
    // this->metrics.fiducial_thread.mtx.unlock();
#endif
    // this->metrics.trav_thread.mtx.lock();
    msg << "|  TRAV CB (" << std::setw(5) << 1. / this->metrics.trav_thread.avg_call_delta
                          << " Hz) ::  " << std::setw(5) << this->metrics.trav_thread.last_comp_time * 1e3
                                       << " ms  | " << std::setw(5) << this->metrics.trav_thread.avg_comp_time * 1e3
                                                   << " ms  | " << std::setw(5) << this->metrics.trav_thread.max_comp_time * 1e3
                                                               << " ms | " << std::setw(6) << this->metrics.trav_thread.samples
                                                                           << " |\n";
    // this->metrics.trav_thread.mtx.unlock();
    msg << "|                                                                   |\n"
           "+- THREAD UTILIZATION ----------------------------------------------+\n"
           "|                                                                   |\n";

    this->metrics.thread_procs_mtx.lock();
    size_t idx = 0;
    for(auto& p : this->metrics.thread_metric_durations)
    {
        static constexpr size_t NUM_PROC_TYPES = static_cast<size_t>(ProcType::NUM_ITEMS);

        static constexpr std::array<char, NUM_PROC_TYPES> CHAR_VARS =
        {
            'I',    // Imu
            'S',    // Scan
        #if TAG_DETECTION_ENABLED
            'D',    // Detection
        #endif
            'M',    // Mapping
        #if LFD_ENABLED
            'F',    // Fiducial
        #endif
            'T',    // Traversibility
            'X',    // metrics(x)
            'm'     // miscelaneous
        };
        static constexpr std::array<const char*, NUM_PROC_TYPES> COLORS =
        {
            "\033[38;5;117m",
            "\033[38;5;47m",
        #if TAG_DETECTION_ENABLED
            "\033[38;5;9m",
        #endif
            "\033[38;5;99m",
        #if LFD_ENABLED
            "\033[38;5;197m",
        #endif
            "\033[38;5;208m",
            "\033[38;5;80m",
            "\033[37m"
        };

        msg << "| " << std::setw(3) << idx++ << ": [";    // start -- 8 chars
        // fill -- 58 chars
        size_t avail_chars = 58;
        for(size_t x = 0; x < NUM_PROC_TYPES; x++)
        {
            auto& d = p.second[x];
            if(d.second > ClockType::time_point::min() && d.second < _tp)
            {
                d.first += std::chrono::duration<double>(_tp - d.second).count();
                d.second = _tp;
            }
            const double fn_chars = d.first / _dt * 58.;
            size_t n_chars = static_cast<size_t>(fn_chars > 0. ? std::max(fn_chars, 1.) : 0.);
            n_chars = std::min(n_chars, avail_chars);
            avail_chars -= n_chars;
            d.first = 0.;
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

    msg << "+-------------------------------------------------------------------+\n";

    std::cout << "\033[2J\033[1;1H" << std::endl;
    std::string msg_str = msg.str();
    RCLCPP_INFO(this->get_logger(), "%s", msg_str.c_str());
#endif

    this->publishMetrics(resident_set_mb, num_threads);

    this->appendMetricStopTime(ProcType::HANDLE_METRICS);
    this->state.print_mtx.unlock();
}



void PerceptionNode::publishMetrics(double mem_usage, size_t n_threads)
{
    cardinal_perception::msg::ProcessMetrics pm;
    pm.cpu_percent = static_cast<float>(this->metrics.process_utilization.last_cpu_percent);
    pm.avg_cpu_percent = static_cast<float>(this->metrics.process_utilization.avg_cpu_percent);
    pm.mem_usage_mb = static_cast<float>(mem_usage);
    pm.num_threads = static_cast<uint32_t>(n_threads);
    this->proc_metrics_pub->publish(pm);

    cardinal_perception::msg::ThreadMetrics tm;
    tm.delta_t = static_cast<float>(this->metrics.imu_thread.last_comp_time);
    tm.avg_delta_t = static_cast<float>(this->metrics.imu_thread.avg_comp_time);
    tm.avg_freq = static_cast<float>(1. / this->metrics.imu_thread.avg_call_delta);
    tm.iterations = this->metrics.imu_thread.samples;
    this->thread_metrics_pub.publish("imu_cb_metrics", tm);

    tm.delta_t = static_cast<float>(this->metrics.scan_thread.last_comp_time);
    tm.avg_delta_t = static_cast<float>(this->metrics.scan_thread.avg_comp_time);
    tm.avg_freq = static_cast<float>(1. / this->metrics.scan_thread.avg_call_delta);
    tm.iterations = this->metrics.scan_thread.samples;
    this->thread_metrics_pub.publish("scan_cb_metrics", tm);

#if TAG_DETECTION_ENABLED
    tm.delta_t = static_cast<float>(this->metrics.det_thread.last_comp_time);
    tm.avg_delta_t = static_cast<float>(this->metrics.det_thread.avg_comp_time);
    tm.avg_freq = static_cast<float>(1. / this->metrics.det_thread.avg_call_delta);
    tm.iterations = this->metrics.det_thread.samples;
    this->thread_metrics_pub.publish("det_cb_metrics", tm);
#endif

    tm.delta_t = static_cast<float>(this->metrics.mapping_thread.last_comp_time);
    tm.avg_delta_t = static_cast<float>(this->metrics.mapping_thread.avg_comp_time);
    tm.avg_freq = static_cast<float>(1. / this->metrics.mapping_thread.avg_call_delta);
    tm.iterations = this->metrics.mapping_thread.samples;
    this->thread_metrics_pub.publish("mapping_cb_metrics", tm);

#if LFD_ENABLED
    tm.delta_t = static_cast<float>(this->metrics.fiducial_thread.last_comp_time);
    tm.avg_delta_t = static_cast<float>(this->metrics.fiducial_thread.avg_comp_time);
    tm.avg_freq = static_cast<float>(1. / this->metrics.fiducial_thread.avg_call_delta);
    tm.iterations = this->metrics.fiducial_thread.samples;
    this->thread_metrics_pub.publish("fiducial_cb_metrics", tm);
#endif

    tm.delta_t = static_cast<float>(this->metrics.trav_thread.last_comp_time);
    tm.avg_delta_t = static_cast<float>(this->metrics.trav_thread.avg_comp_time);
    tm.avg_freq = static_cast<float>(1. / this->metrics.trav_thread.avg_call_delta);
    tm.iterations = this->metrics.trav_thread.samples;
    this->thread_metrics_pub.publish("trav_cb_metrics", tm);
}






#if TAG_DETECTION_ENABLED
void PerceptionNode::detection_worker(const cardinal_perception::msg::TagsTransform::ConstSharedPtr& detection_group)
{
    auto _start = this->appendMetricStartTime(ProcType::DET_CB);

    const geometry_msgs::msg::TransformStamped& tf = detection_group->estimated_tf;
    if( tf.header.frame_id == this->map_frame && tf.child_frame_id == this->base_frame &&
        (this->param.use_tag_detections > 0 || (this->param.use_tag_detections < 0 && detection_group->filter_mask >= 31)) )
    {
        TagDetection::Ptr td = std::make_shared<TagDetection>();
        td->pose << tf.transform;
        td->time_point = util::toFloatSeconds(tf.header.stamp);
        td->pix_area = detection_group->pix_area;
        td->avg_range = detection_group->avg_range;
        td->rms = detection_group->rms;
        td->num_tags = detection_group->num_tags;
        this->transform_sync.endMeasurementIterationSuccess(td, td->time_point);

        // RCLCPP_INFO(this->get_logger(), "[DETECTION CB]: Recv - Base delta: %f", util::toFloatSeconds(this->get_clock()->now()) - td->time_point);
    }

    auto _end = this->appendMetricStopTime(ProcType::DET_CB);
    this->metrics.det_thread.addSample(_start, _end);

    this->handleStatusUpdate();
}
#endif




void PerceptionNode::imu_worker(const sensor_msgs::msg::Imu::SharedPtr& imu)
{
    // RCLCPP_INFO(this->get_logger(), "IMU_CALLBACK");
    auto _start = this->appendMetricStartTime(ProcType::IMU_CB);

    try
    {
        auto tf = this->tf_buffer.lookupTransform(
            this->base_frame,
            imu->header.frame_id,
            util::toTf2TimePoint(imu->header.stamp));

        tf2::doTransform(*imu, *imu, tf);

        // this->lidar_odom.processImu(*imu);
        this->imu_samples.addSample(*imu);
    }
    catch(const std::exception& e)
    {
        RCLCPP_INFO(this->get_logger(), "[IMU CALLBACK]: Failed to process imu measurment.\n\twhat(): %s", e.what());
    }

    auto end = this->appendMetricStopTime(ProcType::IMU_CB);
    this->metrics.imu_thread.addSample(_start, end);

    this->handleStatusUpdate();

    // RCLCPP_INFO(this->get_logger(), "IMU_CALLBACK EXIT");
}




void PerceptionNode::odometry_worker()
{
    do
    {
        auto& scan = this->mt.odometry_resources.waitNewestResource();
        if(!this->state.threads_running.load()) return;

        auto start = this->appendMetricStartTime(ProcType::SCAN_CB);
        {
            this->scan_callback_internal(scan);
        }
        auto end = this->appendMetricStopTime(ProcType::SCAN_CB);
        this->metrics.scan_thread.addSample(start, end);

        this->handleStatusUpdate();
    }
    while(this->state.threads_running.load());
}



void PerceptionNode::mapping_worker()
{
    do
    {
        auto& buff = this->mt.mapping_resources.waitNewestResource();
        if(!this->state.threads_running.load()) return;

        auto start = this->appendMetricStartTime(ProcType::MAP_CB);
        {
            this->mapping_callback_internal(buff);
        }
        auto end = this->appendMetricStopTime(ProcType::MAP_CB);
        this->metrics.mapping_thread.addSample(start, end);

        this->handleStatusUpdate();
    }
    while(this->state.threads_running.load());
}



#if LFD_ENABLED
void PerceptionNode::fiducial_worker()
{
    do
    {
        auto& buff = this->mt.fiducial_resources.waitNewestResource();
        if(!this->state.threads_running.load()) return;

        auto start = this->appendMetricStartTime(ProcType::FID_CB);
        {
            this->fiducial_callback_internal(buff);
        }
        auto end = this->appendMetricStopTime(ProcType::FID_CB);
        this->metrics.fiducial_thread.addSample(start, end);

        this->handleStatusUpdate();
    }
    while(this->state.threads_running.load());
}
#endif



void PerceptionNode::traversibility_worker()
{
    do
    {
        auto& buff = this->mt.traversibility_resources.waitNewestResource();
        if(!this->state.threads_running.load()) return;

        auto start = this->appendMetricStartTime(ProcType::TRAV_CB);
        {
            this->traversibility_callback_internal(buff);
        }
        auto end = this->appendMetricStopTime(ProcType::TRAV_CB);
        this->metrics.trav_thread.addSample(start, end);

        this->handleStatusUpdate();
    }
    while(this->state.threads_running.load());
}





static inline bool do_deskew(
    pcl::PointCloud<PerceptionNode::OdomPointType>& xyz_cloud,
    const sensor_msgs::msg::PointCloud2& scan,
    const pcl::Indices& skip_indices,
    const ImuIntegrator& imu_sampler )
{
    pcl::PointCloud<csm::perception::PointT_32HL> ts_cloud;
    pcl::fromROSMsg(scan, ts_cloud);

    if(xyz_cloud.size() == ts_cloud.size())
    {
        uint64_t min_ts = ts_cloud[0].t, max_ts = ts_cloud[0].t;
        for(size_t i = 1; i < ts_cloud.size(); i++)
        {
            if(ts_cloud[i].t < min_ts) min_ts = ts_cloud[i].t;
            if(ts_cloud[i].t > max_ts) max_ts = ts_cloud[i].t;
        }

        const double ts_diff = static_cast<double>(max_ts - min_ts);
        const double beg_range = util::toFloatSeconds(scan.header.stamp);
        const double end_range = beg_range + ts_diff * 1e-6;

        util::tsq::TSQ<Eigen::Quaterniond> offsets;
        if(imu_sampler.getNormalizedOffsets(offsets, beg_range, end_range))
        {
            std::cout <<
                "[DESKEW]: Obtained " << offsets.size() <<
                " samples (" << offsets.back().second.angularDistance(offsets.front().second) << " rad).";
            // for(size_t i = 0; i < offsets.size(); i++)
            // {
            //     Eigen::Quaterniond& q = offsets[i].second;
            //     std::cout <<
            //         "\n\t" << offsets[i].first <<
            //         " : { " << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << " } (" <<
            //         q.norm() << ')';
            // }
            std::cout << std::endl;

            size_t skip_i = 0;
            for(size_t i = 0; i < xyz_cloud.size(); i++)
            {
                if(skip_indices[skip_i] == i)
                {
                    skip_i++;
                    continue;
                }

                const double t = static_cast<double>(ts_cloud[i].t - min_ts) / ts_diff;
                Eigen::Quaterniond q;
                const size_t idx = util::tsq::binarySearchIdx(offsets, t);
                if(offsets[idx].first == t) q = offsets[idx].second;
                else
                {
                    const auto& a = offsets[idx];
                    const auto& b = offsets[idx - 1];

                    q = a.second.slerp( (t - a.first) / (b.first - a.first), b.second );
                }

                Eigen::Matrix4f rot = Eigen::Matrix4f::Identity();
                rot.block<3, 3>(0, 0) = q.template cast<float>().toRotationMatrix();

                xyz_cloud[i].getVector4fMap() = rot * xyz_cloud[i].getVector4fMap();
            }

            return true;
        }
    }

    return false;

}

void PerceptionNode::scan_callback_internal(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan)
{
// get lidar --> base link transform
    util::geom::PoseTf3f lidar_to_base_tf, base_to_odom_tf;
    const auto scan_stamp = scan->header.stamp;
    try
    {
        this->tf_buffer.lookupTransform(
            this->base_frame,
            scan->header.frame_id,
            util::toTf2TimePoint(scan_stamp)
        ).transform
            >> lidar_to_base_tf.pose
            >> lidar_to_base_tf.tf;
    }
    catch(const std::exception& e)
    {
        RCLCPP_INFO( this->get_logger(),
            "[SCAN CALLBACK]: Failed to get transform from '%s' to '%s'\n\twhat(): %s",
            this->base_frame.c_str(),
            scan->header.frame_id.c_str(),
            e.what() );
        return;
    }

    thread_local pcl::PointCloud<OdomPointType> lo_cloud, tmp_cloud;
    thread_local pcl::Indices nan_indices, bbox_indices, remove_indices;
    lo_cloud.clear();
    // indices get cleared internally when using util functions >>

    const uint32_t iteration_token = this->transform_sync.beginOdometryIteration();
#if TAG_DETECTION_ENABLED
    (void)iteration_token;
#endif

// convert and transform to base link while extracting NaN indices
    pcl::fromROSMsg(*scan, tmp_cloud);
    // lo_cloud = tmp_cloud;
    util::transformAndFilterNaN(
        tmp_cloud,
        tmp_cloud,
        nan_indices,
        lidar_to_base_tf.tf.matrix() );

// apply crop box and accumulate removal indices
    if(this->param.use_crop_filter)
    {
        util::cropbox_filter(
            tmp_cloud,
            bbox_indices,
            this->param.crop_min,
            this->param.crop_max );

        util::pc_combine_sorted(
            nan_indices,
            bbox_indices,
            remove_indices );
    }
    else
    {
        remove_indices = nan_indices;
    }

    // do_deskew(lo_cloud, *scan, remove_indices, this->imu_samples);
    lo_cloud = tmp_cloud;

// apply removal
    util::pc_remove_selection(
        lo_cloud,
        remove_indices );

    // lo_cloud.is_dense = true;
    // pcl::transformPointCloud(lo_cloud, lo_cloud, lidar_to_base_tf.tf.matrix());

#if LFD_ENABLED
// send data to fiducial thread to begin asynchronous localization
    FiducialResources& f = this->mt.fiducial_resources.lockInput();
    f.lidar_to_base = lidar_to_base_tf;
    f.raw_scan = scan;
    if(!f.nan_indices || f.nan_indices.use_count() > 1)
        f.nan_indices = std::make_shared<pcl::Indices>();
    if(!f.remove_indices || f.remove_indices.use_count() > 1)
        f.remove_indices = std::make_shared<pcl::Indices>();
    const_cast<pcl::Indices&>(*f.nan_indices).swap(nan_indices);
    const_cast<pcl::Indices&>(*f.remove_indices).swap(remove_indices);
    const auto& nan_indices_ptr = f.nan_indices;
    const auto& remove_indices_ptr = f.remove_indices;
    f.iteration_count = iteration_token;
    this->mt.fiducial_resources.unlockInputAndNotify(f);
#endif

    // set sensor origin
    lo_cloud.sensor_origin_ << lidar_to_base_tf.pose.vec, 1.f;
    const double new_odom_stamp = util::toFloatSeconds(scan_stamp);

    Eigen::Matrix4f imu_rot = Eigen::Matrix4f::Identity();
    imu_rot.block<3, 3>(0, 0) =
        this->imu_samples
            .getDelta(this->lidar_odom.state.prev_frame_stamp, new_odom_stamp)
            .template cast<float>()
            .toRotationMatrix();

// iterate odometry
    auto lo_status = this->lidar_odom.processScan(
        lo_cloud,
        new_odom_stamp,
        base_to_odom_tf,
        imu_rot );

// on failure >>>
    if(!lo_status)
    {
        this->transform_sync.endOdometryIterationFailure();
    }
// on success >>>
    else
    {
    // Send data to mapping thread
        MappingResources& m = this->mt.mapping_resources.lockInput();
        m.lidar_to_base = lidar_to_base_tf;
        m.base_to_odom = base_to_odom_tf;
        m.raw_scan = scan;
        m.lo_buff.swap(lo_cloud);
    #if LFD_ENABLED     // LFD doesnt share removal indices
        m.nan_indices = nan_indices_ptr;
        m.remove_indices = remove_indices_ptr;
    #else
        if(!m.nan_indices) m.nan_indices = std::make_shared<pcl::Indices>();
        if(!m.remove_indices) m.remove_indices = std::make_shared<pcl::Indices>();
        const_cast<pcl::Indices&>(*m.nan_indices).swap(nan_indices);
        const_cast<pcl::Indices&>(*m.remove_indices).swap(remove_indices);
    #endif
        this->mt.mapping_resources.unlockInputAndNotify(m);

        util::geom::PoseTf3d prev_odom_tf, new_odom_tf;
        const double prev_odom_stamp = this->transform_sync.getOdomTf(prev_odom_tf);

    // Update odom tf
        this->transform_sync.endOdometryIterationSuccess(base_to_odom_tf, new_odom_stamp);

    // Publish velocity
        const double t_diff = this->transform_sync.getOdomTf(new_odom_tf) - prev_odom_stamp;
        Eigen::Vector3d
            l_vel = (new_odom_tf.pose.vec - prev_odom_tf.pose.vec) / t_diff,
            r_vel = (prev_odom_tf.pose.quat.inverse() * new_odom_tf.pose.quat)
                        .toRotationMatrix().eulerAngles(0, 1, 2) / t_diff;

        geometry_msgs::msg::TwistStamped odom_vel;
        odom_vel.twist.linear << l_vel;
        odom_vel.twist.angular << r_vel;
        odom_vel.header.frame_id = this->odom_frame;
        odom_vel.header.stamp = scan_stamp;
        this->velocity_pub->publish(odom_vel);

    // Publish LO debug
        if(this->param.publish_odom_debug)
        {
            this->lidar_odom.publishDebugScans(lo_status, this->odom_frame);
        }

    // Publish deskewed scan
        // sensor_msgs::msg::PointCloud2 deskewed_scan;
        // pcl::toROSMsg(lo_cloud, deskewed_scan);
        // deskewed_scan.header = scan->header;
        // this->scan_pub.publish("deskewed_scan", deskewed_scan);

    // Publish filtering debug
        const auto& trjf = this->transform_sync.trajectoryFilter();

        cardinal_perception::msg::TrajectoryFilterDebug dbg;
        dbg.is_stable = trjf.lastFilterStatus();
        dbg.filter_mask = trjf.lastFilterMask();
        dbg.odom_queue_size = trjf.odomQueueSize();
        dbg.meas_queue_size = trjf.measurementQueueSize();
        dbg.trajectory_length = trjf.trajectoryQueueSize();
        dbg.filter_dt = trjf.lastFilterWindow();
        dbg.linear_error = trjf.lastLinearDelta();
        dbg.angular_error = trjf.lastAngularDelta();
        dbg.linear_deviation = trjf.lastLinearDeviation();
        dbg.angular_deviation = trjf.lastAngularDeviation();
        dbg.avg_linear_error = trjf.lastAvgLinearError();
        dbg.avg_angular_error = trjf.lastAvgAngularError();
        this->traj_filter_debug_pub->publish(dbg);
    }
}





#if LFD_ENABLED
void PerceptionNode::fiducial_callback_internal(FiducialResources& buff)
{
    this->transform_sync.beginMeasurementIteration(buff.iteration_count);

    thread_local pcl::PointCloud<FiducialPointType> reflector_points;
    pcl::fromROSMsg(*buff.raw_scan, reflector_points);

    util::pc_remove_selection(reflector_points, *buff.remove_indices);
    buff.nan_indices.reset();       // signals to odom thread that these buffers can be reused
    buff.remove_indices.reset();

    util::geom::PoseTf3f fiducial_pose;
    auto result = this->fiducial_detector.calculatePose(reflector_points, fiducial_pose.pose);

    if(result)
    {
        util::geom::Pose3d fiducial_to_base, base_to_fiducial;
        fiducial_to_base << (buff.lidar_to_base.tf * (fiducial_pose.tf << fiducial_pose.pose));
        util::geom::inverse(base_to_fiducial, fiducial_to_base);

        this->transform_sync.endMeasurementIterationSuccess(
            base_to_fiducial,
            util::toFloatSeconds(buff.raw_scan->header.stamp) );

        geometry_msgs::msg::PoseStamped p;
        p.pose << fiducial_to_base;
        p.header.stamp = buff.raw_scan->header.stamp;
        p.header.frame_id = this->base_frame;

        this->pose_pub.publish("fiducial_tag_pose", p);
    }
    else
    {
        this->transform_sync.endMeasurementIterationFailure();
    }

#if LFD_PUBLISH_DEBUG
    try
    {
        geometry_msgs::msg::PoseStamped _p;
        sensor_msgs::msg::PointCloud2 _pc;

        const auto& input_cloud = this->fiducial_detector.getInputPoints();

        pcl::toROSMsg(input_cloud, _pc);
        _pc.header = buff.raw_scan->header;

        this->scan_pub.publish("fiducial_reflective_points", _pc);
        
        if(result.has_point_num)
        {
            const auto& seg_clouds = this->fiducial_detector.getSegClouds();
            const auto& seg_planes = this->fiducial_detector.getSegPlanes();
            const auto& seg_plane_centers = this->fiducial_detector.getPlaneCenters();
            const auto& remaining_points = this->fiducial_detector.getRemainingPoints();

            for(uint32_t i = 0; i < result.iterations; i++)
            {
                _p.header = buff.raw_scan->header;
                _p.pose.position << seg_plane_centers[i];
                _p.pose.orientation <<
                    Eigen::Quaternionf::FromTwoVectors(
                        Eigen::Vector3f{1.f, 0.f, 0.f},
                        seg_planes[i].head<3>() );

                std::string topic = (std::ostringstream{} << "fiducial_plane_" << i << "/pose").str();
                this->pose_pub.publish(topic, _p);

                pcl::toROSMsg(seg_clouds[i], _pc);
                _pc.header = buff.raw_scan->header;

                topic = ((std::ostringstream{} << "fiducial_plane_" << i << "/points").str());
                this->scan_pub.publish(topic, _pc);
            }

            if(result.iterations == 3 && !remaining_points.empty())
            {
                pcl::toROSMsg(remaining_points, _pc);
                _pc.header = buff.raw_scan->header;

                this->scan_pub.publish("fiducial_unmodeled_points", _pc);
            }
        }
    }
    catch(const std::exception& e)
    {
        RCLCPP_INFO(this->get_logger(), "[FIDUCIAL DETECTION]: Failed to publish debug data -- what():\n\t%s", e.what());
    }

#endif
}
#endif





void PerceptionNode::mapping_callback_internal(MappingResources& buff)
{
    // RCLCPP_INFO(this->get_logger(), "MAPPING CALLBACK INTERNAL");

    util::geom::PoseTf3f lidar_to_odom_tf;
    lidar_to_odom_tf.pose << (lidar_to_odom_tf.tf = buff.base_to_odom.tf * buff.lidar_to_base.tf);

    pcl::PointCloud<MappingPointType>* filtered_scan_t = nullptr;
    if constexpr(std::is_same<OdomPointType, MappingPointType>::value)
    {
        pcl::transformPointCloud(buff.lo_buff, buff.lo_buff, buff.base_to_odom.tf, true);
        filtered_scan_t = &buff.lo_buff;
    }
    else
    {
        thread_local pcl::PointCloud<MappingPointType> map_input_cloud;
        pcl::fromROSMsg(*buff.raw_scan, map_input_cloud);

        util::pc_remove_selection(map_input_cloud, *buff.remove_indices);
        pcl::transformPointCloud(map_input_cloud, map_input_cloud, lidar_to_odom_tf.tf, true);
        filtered_scan_t = &map_input_cloud;
    }

    auto results = this->environment_map.updateMap(lidar_to_odom_tf.pose.vec, *filtered_scan_t);

    #if 0
    {
        pcl::Indices export_points;
        const Eigen::Vector3f search_range{
            static_cast<float>(this->param.map_export_horizontal_range),
            static_cast<float>(this->param.map_export_horizontal_range),
            static_cast<float>(this->param.map_export_vertical_range) };

        this->environment_map.getMap().boxSearch(
            buff.base_to_odom.pose.vec - search_range,
            buff.base_to_odom.pose.vec + search_range,
            export_points );

        auto& x = this->mt.traversibility_resources.lockInput();
        x.lidar_to_base = buff.lidar_to_base;
        x.base_to_odom = buff.base_to_odom;
        if(!x.points || x.points.use_count() > 1)
            x.points = std::make_shared<pcl::PointCloud<MappingPointType>>();
        util::pc_copy_selection(
            *this->environment_map.getPoints(),
            export_points,
            *x.points );
        x.stamp = util::toFloatSeconds(buff.raw_scan->header.stamp);
        this->mt.traversibility_resources.unlockInputAndNotify(x);
    }
    #endif

    // if(!this->param.rebias_scan_pub_prereq || this->state.has_rebiased
    //     IF_TAG_DETECTION_ENABLED(|| !this->param.use_tag_detections) )
    // {
        try
        {
            sensor_msgs::msg::PointCloud2 output;
        #if PUBLISH_FULL_MAP
            pcl::toROSMsg(*this->environment_map.getPoints(), output);
            output.header.stamp = buff.raw_scan->header.stamp;
            output.header.frame_id = this->odom_frame;
            this->map_cloud_pub->publish(output);
        #endif

            pcl::toROSMsg(*filtered_scan_t, output);
            output.header.stamp = buff.raw_scan->header.stamp;
            output.header.frame_id = this->odom_frame;
            this->filtered_scan_pub->publish(output);
        }
        catch(const std::exception& e)
        {
            RCLCPP_INFO(this->get_logger(), "[MAPPING]: Failed to publish mapping debug scans -- what():\n\t%s", e.what());
        }
    // }

    this->metrics_pub.publish("mapping/search_pointset", static_cast<double>(results.points_searched));
    this->metrics_pub.publish("mapping/points_deleted", static_cast<double>(results.points_deleted));

    // RCLCPP_INFO(this->get_logger(), "[MAPPING]: Processing took %f milliseconds.", dt_ * 1e3);
}





void PerceptionNode::traversibility_callback_internal(TraversibilityResources& buff)
{
    try
    {
        pcl::Indices ground_indices;
        util::progressive_morph_filter(
            *buff.points,
            ground_indices,
            2.f,    // window base (units?)
            0.48f,  // max window size in meters
            0.05f,  // cell size in meters
            0.05f,  // initial distance in meters
            0.12f,  // max distance in meters
            2.f,    // slope
            false );

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*buff.points, output);
        output.header.stamp = util::toTimeStamp(buff.stamp);
        output.header.frame_id = this->odom_frame;
        this->scan_pub.publish("traversability_points", output);

        decltype(buff.points)::element_type ground_seg, obstacle_seg;
        util::pc_copy_selection(*buff.points, ground_indices, ground_seg);
        util::pc_copy_inverse_selection(*buff.points, ground_indices, obstacle_seg);

        pcl::toROSMsg(ground_seg, output);
        output.header.stamp = util::toTimeStamp(buff.stamp);
        output.header.frame_id = this->odom_frame;
        this->scan_pub.publish("traversability_ground_points", output);
        pcl::toROSMsg(obstacle_seg, output);
        output.header.stamp = util::toTimeStamp(buff.stamp);
        output.header.frame_id = this->odom_frame;
        this->scan_pub.publish("traversability_obstacle_points", output);
    }
    catch(const std::exception& e)
    {
        RCLCPP_INFO(this->get_logger(), "[TRAVERSIBILITY]: Failed to publish debug cloud -- what():\n\t%s", e.what());
    }

    /** Find rocks, generate traversibility... and send it somewhere!? */
}





template<bool Start>
inline PerceptionNode::ClockType::time_point appendMetricTimeCommon(PerceptionNode* node, PerceptionNode::ProcType type)
{
    if(type == PerceptionNode::ProcType::NUM_ITEMS) return PerceptionNode::ClockType::time_point::min();
    std::lock_guard<std::mutex> _lock{ node->metrics.thread_procs_mtx };
    auto tp = PerceptionNode::ClockType::now();

    std::thread::id _id = std::this_thread::get_id();
    auto ptr = node->metrics.thread_metric_durations.find(_id);
    if(ptr == node->metrics.thread_metric_durations.end())
    {
        auto x = node->metrics.thread_metric_durations.insert( {_id, {}} );
        if(!x.second) return tp;
        else ptr = x.first;
    }

    auto& dur_buff = ptr->second[static_cast<size_t>(type)];
    if(dur_buff.second > PerceptionNode::ClockType::time_point::min())
    {
        if constexpr(!Start)
        {
            dur_buff.first += std::chrono::duration_cast<std::chrono::duration<double>>(tp - dur_buff.second).count();
            dur_buff.second = PerceptionNode::ClockType::time_point::min();
        }
    }
    else
    {
        if constexpr(Start)
        {
            dur_buff.second = tp;
        }
    }

    return tp;
}

PerceptionNode::ClockType::time_point PerceptionNode::appendMetricStartTime(ProcType type)
{
    return appendMetricTimeCommon<true>(this, type);
}

PerceptionNode::ClockType::time_point PerceptionNode::appendMetricStopTime(ProcType type)
{
    return appendMetricTimeCommon<false>(this, type);
}

};
};

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
