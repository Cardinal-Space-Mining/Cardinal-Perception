/*******************************************************************************
*   Copyright (C) 2024 Cardinal Space Mining Club                              *
*                                                                              *
*   Unless required by applicable law or agreed to in writing, software        *
*   distributed under the License is distributed on an "AS IS" BASIS,          *
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
*   See the License for the specific language governing permissions and        *
*   limitations under the License.                                             *
*                                                                              *
*                                ;xxxxxxx:                                     *
*                               ;$$$$$$$$$       ...::..                       *
*                               $$$$$$$$$$x   .:::::::::::..                   *
*                            x$$$$$$$$$$$$$$::::::::::::::::.                  *
*                        :$$$$$&X;      .xX:::::::::::::.::...                 *
*                .$$Xx++$$$$+  :::.     :;:   .::::::.  ....  :                *
*               :$$$$$$$$$  ;:      ;xXXXXXXXx  .::.  .::::. .:.               *
*              :$$$$$$$$: ;      ;xXXXXXXXXXXXXx: ..::::::  .::.               *
*             ;$$$$$$$$ ::   :;XXXXXXXXXXXXXXXXXX+ .::::.  .:::                *
*              X$$$$$X : +XXXXXXXXXXXXXXXXXXXXXXXX; .::  .::::.                *
*               .$$$$ :xXXXXXXXXXXXXXXXXXXXXXXXXXXX.   .:::::.                 *
*                X$$X XXXXXXXXXXXXXXXXXXXXXXXXXXXXx:  .::::.                   *
*                $$$:.XXXXXXXXXXXXXXXXXXXXXXXXXXX  ;; ..:.                     *
*                $$& :XXXXXXXXXXXXXXXXXXXXXXXX;  +XX; X$$;                     *
*                $$$::XXXXXXXXXXXXXXXXXXXXXX: :XXXXX; X$$;                     *
*                X$$X XXXXXXXXXXXXXXXXXXX; .+XXXXXXX; $$$                      *
*                $$$$ ;XXXXXXXXXXXXXXX+  +XXXXXXXXx+ X$$$+                     *
*              x$$$$$X ;XXXXXXXXXXX+ :xXXXXXXXX+   .;$$$$$$                    *
*             +$$$$$$$$ ;XXXXXXx;;+XXXXXXXXX+    : +$$$$$$$$                   *
*              +$$$$$$$$: xXXXXXXXXXXXXXX+      ; X$$$$$$$$                    *
*               :$$$$$$$$$. +XXXXXXXXX:      ;: x$$$$$$$$$                     *
*               ;x$$$$XX$$$$+ .;+X+      :;: :$$$$$xX$$$X                      *
*              ;;;;;;;;;;X$$$$$$$+      :X$$$$$$&.                             *
*              ;;;;;;;:;;;;;x$$$$$$$$$$$$$$$$x.                                *
*              :;;;;;;;;;;;;.  :$$$$$$$$$$X                                    *
*               .;;;;;;;;:;;    +$$$$$$$$$                                     *
*                 .;;;;;;.       X$$$$$$$:                                     *
*                                                                              *
*******************************************************************************/

#include "./perception.hpp"
#include "imu_transform.hpp"

#include <sstream>
#include <fstream>
#include <stdio.h>
#include <iomanip>

#include <boost/algorithm/string.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#ifndef ENABLE_PRINT_STATUS
#define ENABLE_PRINT_STATUS 1
#endif

#define MAPPING_THREAD_WAIT_RETRY_LOCK 0b01
#define MAPPING_THREAD_WAIT_UPDATE_RESOURCES 0b10


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
    // mt_callback_group{ this->create_callback_group(rclcpp::CallbackGroupType::Reentrant) },
    metrics_pub{ this, "/localization/" },
    pose_pub{ this, "/localization/" },
    scan_pub{ this, "/localization/" },
    lidar_odom{ this },
    trajectory_filter{}
{
    this->getParams();

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
    // ops.callback_group = this->mt_callback_group;

    this->detections_sub = this->create_subscription<cardinal_perception::msg::TagsTransform>(
        "tags_detections", rclcpp::SensorDataQoS{},
        [this](const cardinal_perception::msg::TagsTransform::ConstSharedPtr& det){ this->detection_callback(det); }, ops);
    this->scan_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        scan_topic, rclcpp::SensorDataQoS{},
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan){ this->scan_callback(scan); }, ops);
    this->imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, rclcpp::SensorDataQoS{},
        [this](const sensor_msgs::msg::Imu::SharedPtr imu){ this->imu_callback(imu); }, ops);

    this->filtered_scan_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_scan", rclcpp::SensorDataQoS{});
    this->map_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_cloud", rclcpp::SensorDataQoS{});
    this->velocity_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("odom_velocity", rclcpp::SensorDataQoS{});

    this->proc_metrics_pub = this->create_publisher<cardinal_perception::msg::ProcessMetrics>("/localization/process_metrics", rclcpp::SensorDataQoS{});
    this->imu_metrics_pub = this->create_publisher<cardinal_perception::msg::ThreadMetrics>("/localization/imu_cb_metrics", rclcpp::SensorDataQoS{});
    this->det_metrics_pub = this->create_publisher<cardinal_perception::msg::ThreadMetrics>("/localization/det_cb_metrics", rclcpp::SensorDataQoS{});
    this->scan_metrics_pub = this->create_publisher<cardinal_perception::msg::ThreadMetrics>("/localization/scan_cb_metrics", rclcpp::SensorDataQoS{});
    this->mapping_metrics_pub = this->create_publisher<cardinal_perception::msg::ThreadMetrics>("/localization/mapping_cb_metrics", rclcpp::SensorDataQoS{});
    this->traj_filter_debug_pub = this->create_publisher<cardinal_perception::msg::TrajectoryFilterDebug>("/localization/trajectory_filter", rclcpp::SensorDataQoS{});
#if USE_GTSAM_PGO > 0
    this->path_pub = this->create_publisher<nav_msgs::msg::Path>("path", rclcpp::SensorDataQoS{});
#endif

    this->mt.localization_threads.reserve(this->param.max_localization_threads);    // TODO: actually fix this?
    this->mt.mapping_threads.reserve(this->param.max_mapping_threads);
    this->mapping.map_octree.setResolution(this->param.mapping_voxel_size);
}

PerceptionNode::~PerceptionNode()
{
    // RCLCPP_INFO(this->get_logger(), "PERCEPTION NODE DESTRUCTOR BEGIN");
    this->shutdown();
    // RCLCPP_INFO(this->get_logger(), "PERCEPTION NODE DESTRUCTOR END");
}

void PerceptionNode::shutdown()
{
    this->state.threads_running = false;
    this->mt.mapping_update_notifier.notify_all();
    for(auto& x : this->mt.localization_threads)
    {
        if(x.thread.joinable())
        {
            x.notifier.notify_all();
            x.thread.join();
        }
    }
    for(auto& x : this->mt.mapping_threads)
    {
        if(x.thread.joinable())
        {
            x.notifier.notify_all();
            x.thread.join();
        }
    }
}


void PerceptionNode::getParams()
{
    util::declare_param(this, "map_frame_id", this->map_frame, "map");
    util::declare_param(this, "odom_frame_id", this->odom_frame, "odom");
    util::declare_param(this, "base_frame_id", this->base_frame, "base_link");

    util::declare_param(this, "use_tag_detections", this->param.use_tag_detections, -1);
    util::declare_param(this, "require_rebias_before_tf_pub", this->param.rebias_tf_pub_prereq, false);
    util::declare_param(this, "require_rebias_before_scan_pub", this->param.rebias_scan_pub_prereq, false);
    util::declare_param(this, "metrics_pub_freq", this->param.metrics_pub_freq, 10.);

    double sample_window_s, filter_window_s, avg_linear_err_thresh, avg_angular_err_thresh, max_linear_deviation_thresh, max_angular_deviation_thresh;
    util::declare_param(this, "trajectory_filter.sampling_window_s", sample_window_s, 0.5);
    util::declare_param(this, "trajectory_filter.min_filter_window_s", filter_window_s, 0.3);
    util::declare_param(this, "trajectory_filter.thresh.avg_linear_error", avg_linear_err_thresh, 0.2);
    util::declare_param(this, "trajectory_filter.thresh.avg_angular_error", avg_angular_err_thresh, 0.1);
    util::declare_param(this, "trajectory_filter.thresh.max_linear_deviation", max_linear_deviation_thresh, 4e-2);
    util::declare_param(this, "trajectory_filter.thresh.max_angular_deviation", max_angular_deviation_thresh, 4e-2);
    this->trajectory_filter.applyParams(
        sample_window_s,
        filter_window_s,
        avg_linear_err_thresh,
        avg_angular_err_thresh,
        max_linear_deviation_thresh,
        max_angular_deviation_thresh );

    util::declare_param(this, "max_localization_threads", this->param.max_localization_threads, 1);
    util::declare_param(this, "max_mapping_threads", this->param.max_mapping_threads, 1);

    util::declare_param(this, "mapping.valid_range", this->param.mapping_valid_range, 0.);
    util::declare_param(this, "mapping.frustum_search_radius", this->param.mapping_frustum_search_radius, 0.01);
    util::declare_param(this, "mapping.delete_range_thresh", this->param.mapping_delete_range_thresh, 0.1);
    util::declare_param(this, "mapping.add_max_range", this->param.mapping_add_max_range, 5.);
    util::declare_param(this, "mapping.voxel_size", this->param.mapping_voxel_size, 0.1);
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
        auto _tp = this->appendMetricStartTime(ProcType::HANDLE_METRICS);
        const double _dt = util::toFloatSeconds(_tp - this->state.last_print_time);
        if(_dt > (1. / this->param.metrics_pub_freq))
        {
            this->state.last_print_time = _tp;

            double resident_set_mb = 0.;
            size_t num_threads = 0;
            this->metrics.process_utilization.update();
            util::proc::getProcessStats(resident_set_mb, num_threads);

        #if ENABLE_PRINT_STATUS
            std::ostringstream msg;

            msg << std::setprecision(2) << std::fixed << std::right << std::setfill(' ') << std::endl;
            msg << "+-------------------------------------------------------------------+\n"
                   "| =================== Cardinal Perception v0.4.1 ================== |\n"
                   "+- RESOURCES -------------------------------------------------------+\n"
                   "|                      ::  Current  |  Average  |  Maximum          |\n";
            msg << "|      CPU Utilization ::  " << std::setw(6) << (this->metrics.process_utilization.last_cpu_percent)
                                                << " %  |  " << std::setw(6) << this->metrics.process_utilization.avg_cpu_percent
                                                            << " %  |  " << std::setw(5) << this->metrics.process_utilization.max_cpu_percent
                                                                         << " %         |\n";
            msg << "|       RAM Allocation :: " << std::setw(6) << resident_set_mb
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
            // this->metrics.det_thread.mtx.lock();
            msg << "|   DET CB (" << std::setw(5) << 1. / this->metrics.det_thread.avg_call_delta
                                 << " Hz) ::  " << std::setw(5) << this->metrics.det_thread.last_comp_time * 1e6
                                               << " us  | " << std::setw(5) << this->metrics.det_thread.avg_comp_time * 1e6
                                                           << " us  | " << std::setw(5) << this->metrics.det_thread.max_comp_time * 1e3
                                                                       << " ms | " << std::setw(6) << this->metrics.det_thread.samples
                                                                                   << " |\n";
            // this->metrics.det_thread.mtx.unlock();
            // this->metrics.imu_thread.mtx.lock();
            msg << "|   IMU CB (" << std::setw(5) << 1. / this->metrics.imu_thread.avg_call_delta
                                 << " Hz) ::  " << std::setw(5) << this->metrics.imu_thread.last_comp_time * 1e6
                                               << " us  | " << std::setw(5) << this->metrics.imu_thread.avg_comp_time * 1e6
                                                           << " us  | " << std::setw(5) << this->metrics.imu_thread.max_comp_time * 1e3
                                                                       << " ms | " << std::setw(6) << this->metrics.imu_thread.samples
                                                                                   << " |\n";
            // this->metrics.imu_thread.mtx.unlock();
            // this->metrics.scan_thread.mtx.lock();
            msg << "|  Scan CB (" << std::setw(5) << 1. / this->metrics.scan_thread.avg_call_delta
                                 << " Hz) ::  " << std::setw(5) << this->metrics.scan_thread.last_comp_time * 1e3
                                               << " ms  | " << std::setw(5) << this->metrics.scan_thread.avg_comp_time * 1e3
                                                           << " ms  | " << std::setw(5) << this->metrics.scan_thread.max_comp_time * 1e3
                                                                       << " ms | " << std::setw(6) << this->metrics.scan_thread.samples
                                                                                   << " |\n";
            // this->metrics.scan_thread.mtx.unlock();
            // this->metrics.mapping_thread.mtx.lock();
            msg << "|   Map CB (" << std::setw(5) << 1. / this->metrics.mapping_thread.avg_call_delta
                                 << " Hz) ::  " << std::setw(5) << this->metrics.mapping_thread.last_comp_time * 1e3
                                               << " ms  | " << std::setw(5) << this->metrics.mapping_thread.avg_comp_time * 1e3
                                                           << " ms  | " << std::setw(5) << this->metrics.mapping_thread.max_comp_time * 1e3
                                                                       << " ms | " << std::setw(6) << this->metrics.mapping_thread.samples
                                                                                   << " |\n";
            // this->metrics.mapping_thread.mtx.unlock();
            msg << "|                                                                   |\n"
                   "+- THREAD UTILIZATION ----------------------------------------------+\n"
                   "|                                                                   |\n";

            this->metrics.thread_procs_mtx.lock();
            size_t idx = 0;
            for(auto& p : this->metrics.thread_metric_durations)
            {
                static constexpr char const* CHAR_VARS = "TSIMXm"; // Tags, Scan, Imu, Mapping, metrics(X), misc
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

            msg << "+-------------------------------------------------------------------+" << std::endl;
                // << "\n| MTSync waited iterations           : " << this->metrics.mapping_waited_loops.load()
                // << "\n| MTSync wait retry exits            : " << this->metrics.mapping_wait_retry_exits.load()
                // << "\n| MTSync wait refresh exits          : " << this->metrics.mapping_wait_refresh_exits.load()
                // << "\n| MTSync resource update attempts    : " << this->metrics.mapping_update_attempts.load()
                // << "\n| MTSync resource update completions : " << this->metrics.mapping_update_completes.load() << std::endl;

            // print
            // printf("\033[2J\033[1;1H");
            std::cout << "\033[2J\033[1;1H" << std::endl;
            RCLCPP_INFO(this->get_logger(), "%s", msg.str().c_str());
        #endif
            {
                cardinal_perception::msg::ProcessMetrics pm;
                pm.cpu_percent = static_cast<float>(this->metrics.process_utilization.last_cpu_percent);
                pm.avg_cpu_percent = static_cast<float>(this->metrics.process_utilization.avg_cpu_percent);
                pm.mem_usage_mb = static_cast<float>(resident_set_mb);
                pm.num_threads = static_cast<uint32_t>(num_threads);
                this->proc_metrics_pub->publish(pm);

                cardinal_perception::msg::ThreadMetrics tm;
                tm.delta_t = static_cast<float>(this->metrics.imu_thread.last_comp_time);
                tm.avg_delta_t = static_cast<float>(this->metrics.imu_thread.avg_comp_time);
                tm.avg_freq = static_cast<float>(1. / this->metrics.imu_thread.avg_call_delta);
                tm.iterations = this->metrics.imu_thread.samples;
                this->imu_metrics_pub->publish(tm);

                tm.delta_t = static_cast<float>(this->metrics.det_thread.last_comp_time);
                tm.avg_delta_t = static_cast<float>(this->metrics.det_thread.avg_comp_time);
                tm.avg_freq = static_cast<float>(1. / this->metrics.det_thread.avg_call_delta);
                tm.iterations = this->metrics.det_thread.samples;
                this->det_metrics_pub->publish(tm);

                tm.delta_t = static_cast<float>(this->metrics.scan_thread.last_comp_time);
                tm.avg_delta_t = static_cast<float>(this->metrics.scan_thread.avg_comp_time);
                tm.avg_freq = static_cast<float>(1. / this->metrics.scan_thread.avg_call_delta);
                tm.iterations = this->metrics.scan_thread.samples;
                this->scan_metrics_pub->publish(tm);

                tm.delta_t = static_cast<float>(this->metrics.mapping_thread.last_comp_time);
                tm.avg_delta_t = static_cast<float>(this->metrics.mapping_thread.avg_comp_time);
                tm.avg_freq = static_cast<float>(1. / this->metrics.mapping_thread.avg_call_delta);
                tm.iterations = this->metrics.mapping_thread.samples;
                this->mapping_metrics_pub->publish(tm);
            }
        }
        this->appendMetricStopTime(ProcType::HANDLE_METRICS);
        this->state.print_mtx.unlock();
    }
}


void PerceptionNode::detection_callback(const cardinal_perception::msg::TagsTransform::ConstSharedPtr& detection_group)
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
        this->trajectory_filter.addMeasurement(td, td->time_point);

        // RCLCPP_INFO(this->get_logger(), "[DETECTION CB]: Recv - Base delta: %f", util::toFloatSeconds(this->get_clock()->now()) - td->time_point);
    }

    auto _end = this->appendMetricStopTime(ProcType::DET_CB);
    this->metrics.det_thread.addSample(_start, _end);

    this->handleStatusUpdate();
}

void PerceptionNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu)
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

        this->lidar_odom.processImu(*imu);
    }
    catch(const std::exception& e)
    {
        RCLCPP_INFO(this->get_logger(), "IMU CALLBACK: [dlo] failed to process imu measurment.\n\twhat(): %s", e.what());
    }

    auto _end = this->appendMetricStopTime(ProcType::IMU_CB);
    this->metrics.imu_thread.addSample(_start, _end);

    this->handleStatusUpdate();

    // RCLCPP_INFO(this->get_logger(), "IMU_CALLBACK EXIT");
}

void PerceptionNode::scan_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan)
{
    // RCLCPP_INFO(this->get_logger(), "SCAN CALLBACK");
    std::unique_lock<std::mutex> lock{ this->mt.localization_thread_queue_mtx };
    if(this->mt.localization_thread_queue.size() <= 0)
    {
        // RCLCPP_INFO(this->get_logger(), 
        //     "[SCAN CB]: No queued threads -- queued: %ld, total: %ld",
        //     this->mt.localization_thread_queue.size(),
        //     this->mt.localization_threads.size() );

        if( this->param.max_localization_threads > 0 &&
            this->mt.localization_threads.size() >= (size_t)this->param.max_localization_threads) return;

        this->mt.localization_threads.emplace_back();
        auto& inst = this->mt.localization_threads.back();
        inst.thread = std::thread{ &PerceptionNode::localization_worker, this, std::ref(inst) };
        this->mt.localization_thread_queue.push_back(&inst);

        // RCLCPP_INFO(this->get_logger(), 
        //     "[SCAN CB]: Queued new thread -- queued: %ld, total: %ld",
        //     this->mt.localization_thread_queue.size(),
        //     this->mt.localization_threads.size() );
    }
    auto* inst = this->mt.localization_thread_queue.front();
    this->mt.localization_thread_queue.pop_front();
    lock.unlock();
    inst->scan = scan;  // TODO: this is technically unsafe if ROS reuses this buffer after callback exits
    inst->link_state = 1;
    inst->notifier.notify_all();
}


void PerceptionNode::localization_worker(ScanCbThread& inst)
{
    do
    {
        std::mutex temp_mtx;
        std::unique_lock<std::mutex> temp_lock{ temp_mtx };
        while(this->state.threads_running.load() && !inst.link_state)
        {
            inst.notifier.wait(temp_lock);
        }
        if(!this->state.threads_running.load()) return;

        // RCLCPP_INFO(this->get_logger(), "SCAN CALLBACK START");
        this->scan_callback_internal(inst.scan);
        // RCLCPP_INFO(this->get_logger(), "SCAN CALLBACK END");
        inst.link_state = 0;

        this->mt.localization_thread_queue_mtx.lock();
        this->mt.localization_thread_queue.push_back(&inst);
        this->mt.localization_thread_queue_mtx.unlock();
        // RCLCPP_INFO(this->get_logger(), 
        //     "[LOCALIZATION WORKER]: Queued current thread -- queued: %ld, total: %ld",
        //     this->mt.localization_thread_queue.size(),
        //     this->mt.localization_threads.size() );
    }
    while(this->state.threads_running.load());
}

void PerceptionNode::mapping_worker(MappingCbThread& inst)
{
    do
    {
        std::mutex temp_mtx;
        std::unique_lock<std::mutex> temp_lock{ temp_mtx };
        while(this->state.threads_running.load() && !inst.link_state)
        {
            inst.notifier.wait(temp_lock);
        }
        if(!this->state.threads_running.load()) return;

        // RCLCPP_INFO(this->get_logger(), "MAPPING CALLBACK START");
        this->mapping_callback_internal(inst);
        // RCLCPP_INFO(this->get_logger(), "MAPPING CALLBACK END");
        inst.link_state = 0;

        this->mt.mapping_thread_queue_mtx.lock();
        this->mt.mapping_thread_queue.push_back(&inst);
        this->mt.mapping_thread_queue_mtx.unlock();
        this->mt.mapping_reverse_notifier.notify_all();
    }
    while(this->state.threads_running.load());
}


void PerceptionNode::scan_callback_internal(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan)
{
    // RCLCPP_INFO(this->get_logger(), "SCAN CALLBACK INTERNAL");
    auto _start = this->appendMetricStartTime(ProcType::SCAN_CB);

    pcl::PointCloud<OdomPointType>::Ptr filtered_scan = std::make_shared<pcl::PointCloud<OdomPointType>>();
    util::geom::PoseTf3d new_odom_tf;
    Eigen::Vector3d lidar_off;
    int64_t dlo_status = 0;
    const auto scan_stamp = scan->header.stamp;

    try
    {
        sensor_msgs::msg::PointCloud2::SharedPtr
            scan_ = std::make_shared<sensor_msgs::msg::PointCloud2>();

        auto tf = this->tf_buffer.lookupTransform(
            this->base_frame,
            scan->header.frame_id,
            util::toTf2TimePoint(scan->header.stamp));

        tf2::doTransform(*scan, *scan_, tf);

        dlo_status = this->lidar_odom.processScan(scan_, new_odom_tf, filtered_scan);
        lidar_off << tf.transform.translation;
    }
    catch(const std::exception& e)
    {
        RCLCPP_INFO(this->get_logger(), "SCAN CALLBACK: [dlo] failed to process scan.\n\twhat(): %s", e.what());
        dlo_status = 0;
    }

    if(dlo_status)
    {
        const double new_odom_stamp = util::toFloatSeconds(scan_stamp);

        do
        {
            std::unique_lock<std::mutex> lock{ this->mt.mapping_thread_queue_mtx };
            if(this->mt.mapping_thread_queue.size() <= 0)
            {
                if( this->param.max_mapping_threads > 0 &&
                    this->mt.mapping_threads.size() >= (size_t)this->param.max_mapping_threads)
                {
                    if(this->mt.mapping_threads_waiting.load() > 0)
                    {
                        // this->metrics.mapping_update_attempts++;
                        this->mt.mapping_notifier_status |= MAPPING_THREAD_WAIT_UPDATE_RESOURCES;
                        this->mt.mapping_update_notifier.notify_one();

                        if( this->mt.mapping_reverse_notifier.wait_for(lock, std::chrono::microseconds(50)) == std::cv_status::timeout ||
                            this->mt.mapping_thread_queue.size() <= 0 ) break;
                        // this->metrics.mapping_update_completes++;
                    }
                    else break;
                }
                else
                {
                    this->mt.mapping_threads.emplace_back();
                    auto& inst = this->mt.mapping_threads.back();
                    inst.thread = std::thread{ &PerceptionNode::mapping_worker, this, std::ref(inst) };
                    this->mt.mapping_thread_queue.push_back(&inst);
                }
            }
            auto* inst = this->mt.mapping_thread_queue.front();
            this->mt.mapping_thread_queue.pop_front();
            lock.unlock();
            if(!inst->filtered_scan) inst->filtered_scan = std::make_shared<pcl::PointCloud<LidarOdometry::PointType>>();    // interesting crash here
            std::swap(inst->filtered_scan, filtered_scan);
            inst->odom_tf = new_odom_tf.tf.template cast<float>();
            inst->lidar_off = lidar_off.template cast<float>();
            inst->stamp = new_odom_stamp;
            inst->link_state = 1;
            inst->notifier.notify_all();
        }
        while(0);

        geometry_msgs::msg::PoseStamped _pose;
        _pose.header.stamp = scan_stamp;

        this->trajectory_filter.addOdom(new_odom_tf.pose, new_odom_stamp);

        const bool stable = this->trajectory_filter.lastFilterStatus();
        const size_t
            odom_q_sz = this->trajectory_filter.odomQueueSize(),
            meas_q_sz = this->trajectory_filter.measurementQueueSize(),
            trajectory_sz = this->trajectory_filter.trajectoryQueueSize();

        cardinal_perception::msg::TrajectoryFilterDebug dbg;
        dbg.is_stable = stable;
        dbg.filter_mask = this->trajectory_filter.lastFilterMask();
        dbg.odom_queue_size = odom_q_sz;
        dbg.meas_queue_size = meas_q_sz;
        dbg.trajectory_length = trajectory_sz;
        dbg.filter_dt = this->trajectory_filter.lastFilterWindow();
        dbg.linear_error = this->trajectory_filter.lastLinearDelta();
        dbg.angular_error = this->trajectory_filter.lastAngularDelta();
        dbg.linear_deviation = this->trajectory_filter.lastLinearDeviation();
        dbg.angular_deviation = this->trajectory_filter.lastAngularDeviation();
        dbg.avg_linear_error = this->trajectory_filter.lastAvgLinearError();
        dbg.avg_angular_error = this->trajectory_filter.lastAvgAngularError();
        this->traj_filter_debug_pub->publish(dbg);

    #if USE_GTSAM_PGO > 0
        size_t run_isam = 0;
    #endif

        this->state.tf_mtx.lock();  // TODO: timeout

        if(!(dlo_status & (1 << 1)))
        {
        #if USE_GTSAM_PGO > 0
            const bool new_keyframe = dlo_status & (1 << 2);

            static gtsam::noiseModel::Diagonal::shared_ptr odom_noise =    // shared for multiple branches >>>
                gtsam::noiseModel::Diagonal::Variances( (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 25e-6, 25e-6, 25e-6).finished() );
        #endif

            if(stable)
            {
                auto keypose = this->trajectory_filter.getFiltered();
                const TagDetection::Ptr& detection = keypose.second.measurement;

                // _pose.header.frame_id = this->odom_frame;
                // _pose.pose << keypose.second.odometry;
                // this->pose_pub.publish("filtered_odom_pose", _pose);

                // _pose.header.frame_id = this->map_frame;
                // _pose.pose << detection->pose;
                // this->pose_pub.publish("filtered_aruco_pose", _pose);

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

            if(new_keyframe)
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
                    this->state.has_rebiased = true;
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

        {
            const double t_diff = new_odom_stamp - this->state.last_odom_stamp;
            Eigen::Vector3d
                l_vel = (new_odom_tf.pose.vec - this->state.odom_tf.pose.vec) / t_diff,
                r_vel = (this->state.odom_tf.pose.quat.inverse() * new_odom_tf.pose.quat)
                    .toRotationMatrix().eulerAngles(0, 1, 2) / t_diff;

            geometry_msgs::msg::TwistStamped odom_vel;
            odom_vel.twist.linear << l_vel;
            odom_vel.twist.angular << r_vel;
            odom_vel.header.frame_id = this->odom_frame;
            odom_vel.header.stamp = scan_stamp;

            this->velocity_pub->publish(odom_vel);
        }

        // TODO: use PGO??? >>
        this->state.odom_tf = new_odom_tf;
        this->state.last_odom_stamp = new_odom_stamp;

        // publish tf
        if(!this->param.rebias_tf_pub_prereq || this->state.has_rebiased || !this->param.use_tag_detections)
        {
            this->sendTf(scan_stamp, false);
        }
        this->state.tf_mtx.unlock();

        this->lidar_odom.publishDebugScans();
    }

    auto _end = this->appendMetricStopTime(ProcType::SCAN_CB);
    this->metrics.scan_thread.addSample(_start, _end);

    this->handleStatusUpdate();

    // RCLCPP_INFO(this->get_logger(), "SCAN_CALLBACK EXIT -- %s", (std::stringstream{} << std::this_thread::get_id()).str().c_str());
}

void PerceptionNode::mapping_callback_internal(const MappingCbThread& inst)
{
    // RCLCPP_INFO(this->get_logger(), "MAPPING CALLBACK INTERNAL");

    auto _start = this->appendMetricStartTime(ProcType::MAP_CB);
    std::unique_lock _lock{ this->mapping.mtx, std::try_to_lock };
    while(!_lock.owns_lock())
    {
        // this->metrics.mapping_waited_loops++;
        std::mutex temp_mtx;
        std::unique_lock<std::mutex> temp_lock{ temp_mtx };
        this->mt.mapping_threads_waiting++;
        while(this->state.threads_running.load() && !this->mt.mapping_notifier_status.load())
        {
            this->mt.mapping_update_notifier.wait(temp_lock);
        }
        this->mt.mapping_threads_waiting--;
        const uint32_t notify_state = this->mt.mapping_notifier_status.load();  // technically incorrect since two threads could both chose the first branch if timed perfectly
        if(!this->state.threads_running.load() || notify_state & MAPPING_THREAD_WAIT_UPDATE_RESOURCES)  // (need to atomically check AND update notify state) -- but this is extremely rare and not catastrophic so whatev
        {
            this->mt.mapping_notifier_status &= ~(notify_state & MAPPING_THREAD_WAIT_UPDATE_RESOURCES);
            // this->metrics.mapping_wait_refresh_exits++;
            return;  // exit called or resources need to be updated
        }
        else    // notify_state & MAPPING_THREAD_WAIT_RETRY_LOCK
        {
            this->mt.mapping_notifier_status &= ~MAPPING_THREAD_WAIT_RETRY_LOCK;
            // this->metrics.mapping_wait_retry_exits++;
            _lock.try_lock();
        }
    }

    // std::cout << "EXHIBIT A" << std::endl;
    
    pcl::PointCloud<OdomPointType>::Ptr filtered_scan_t = std::make_shared<pcl::PointCloud<OdomPointType>>();
    pcl::transformPointCloud(*inst.filtered_scan, *filtered_scan_t, inst.odom_tf);

    auto map_cloud_ptr = this->mapping.map_octree.getInputCloud();
    if(map_cloud_ptr->empty())
    {
        this->mapping.map_octree.addPoints(filtered_scan_t);
        return;
    }

    // std::cout << "EXHIBIT B" << std::endl;

    const Eigen::Vector3f lidar_origin = inst.odom_tf * inst.lidar_off;

    // std::cout << "EXHIBIT C" << std::endl;

    thread_local pcl::Indices search_indices;
    thread_local std::vector<float> dists;
    search_indices.clear();
    dists.clear();

    MappingPointType lp;
    lp.getVector3fMap() = lidar_origin;
    this->mapping.map_octree.radiusSearch(
        lp,
        this->param.mapping_valid_range,
        search_indices, dists );

    // std::cout << "EXHIBIT D" << std::endl;

    if(!this->mapping.submap_ranges) this->mapping.submap_ranges = std::make_shared<pcl::PointCloud<CollisionPointType>>();
    this->mapping.submap_ranges->reserve(search_indices.size());
    this->mapping.submap_ranges->clear();

    for(size_t i = 0; i < search_indices.size(); i++)
    {
        auto& v = this->mapping.submap_ranges->points.emplace_back();
        v.curvature = std::sqrt(dists[i]);
        v.label = search_indices[i];

        const auto& p = map_cloud_ptr->points[v.label];
        v.getNormalVector3fMap() = (p.getVector3fMap() - lidar_origin);
        v.getVector3fMap() = v.getNormalVector3fMap().normalized();
    }
    this->mapping.submap_ranges->width = this->mapping.submap_ranges->points.size();

    // std::cout << "EXHIBIT E" << std::endl;

    this->mapping.collision_kdtree.setInputCloud(this->mapping.submap_ranges);

    // std::cout << "EXHIBIT F" << std::endl;

    thread_local std::set<pcl::index_t> submap_remove_indices;
    thread_local pcl::Indices points_to_add;
    auto& scan_vec = filtered_scan_t->points;

    submap_remove_indices.clear();
    points_to_add.reserve(scan_vec.size());
    points_to_add.clear();

    for(size_t i = 0; i < scan_vec.size(); i++)
    {
        CollisionPointType p;
        p.getNormalVector3fMap() = (scan_vec[i].getVector3fMap() - lidar_origin);
        p.curvature = p.getNormalVector3fMap().norm();
        p.getVector3fMap() = p.getNormalVector3fMap().normalized();

        this->mapping.collision_kdtree.radiusSearch(p, this->param.mapping_frustum_search_radius, search_indices, dists);
        for(pcl::index_t k : search_indices)
        {
            if(p.curvature - this->mapping.submap_ranges->points[k].curvature > this->param.mapping_delete_range_thresh)
            {
                submap_remove_indices.insert(this->mapping.submap_ranges->points[k].label);
            }
        }

        if(p.curvature <= this->param.mapping_add_max_range)
        {
            points_to_add.push_back(i);
        }
    }

    // std::cout << "EXHIBIT G" << std::endl;

    for(auto itr = submap_remove_indices.begin(); itr != submap_remove_indices.end(); itr++)
    {
        this->mapping.map_octree.deletePoint(*itr);
    }

    // std::cout << "EXHIBIT H" << std::endl;

    std::shared_ptr<pcl::Indices> _points_to_add = std::shared_ptr<pcl::Indices>(&points_to_add, [](auto x){ (void)x; });
    this->mapping.map_octree.addPoints(filtered_scan_t, _points_to_add);

    this->mapping.map_octree.normalizeCloud();

    // RCLCPP_INFO(this->get_logger(),
    //     "[MAPPING]: Map points: %lu, search points: %lu, deleted points: %lu, added points: %lu,\n"
    //     "\tholes added: %lu, holes removed: %lu, voxel attempts: %lu",
    //     map_cloud_ptr->size(),
    //     this->mapping.submap_ranges->size(),
    //     submap_remove_indices.size(),
    //     filtered_scan_t->size(),
    //     this->mapping.map_octree.holes_added.load(),
    //     this->mapping.map_octree.holes_removed.load(),
    //     this->mapping.map_octree.voxel_attempts.load() );

    if(!this->param.rebias_scan_pub_prereq || this->state.has_rebiased || !this->param.use_tag_detections)
    {
        try
        {
            sensor_msgs::msg::PointCloud2 output;
            pcl::toROSMsg(*map_cloud_ptr, output);
            output.header.stamp = util::toTimeStamp(inst.stamp);
            output.header.frame_id = this->odom_frame;
            this->map_cloud_pub->publish(output);

            pcl::toROSMsg(*filtered_scan_t, output);
            output.header.stamp = util::toTimeStamp(inst.stamp);
            output.header.frame_id = this->odom_frame;
            this->filtered_scan_pub->publish(output);
        }
        catch(const std::exception& e)
        {
            RCLCPP_INFO(this->get_logger(), "[MAPPING]: Failed to publish mapping debug scans -- what():\n\t%s", e.what());
        }
    }

    _lock.unlock();
    if(this->mt.mapping_threads_waiting.load() > 0)
    {
        this->mt.mapping_notifier_status |= MAPPING_THREAD_WAIT_RETRY_LOCK;
        this->mt.mapping_update_notifier.notify_one();
    }

    auto _end = this->appendMetricStopTime(ProcType::MAP_CB);
    this->metrics.mapping_thread.addSample(_start, _end);
    // RCLCPP_INFO(this->get_logger(), "[MAPPING]: Processing took %f milliseconds.", dt_ * 1e3);

    this->handleStatusUpdate();
}


PerceptionNode::ClockType::time_point PerceptionNode::appendMetricStartTime(ProcType type)
{
    if(type == ProcType::NUM_ITEMS) return ClockType::time_point::min();
    std::lock_guard<std::mutex> _lock{ this->metrics.thread_procs_mtx };
    auto tp = ClockType::now();

    std::thread::id _id = std::this_thread::get_id();
    auto ptr = this->metrics.thread_metric_durations.find(_id);
    if(ptr == this->metrics.thread_metric_durations.end())
    {
        auto x = this->metrics.thread_metric_durations.insert( {_id, {}} );
        if(!x.second) return tp;
        else ptr = x.first;
    }

    auto& dur_buff = ptr->second[(size_t)type];
    if(dur_buff.second > ClockType::time_point::min())
    {
        // dur_buff.first = std::chrono::duration_cast<std::chrono::duration<double>>(tp - dur_buff.second).count();
        // dur_buff.second = ClockType::time_point::min();
        // ERROR!
    }
    else
    {
        dur_buff.second = tp;
    }

    return tp;
}

PerceptionNode::ClockType::time_point PerceptionNode::appendMetricStopTime(ProcType type)
{
    if(type == ProcType::NUM_ITEMS) return ClockType::time_point::min();
    std::lock_guard<std::mutex> _lock{ this->metrics.thread_procs_mtx };
    auto tp = ClockType::now();

    std::thread::id _id = std::this_thread::get_id();
    auto ptr = this->metrics.thread_metric_durations.find(_id);
    if(ptr == this->metrics.thread_metric_durations.end())
    {
        auto x = this->metrics.thread_metric_durations.insert( {_id, {}} );
        if(!x.second) return tp;
        else ptr = x.first;
    }

    auto& dur_buff = ptr->second[(size_t)type];
    if(dur_buff.second > ClockType::time_point::min())
    {
        dur_buff.first = std::chrono::duration_cast<std::chrono::duration<double>>(tp - dur_buff.second).count();
        dur_buff.second = ClockType::time_point::min();
    }
    else
    {
        // dur_buff.second = tp;
        // ERROR!
    }

    return tp;
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
