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


using namespace util::geom::cvt::ops;


namespace csm
{
namespace perception
{

PerceptionNode::PerceptionNode() :
    Node("cardinal_perception"),
    tf_buffer{std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)},
    tf_listener{tf_buffer},
    tf_broadcaster{*this},
    lidar_odom{*this},
    transform_sync{this->tf_broadcaster},
    trav_gen{4},
    metrics_pub{this, PERCEPTION_TOPIC("metrics/"), PERCEPTION_PUBSUB_QOS},
    scan_pub{this, PERCEPTION_TOPIC(""), PERCEPTION_PUBSUB_QOS},
    pose_pub{this, PERCEPTION_TOPIC("poses/"), PERCEPTION_PUBSUB_QOS},
    thread_metrics_pub{
        this,
        PERCEPTION_TOPIC("metrics/"),
        PERCEPTION_PUBSUB_QOS}
{
    this->getParams();
    this->initPubSubs();
    this->transform_sync.setFrameIds(
        this->map_frame,
        this->odom_frame,
        this->base_frame);

#define PERCEPTION_THREADS                                                 \
    ((PERCEPTION_USE_LFD_PIPELINE > 0) + (PERCEPTION_ENABLE_MAPPING > 0) + \
     (PERCEPTION_ENABLE_TRAVERSIBILITY > 0) +                              \
     (PERCEPTION_ENABLE_PATH_PLANNING))

    this->mt.threads.reserve(PERCEPTION_THREADS);
    this->mt.threads.emplace_back(&PerceptionNode::odometry_worker, this);
    IF_LFD_ENABLED(
        this->mt.threads.emplace_back(&PerceptionNode::fiducial_worker, this);)
    IF_MAPPING_ENABLED(
        this->mt.threads.emplace_back(&PerceptionNode::mapping_worker, this);)
    IF_TRAVERSABILITY_ENABLED(this->mt.threads.emplace_back(
        &PerceptionNode::traversability_worker,
        this);)
    IF_PATH_PLANNING_ENABLED(this->mt.threads.emplace_back(
        &PerceptionNode::path_planning_worker,
        this);)
}

PerceptionNode::~PerceptionNode() { this->shutdown(); }


void PerceptionNode::shutdown()
{
    this->state.threads_running = false;

    this->mt.odometry_resources.notifyExit();
    IF_LFD_ENABLED(this->mt.fiducial_resources.notifyExit();)
    IF_MAPPING_ENABLED(this->mt.mapping_resources.notifyExit();)
    IF_TRAVERSABILITY_ENABLED(this->mt.traversibility_resources.notifyExit();)
    IF_PATH_PLANNING_ENABLED(this->mt.pplan_target_notifier.notifyExit();
                             this->mt.path_planning_resources.notifyExit();)

    for (auto& x : this->mt.threads)
    {
        if (x.joinable())
        {
            x.join();
        }
    }
}



void PerceptionNode::getParams()
{
    util::declare_param(this, "map_frame_id", this->map_frame, "map");
    util::declare_param(this, "odom_frame_id", this->odom_frame, "odom");
    util::declare_param(this, "base_frame_id", this->base_frame, "base_link");

    // --- TAG DETECTION PARAMS ------------------------------------------------
    IF_TAG_DETECTION_ENABLED(
        util::declare_param(
            this,
            "tag_usage_mode",
            this->param.tag_usage_mode,
            -1);)
    util::declare_param(
        this,
        "metrics_pub_freq",
        this->param.metrics_pub_freq,
        10.);

    // --- CROP BOUNDS ---------------------------------------------------------
    std::vector<double> _min, _max;
    util::declare_param(this, "robot_crop_filter.min", _min, {0., 0., 0.});
    util::declare_param(this, "robot_crop_filter.max", _max, {0., 0., 0.});
    this->param.base_link_crop_min = Eigen::Vector3f{
        static_cast<float>(_min[0]),
        static_cast<float>(_min[1]),
        static_cast<float>(_min[2])};
    this->param.base_link_crop_max = Eigen::Vector3f{
        static_cast<float>(_max[0]),
        static_cast<float>(_max[1]),
        static_cast<float>(_max[2])};
    this->param.use_crop_filter =
        (this->param.base_link_crop_min != this->param.base_link_crop_max);

    // --- TRAJECTORY FILTER ---------------------------------------------------
    double sample_window_s, filter_window_s, avg_linear_err_thresh,
        avg_angular_err_thresh, max_linear_deviation_thresh,
        max_angular_deviation_thresh;
    util::declare_param(
        this,
        "trajectory_filter.sampling_window_s",
        sample_window_s,
        0.5);
    util::declare_param(
        this,
        "trajectory_filter.min_filter_window_s",
        filter_window_s,
        0.3);
    util::declare_param(
        this,
        "trajectory_filter.thresh.avg_linear_error",
        avg_linear_err_thresh,
        0.2);
    util::declare_param(
        this,
        "trajectory_filter.thresh.avg_angular_error",
        avg_angular_err_thresh,
        0.1);
    util::declare_param(
        this,
        "trajectory_filter.thresh.max_linear_deviation",
        max_linear_deviation_thresh,
        4e-2);
    util::declare_param(
        this,
        "trajectory_filter.thresh.max_angular_deviation",
        max_angular_deviation_thresh,
        4e-2);
    this->transform_sync.trajectoryFilter().applyParams(
        sample_window_s,
        filter_window_s,
        avg_linear_err_thresh,
        avg_angular_err_thresh,
        max_linear_deviation_thresh,
        max_angular_deviation_thresh);

    // --- LIDAR FIDUCIAL DETECTOR ---------------------------------------------
#if LFD_ENABLED
    double lfd_range_thresh, plane_distance, eps_angle, vox_res, avg_off,
        max_remaining_proportion;
    int min_points_thresh{0}, min_seg_points_thresh{0};
    util::declare_param(
        this,
        "fiducial_detection.max_range",
        lfd_range_thresh,
        2.);
    util::declare_param(
        this,
        "fiducial_detection.plane_distance_threshold",
        plane_distance,
        0.005);
    util::declare_param(
        this,
        "fiducial_detection.plane_eps_thresh",
        eps_angle,
        0.1);
    util::declare_param(
        this,
        "fiducial_detection.vox_resolution",
        vox_res,
        0.03);
    util::declare_param(
        this,
        "fiducial_detection.avg_center_offset",
        avg_off,
        0.4);
    util::declare_param(
        this,
        "fiducial_detection.remaining_points_thresh",
        max_remaining_proportion,
        0.05);
    util::declare_param(
        this,
        "fiducial_detection.minimum_input_points",
        min_points_thresh,
        100);
    util::declare_param(
        this,
        "fiducial_detection.minimum_segmented_points",
        min_seg_points_thresh,
        15);
    this->fiducial_detector.applyParams(
        lfd_range_thresh,
        plane_distance,
        eps_angle,
        vox_res,
        avg_off,
        static_cast<size_t>(min_points_thresh),
        static_cast<size_t>(min_seg_points_thresh),
        max_remaining_proportion);
#endif

    // --- MAPPING -------------------------------------------------------------
#if MAPPING_ENABLED
    double frustum_search_radius, radial_dist_thresh, delete_delta_coeff,
        delete_max_range, add_max_range, voxel_size;
    util::declare_param(
        this,
        "mapping.frustum_search_radius",
        frustum_search_radius,
        0.01);
    util::declare_param(
        this,
        "mapping.radial_distance_thresh",
        radial_dist_thresh,
        0.01);
    util::declare_param(
        this,
        "mapping.delete_delta_coeff",
        delete_delta_coeff,
        0.1);
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
        voxel_size);
#endif

    // --- TRAVERSIBILITY ------------------------------------------------------
#if TRAVERSABILITY_ENABLED
    float norm_estimation_rad, interp_grid_res, non_trav_grad_angle,
        req_clearance, avoid_radius;
    int interp_point_samples = 0;
    util::declare_param(
        this,
        "traversibility.chunk_horizontal_range",
        this->param.map_export_horizontal_range,
        4.);
    util::declare_param(
        this,
        "traversibility.chunk_vertical_range",
        this->param.map_export_vertical_range,
        1.);
    util::declare_param(
        this,
        "traversibility.normal_estimation_radius",
        norm_estimation_rad,
        -1.f);
    util::declare_param(
        this,
        "traversibility.interp_grid_res",
        interp_grid_res,
        voxel_size);
    util::declare_param(
        this,
        "traversibility.non_trav_grad_angle",
        non_trav_grad_angle,
        45.f);
    util::declare_param(
        this,
        "traversibility.required_clearance",
        req_clearance,
        1.5f);
    util::declare_param(
        this,
        "traversibility.avoidance_radius",
        avoid_radius,
        0.5f);
    util::declare_param(
        this,
        "traversibility.interp_point_samples",
        interp_point_samples,
        7);
    if (norm_estimation_rad <= 0.f)
    {
        norm_estimation_rad = voxel_size * 2;
    }
    this->trav_gen.configure(
        norm_estimation_rad,
        interp_grid_res,
        non_trav_grad_angle,
        req_clearance,
        avoid_radius,
        interp_point_samples);
#endif
}



void PerceptionNode::initPubSubs()
{
    std::string scan_topic, imu_topic;
    util::declare_param(this, "scan_topic", scan_topic, "scan");
    util::declare_param(this, "imu_topic", imu_topic, "imu");

    this->imu_sub = this->create_subscription<ImuMsg>(
        imu_topic,
        PERCEPTION_PUBSUB_QOS,
        [this](ImuMsg::SharedPtr imu) { this->imu_worker(imu); });
    this->scan_sub = this->create_subscription<PointCloudMsg>(
        scan_topic,
        PERCEPTION_PUBSUB_QOS,
        [this](const PointCloudMsg::ConstSharedPtr& scan)
        { this->mt.odometry_resources.updateAndNotify(scan); });
#if TAG_DETECTION_ENABLED
    this->detections_sub = this->create_subscription<TagsTransformMsg>(
        "tags_detections",
        PERCEPTION_PUBSUB_QOS,
        [this](const TagsTransformMsg::ConstSharedPtr& det)
        { this->detection_worker(det); });
#endif

    this->path_plan_service = this->create_service<UpdatePathPlanSrv>(
        PERCEPTION_TOPIC("update_path_planning"),
        [this](
            UpdatePathPlanSrv::Request::SharedPtr req,
            UpdatePathPlanSrv::Response::SharedPtr resp)
        {
            if (req->completed)
            {
                this->state.pplan_enabled = false;
            }
            else
            {
                this->state.pplan_enabled = true;
                this->mt.pplan_target_notifier.updateAndNotify(req->target);
            }

            resp->running = this->state.pplan_enabled;
        });

    this->velocity_pub = this->create_publisher<TwistStampedMsg>(
        PERCEPTION_TOPIC("odom_velocity"),
        PERCEPTION_PUBSUB_QOS);

    this->proc_metrics_pub = this->create_publisher<ProcessMetricsMsg>(
        PERCEPTION_TOPIC("metrics/process_stats"),
        PERCEPTION_PUBSUB_QOS);

    this->traj_filter_debug_pub =
        this->create_publisher<TrajectoryFilterDebugMsg>(
            PERCEPTION_TOPIC("metrics/trajectory_filter_stats"),
            PERCEPTION_PUBSUB_QOS);

    this->path_plan_pub = this->create_publisher<PathMsg>(
        PERCEPTION_TOPIC("planned_path"),
        PERCEPTION_PUBSUB_QOS);
}




void PerceptionNode::handleStatusUpdate()
{
    // try lock mutex
    if (!this->state.print_mtx.try_lock())
    {
        return;
    }

    // check frequency
    auto _tp =
        this->metrics.manager.registerProcStart(ProcType::HANDLE_METRICS);
    const double _dt = util::toFloatSeconds(_tp - this->state.last_print_time);
    if (_dt <= (1. / this->param.metrics_pub_freq))
    {
        this->metrics.manager.registerProcEnd(ProcType::HANDLE_METRICS);
        this->state.print_mtx.unlock();
        return;
    }

    this->state.last_print_time = _tp;

    double resident_set_mb = 0.;
    size_t num_threads = 0;
    double cpu_temp = 0.;

    this->metrics.process_utilization.update();
    util::proc::getProcessStats(resident_set_mb, num_threads);
#ifdef HAS_SENSORS
    cpu_temp = util::proc::readCpuTemp();
#endif

#if PERCEPTION_PRINT_STATUS_DISPLAY
    std::ostringstream msg;

    // clang-format off
    msg << std::setprecision(2) << std::fixed << std::right << std::setfill(' ') << '\n';
    msg << "+-------------------------------------------------------------------+\n"
           "| =================== Cardinal Perception v0.6.0 ================== |\n"
           "+- RESOURCES -------------------------------------------------------+\n"
           "|                      ::  Current  |  Average  |  Maximum          |\n";
    msg << "|      CPU Utilization :: " << std::setw(6) << (this->metrics.process_utilization.last_cpu_percent)
                                        << " %  | " << std::setw(6) << this->metrics.process_utilization.avg_cpu_percent
                                                    << " %  |   " << std::setw(5) << this->metrics.process_utilization.max_cpu_percent
                                                                 << " %         |\n";
    msg << "|      CPU Temperature :: " << std::setw(6) << cpu_temp
                                        << "*C  |                               |\n";
    msg << "|       RAM Allocation :: " << std::setw(6) << resident_set_mb
                                        << " MB |                               |\n";
    msg << "|        Total Threads ::  " << std::setw(5) << num_threads
                                        << "    |                               |\n";
    msg << "|                                                                   |\n"
           "+- CALLBACKS -------------------------------------------------------+\n"
           "|                        Comp. Time | Avg. Time | Max Time | Total  |\n";
    msg << std::setprecision(1) << std::fixed << std::right << std::setfill(' ');

    static constexpr char const*
        PROC_STRINGS[] =
        {
            "|   IMU CB (",
            "|  SCAN CB (",
            "|   DET CB (",
            "|   FID CB (",
            "|   MAP CB (",
            "|  TRAV CB (",
            "| PPLAN CB (",
            "",
            ""
        };
    // clang-format on

    std::unique_lock<std::mutex> lock_;
    const auto& proc_stats_map =
        this->metrics.manager.readProcStatistics(lock_);
    for (const auto& proc_stat : proc_stats_map)
    {
        const char* proc_string =
            PROC_STRINGS[static_cast<size_t>(proc_stat.first)];
        if (!*proc_string)
        {
            continue;
        }

        // clang-format off
        msg << proc_string << std::setw(5) << (1. / proc_stat.second.avg_call_delta)
                           << " Hz) ::  " << std::setw(5) << (proc_stat.second.last_comp_time * 1e3)
                                       << " ms  | " << std::setw(5) << (proc_stat.second.avg_comp_time * 1e3)
                                                   << " ms  | " << std::setw(5) << (proc_stat.second.max_comp_time * 1e3)
                                                               << " ms | " << std::setw(6) << (proc_stat.second.samples)
                                                                           << " |\n";
        // clang-format on
    }
    msg << "|                                                                   |\n"
           "+- THREAD UTILIZATION ----------------------------------------------+\n"
           "|                                                                   |\n";

    static constexpr char PROC_CHARS[] = {
        'I',  // imu
        'S',  // scan
        'D',  // tags [detection]
        'F',  // lfd
        'M',  // map
        'T',  // trav
        'P',  // pplan
        'X',  // metrics
        'm'   // misc
    };
    static constexpr char const* PROC_COLORS[] = {
        "\033[38;5;117m",  // imu
        "\033[38;5;47m",   // scan
        "\033[38;5;9m",    // tags
        "\033[38;5;197m",  // lfd
        "\033[38;5;99m",   // map
        "\033[38;5;208m",  // trav
        "\033[38;5;220m",  // pplan
        "\033[38;5;80m",   // metrics
        "\033[37m"         // misc
    };

    size_t idx = 0;
    auto& thread_durations_map =
        this->metrics.manager.getThreadDurations(lock_);
    for (auto& thread_durs : thread_durations_map)
    {
        msg << "| " << std::setw(3) << idx++ << ": [";  // start -- 8 chars
        // fill -- 58 chars
        size_t avail_chars = 58;
        for (auto& proc_dur : thread_durs.second)
        {
            auto& d = proc_dur.second;
            if (d.second > ClockType::time_point::min() && d.second < _tp)
            {
                d.first +=
                    std::chrono::duration<double>(_tp - d.second).count();
                d.second = _tp;
            }

            const double fn_chars = d.first / _dt * 58.;
            size_t n_chars = static_cast<size_t>(
                fn_chars > 0. ? std::max(fn_chars, 1.) : 0.);
            n_chars = std::min(n_chars, avail_chars);
            avail_chars -= n_chars;

            d.first =
                0.;  // TODO: this is wrong if the thread takes longer to process than the metrics pub freq

            if (n_chars > 0)
            {
                const size_t x = static_cast<size_t>(proc_dur.first);
                msg << PROC_COLORS[x] << std::setfill('=') << std::setw(n_chars)
                    << PROC_CHARS[x];
            }
        }
        msg << "\033[0m";
        if (avail_chars > 0)
        {
            msg << std::setfill(' ') << std::setw(avail_chars) << ' ';
        }
        msg << "] |\n";  // end -- 3 chars
    }
    lock_.unlock();

    msg << "+-------------------------------------------------------------------+\n";

    std::cout << "\033[2J\033[1;1H" << std::endl;
    std::string msg_str = msg.str();
    RCLCPP_INFO(this->get_logger(), "%s", msg_str.c_str());
#else
    static bool has_printed_disclaimer = false;
    if (!has_printed_disclaimer)
    {
        std::ostringstream msg;
        msg << "\n"
               "+-------------------------------------------------------------------+\n"
               "|                                                                   |\n"
               "|          CARDINAL PERCEPTION STATUS PRINTING IS DISABLED.         |\n"
               "|                                                                   |\n"
               "+-------------------------------------------------------------------+";
        std::string msg_str = msg.str();
        RCLCPP_INFO(this->get_logger(), "%s", msg_str.c_str());
        has_printed_disclaimer = true;
    }
#endif

    this->publishMetrics(resident_set_mb, num_threads, cpu_temp);

    this->metrics.manager.registerProcEnd(ProcType::HANDLE_METRICS);
    this->state.print_mtx.unlock();
}



void PerceptionNode::publishMetrics(
    double mem_usage,
    size_t n_threads,
    double cpu_temp)
{
    ProcessMetricsMsg pm;
    pm.cpu_percent =
        static_cast<float>(this->metrics.process_utilization.last_cpu_percent);
    pm.avg_cpu_percent =
        static_cast<float>(this->metrics.process_utilization.avg_cpu_percent);
    pm.mem_usage_mb = static_cast<float>(mem_usage);
    pm.num_threads = static_cast<uint32_t>(n_threads);
    pm.cpu_temp = static_cast<float>(cpu_temp);
    this->proc_metrics_pub->publish(pm);

    static constexpr char const* TOPICS[] = {
        "imu_cb_stats",
        "scan_cb_stats",
        "det_cb_stats",
        "fiducial_cb_stats",
        "mapping_cb_stats",
        "trav_cb_stats",
        "pplan_cb_stats",
        "",
        ""};

    this->metrics.manager.publishThreadMetrics(
        this->thread_metrics_pub,
        [&](ProcType p) { return TOPICS[static_cast<size_t>(p)]; });
}





// --- THREAD LOOPS -----------------------------------------------------------

void PerceptionNode::odometry_worker()
{
    do
    {
        auto& scan = this->mt.odometry_resources.waitNewestResource();
        if (!this->state.threads_running.load())
        {
            return;
        }

        PROFILING_NOTIFY_BASIC(odometry);
        this->metrics.manager.registerProcStart(ProcType::SCAN_CB);
        {
            this->scan_callback_internal(scan);
        }
        this->metrics.manager.registerProcEnd(ProcType::SCAN_CB, true);
        PROFILING_NOTIFY_BASIC(odometry);

        this->handleStatusUpdate();

    } while (this->state.threads_running.load());
}



#if LFD_ENABLED
void PerceptionNode::fiducial_worker()
{
    do
    {
        auto& buff = this->mt.fiducial_resources.waitNewestResource();
        if (!this->state.threads_running.load())
        {
            return;
        }

        PROFILING_NOTIFY_BASIC(lidar_fiducial);
        this->metrics.manager.registerProcStart(ProcType::FID_CB);
        {
            this->fiducial_callback_internal(buff);
        }
        this->metrics.manager.registerProcEnd(ProcType::FID_CB, true);
        PROFILING_NOTIFY_BASIC(lidar_fiducial);

        this->handleStatusUpdate();

    } while (this->state.threads_running.load());
}
#endif



#if MAPPING_ENABLED
void PerceptionNode::mapping_worker()
{
    do
    {
        auto& buff = this->mt.mapping_resources.waitNewestResource();
        if (!this->state.threads_running.load())
        {
            return;
        }

        PROFILING_NOTIFY_BASIC(mapping);
        this->metrics.manager.registerProcStart(ProcType::MAP_CB);
        {
            this->mapping_callback_internal(buff);
        }
        this->metrics.manager.registerProcEnd(ProcType::MAP_CB, true);
        PROFILING_NOTIFY_BASIC(mapping);

        this->handleStatusUpdate();

    } while (this->state.threads_running.load());
}
#endif



#if TRAVERSABILITY_ENABLED
void PerceptionNode::traversability_worker()
{
    do
    {
        auto& buff = this->mt.traversibility_resources.waitNewestResource();
        if (!this->state.threads_running.load())
        {
            return;
        }

        PROFILING_NOTIFY_BASIC(traversibility);
        this->metrics.manager.registerProcStart(ProcType::TRAV_CB);
        {
            this->traversibility_callback_internal(buff);
        }
        this->metrics.manager.registerProcEnd(ProcType::TRAV_CB, true);
        PROFILING_NOTIFY_BASIC(traversibility);

        this->handleStatusUpdate();

    } while (this->state.threads_running.load());
}
#endif



#if PATH_PLANNING_ENABLED
void PerceptionNode::path_planning_worker()
{
    do
    {
        if (this->state.pplan_enabled)
        {
            auto& buff = this->mt.path_planning_resources.waitNewestResource();
            if (!this->state.threads_running.load())
            {
                return;
            }

            buff.target = this->mt.pplan_target_notifier.aquireNewestOutput();

            this->metrics.manager.registerProcStart(ProcType::PPLAN_CB);
            {
                this->path_planning_callback_internal(buff);
            }
            this->metrics.manager.registerProcEnd(ProcType::PPLAN_CB, true);

            this->handleStatusUpdate();
        }
        else
        {
            this->mt.pplan_target_notifier.waitNewestResource();
        }

    } while (this->state.threads_running.load());
}
#endif

};  // namespace perception
};  // namespace csm
