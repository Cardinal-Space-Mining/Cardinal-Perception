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

static constexpr char const* STARTUP_SPLASH =
    "\n"
    "  CARDINAL           ;xxxxxxx:                      \n"
    "  PERCEPTION        ;$$$$$$$$$       ...::..        \n"
    "  " PERCEPTION_VERSION_STR "            $$$$$$$$$$x   .:::::::::::..    \n"
    "                 x$$$$$$$$$$$$$$::::::::::::::::.   \n"
    "             :$$$$$&X;      .xX:::::::::::::.::...  \n"
    "     .$$Xx++$$$$+  :::.     :;:   .::::::.  ....  : \n"
    "    :$$$$$$$$$  ;:      ;xXXXXXXXx  .::.  .::::. .:.\n"
    "   :$$$$$$$$: ;      ;xXXXXXXXXXXXXx: ..::::::  .::.\n"
    "  ;$$$$$$$$ ::   :;XXXXXXXXXXXXXXXXXX+ .::::.  .::: \n"
    "   X$$$$$X : +XXXXXXXXXXXXXXXXXXXXXXXX; .::  .::::. \n"
    "    .$$$$ :xXXXXXXXXXXXXXXXXXXXXXXXXXXX.   .:::::.  \n"
    "     X$$X XXXXXXXXXXXXXXXXXXXXXXXXXXXXx:  .::::.    \n"
    "     $$$:.XXXXXXXXXXXXXXXXXXXXXXXXXXX  ;; ..:.      \n"
    "     $$& :XXXXXXXXXXXXXXXXXXXXXXXX;  +XX; X$$;      \n"
    "     $$$: XXXXXXXXXXXXXXXXXXXXXX; :XXXXX; X$$;      \n"
    "     X$$X XXXXXXXXXXXXXXXXXXX; .+XXXXXXX; $$$       \n"
    "     $$$$ ;XXXXXXXXXXXXXXX+  +XXXXXXXXx+ X$$$+      \n"
    "   x$$$$$X ;XXXXXXXXXXX+ :xXXXXXXXX+   .;$$$$$$     \n"
    "  +$$$$$$$$ ;XXXXXXx;;+XXXXXXXXX+    : +$$$$$$$$    \n"
    "   +$$$$$$$$: xXXXXXXXXXXXXXX+      ; X$$$$$$$$     \n"
    "    :$$$$$$$$$. +XXXXXXXXX;      ;: x$$$$$$$$$      \n"
    "    ;x$$$$XX$$$$+ .;+X+      :;: :$$$$$xX$$$X       \n"
    "   ;;;;;;;;;;X$$$$$$$+      :X$$$$$$&.              \n"
    "   ;;;;;;;:;;;;;x$$$$$$$$$$$$$$$$x.                 \n"
    "   :;;;;;;;;;;;;.  :$$$$$$$$$$X                     \n"
    "    .;;;;;;;;:;;    +$$$$$$$$$                      \n"
    "      .;;;;;;.       X$$$$$$$:                      \n"
    "\n"
    "  Copyright (C) 2024-2025 Cardinal Space Mining Club\n";

struct PerceptionConfig
{
public:
    explicit PerceptionConfig(PerceptionNode& node) : node{node} {}

    inline static PerceptionConfig& castOrAllocate(
        void* buff,
        PerceptionNode& node)
    {
        PerceptionConfig* config_ptr;
        if (!buff)
        {
            config_ptr = new PerceptionConfig(node);
        }
        else
        {
            config_ptr = static_cast<PerceptionConfig*>(buff);
        }
        return *config_ptr;
    }
    inline static void handleDeallocate(void* buff, PerceptionConfig& config)
    {
        if (!buff)
        {
            delete &config;
        }
    }

public:
    PerceptionNode& node;

    // core
    std::string scan_topic;
    std::string imu_topic;

    float metrics_pub_freq;

    std::vector<double> crop_bbox_min;
    std::vector<double> crop_bbox_max;

    // trajectory filter
    double trjf_sample_window_s;
    double trjf_filter_window_s;
    double trjf_avg_linear_err_thresh;
    double trjf_avg_angular_err_thresh;
    double trjf_max_linear_dev_thresh;
    double trjf_max_angular_dev_thresh;

    // lidar fiducial detector
    double lfd_range_thresh;
    double lfd_plane_dist;
    double lfd_eps_angle;
    double lfd_vox_res;
    double lfd_avg_off;
    double lfd_max_remaining_proportion;
    int lfd_min_points_thresh;
    int lfd_min_seg_points_thresh;

    // mapping
    double kfc_frustum_search_radius;
    double kfc_radial_dist_thresh;
    double kfc_delete_delta_coeff;
    double kfc_delete_max_range;
    double kfc_add_max_range;
    double kfc_voxel_size;

    // traversibility
    float trav_norm_estimation_radius;
    float trav_interp_grid_res;
    float trav_avoid_grad_angle;
    float trav_req_clearance;
    float trav_avoid_radius;
    int trav_interp_point_samples;

public:
    static inline constexpr char const* getFiducialModeStr()
    {
#if PERCEPTION_USE_TAG_DETECTION_PIPELINE
        return "AprilTag";
#elif PERCEPTION_USE_LFD_PIPELINE
        return "Lidar fiducial";
#else
        return "Disabled";
#endif
    }
    static inline constexpr char const* getEnableDisableStr(int val)
    {
        return val ? "Enabled" : "Disabled";
    }
};


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
    pose_pub{this, PERCEPTION_TOPIC("poses/"), PERCEPTION_PUBSUB_QOS}
{
    PerceptionConfig config{*this};

    this->getParams(&config);
    this->initPubSubs(&config);
    this->transform_sync.setFrameIds(
        this->map_frame,
        this->odom_frame,
        this->base_frame);

    this->printStartup(&config);

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



void PerceptionNode::getParams(void* buff)
{
    PerceptionConfig& config = PerceptionConfig::castOrAllocate(buff, *this);

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
    util::declare_param(this, "metrics_pub_freq", config.metrics_pub_freq, 10.);

    // --- CROP BOUNDS ---------------------------------------------------------
    util::declare_param(
        this,
        "robot_crop_filter.min",
        config.crop_bbox_min,
        {0., 0., 0.});
    util::declare_param(
        this,
        "robot_crop_filter.max",
        config.crop_bbox_max,
        {0., 0., 0.});
    this->param.base_link_crop_min = Eigen::Vector3f{
        static_cast<float>(config.crop_bbox_min[0]),
        static_cast<float>(config.crop_bbox_min[1]),
        static_cast<float>(config.crop_bbox_min[2])};
    this->param.base_link_crop_max = Eigen::Vector3f{
        static_cast<float>(config.crop_bbox_max[0]),
        static_cast<float>(config.crop_bbox_max[1]),
        static_cast<float>(config.crop_bbox_max[2])};
    this->param.use_crop_filter =
        (this->param.base_link_crop_min != this->param.base_link_crop_max);

    // --- TRAJECTORY FILTER ---------------------------------------------------
    util::declare_param(
        this,
        "trajectory_filter.sampling_window_s",
        config.trjf_sample_window_s,
        0.5);
    util::declare_param(
        this,
        "trajectory_filter.min_filter_window_s",
        config.trjf_filter_window_s,
        0.3);
    util::declare_param(
        this,
        "trajectory_filter.thresh.avg_linear_error",
        config.trjf_avg_linear_err_thresh,
        0.2);
    util::declare_param(
        this,
        "trajectory_filter.thresh.avg_angular_error",
        config.trjf_avg_angular_err_thresh,
        0.1);
    util::declare_param(
        this,
        "trajectory_filter.thresh.max_linear_deviation",
        config.trjf_max_linear_dev_thresh,
        4e-2);
    util::declare_param(
        this,
        "trajectory_filter.thresh.max_angular_deviation",
        config.trjf_max_angular_dev_thresh,
        4e-2);
    this->transform_sync.trajectoryFilter().applyParams(
        config.trjf_sample_window_s,
        config.trjf_filter_window_s,
        config.trjf_avg_linear_err_thresh,
        config.trjf_avg_angular_err_thresh,
        config.trjf_max_linear_dev_thresh,
        config.trjf_max_angular_dev_thresh);

    // --- LIDAR FIDUCIAL DETECTOR ---------------------------------------------
#if LFD_ENABLED
    util::declare_param(
        this,
        "fiducial_detection.max_range",
        config.lfd_range_thresh,
        2.);
    util::declare_param(
        this,
        "fiducial_detection.plane_distance_threshold",
        config.lfd_plane_dist,
        0.005);
    util::declare_param(
        this,
        "fiducial_detection.plane_eps_thresh",
        config.lfd_eps_angle,
        0.1);
    util::declare_param(
        this,
        "fiducial_detection.vox_resolution",
        config.lfd_vox_res,
        0.03);
    util::declare_param(
        this,
        "fiducial_detection.avg_center_offset",
        config.lfd_avg_off,
        0.4);
    util::declare_param(
        this,
        "fiducial_detection.remaining_points_thresh",
        config.lfd_max_remaining_proportion,
        0.05);
    util::declare_param(
        this,
        "fiducial_detection.minimum_input_points",
        config.lfd_min_points_thresh,
        100);
    util::declare_param(
        this,
        "fiducial_detection.minimum_segmented_points",
        config.lfd_min_seg_points_thresh,
        15);
    this->fiducial_detector.applyParams(
        config.lfd_range_thresh,
        config.lfd_plane_dist,
        config.lfd_eps_angle,
        config.lfd_vox_res,
        config.lfd_avg_off,
        static_cast<size_t>(config.lfd_min_points_thresh),
        static_cast<size_t>(config.lfd_min_seg_points_thresh),
        config.lfd_max_remaining_proportion);
#endif

    // --- MAPPING -------------------------------------------------------------
#if MAPPING_ENABLED
    util::declare_param(
        this,
        "mapping.frustum_search_radius",
        config.kfc_frustum_search_radius,
        0.01);
    util::declare_param(
        this,
        "mapping.radial_distance_thresh",
        config.kfc_radial_dist_thresh,
        0.01);
    util::declare_param(
        this,
        "mapping.delete_delta_coeff",
        config.kfc_delete_delta_coeff,
        0.1);
    util::declare_param(
        this,
        "mapping.delete_max_range",
        config.kfc_delete_max_range,
        4.);
    util::declare_param(
        this,
        "mapping.add_max_range",
        config.kfc_add_max_range,
        4.);
    util::declare_param(this, "mapping.voxel_size", config.kfc_voxel_size, 0.1);
    this->environment_map.applyParams(
        config.kfc_frustum_search_radius,
        config.kfc_radial_dist_thresh,
        config.kfc_delete_delta_coeff,
        0.,
        config.kfc_delete_max_range,
        config.kfc_add_max_range,
        config.kfc_voxel_size);
#endif

    // --- TRAVERSIBILITY ------------------------------------------------------
#if TRAVERSABILITY_ENABLED
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
        config.trav_norm_estimation_radius,
        -1.f);
    util::declare_param(
        this,
        "traversibility.interp_grid_res",
        config.trav_interp_grid_res,
        config.kfc_voxel_size);
    util::declare_param(
        this,
        "traversibility.non_trav_grad_angle",
        config.trav_avoid_grad_angle,
        45.f);
    util::declare_param(
        this,
        "traversibility.required_clearance",
        config.trav_req_clearance,
        1.5f);
    util::declare_param(
        this,
        "traversibility.avoidance_radius",
        config.trav_avoid_radius,
        0.5f);
    util::declare_param(
        this,
        "traversibility.interp_point_samples",
        config.trav_interp_point_samples,
        7);
    if (config.trav_norm_estimation_radius <= 0.f)
    {
        config.trav_norm_estimation_radius = config.kfc_voxel_size * 2;
    }
    this->trav_gen.configure(
        config.trav_norm_estimation_radius,
        config.trav_interp_grid_res,
        config.trav_avoid_grad_angle,
        config.trav_req_clearance,
        config.trav_avoid_radius,
        config.trav_interp_point_samples);
#endif

    PerceptionConfig::handleDeallocate(buff, config);
}



void PerceptionNode::initPubSubs(void* buff)
{
    PerceptionConfig& config = PerceptionConfig::castOrAllocate(buff, *this);

    util::declare_param(this, "scan_topic", config.scan_topic, "scan");
    util::declare_param(this, "imu_topic", config.imu_topic, "imu");

    this->imu_sub = this->create_subscription<ImuMsg>(
        config.imu_topic,
        PERCEPTION_PUBSUB_QOS,
        [this](ImuMsg::SharedPtr imu) { this->imu_worker(imu); });
    this->scan_sub = this->create_subscription<PointCloudMsg>(
        config.scan_topic,
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

    this->proc_stats_pub = this->create_publisher<ProcessStatsMsg>(
        PERCEPTION_TOPIC("process_stats"),
        PERCEPTION_PUBSUB_QOS);

    this->traj_filter_debug_pub =
        this->create_publisher<TrajectoryFilterDebugMsg>(
            PERCEPTION_TOPIC("metrics/trajectory_filter_stats"),
            PERCEPTION_PUBSUB_QOS);

    this->path_plan_pub = this->create_publisher<PathMsg>(
        PERCEPTION_TOPIC("planned_path"),
        PERCEPTION_PUBSUB_QOS);

    this->proc_stats_timer = this->create_wall_timer(
        std::chrono::milliseconds(
            static_cast<int>(1000.f / config.metrics_pub_freq)),
        [this]()
        {
            this->process_stats.update();
            this->proc_stats_pub->publish(this->process_stats.toMsg());
        });

    PerceptionConfig::handleDeallocate(buff, config);
}



void PerceptionNode::printStartup(void* buff)
{
    std::ostringstream msg;
    msg << "\n" << STARTUP_SPLASH;

#if PERCEPTION_PRINT_STARTUP_CONFIGS
    // alignment, feat. ChatGPT lol
    constexpr int CONFIG_ALIGN_WIDTH = 25;
    auto align = [](const char* label)
    {
        std::ostringstream oss;
        oss << " |  " << std::left << std::setw(CONFIG_ALIGN_WIDTH) << label
            << ": ";
        return oss.str();
    };

    if (buff)
    {
        PerceptionConfig& config = *static_cast<PerceptionConfig*>(buff);

        msg << std::setprecision(3);
        msg << "\n"
               " +-- CONFIGURATION ---------------------------------+\n"
               " |\n"
               " +- PIPELINE STAGES\n"
            << align("Fiducial mode") << PerceptionConfig::getFiducialModeStr()
            << "\n"
            << align("Mapping")
            << PerceptionConfig::getEnableDisableStr(PERCEPTION_ENABLE_MAPPING)
            << "\n"
            << align("Traversibility")
            << PerceptionConfig::getEnableDisableStr(
                   PERCEPTION_ENABLE_TRAVERSIBILITY)
            << "\n"
            << align("Path Planning")
            << PerceptionConfig::getEnableDisableStr(
                   PERCEPTION_ENABLE_PATH_PLANNING)
            << "\n"
               " |\n"
               " +- FEATURES\n"
            << align("Scan Deskew")
            << PerceptionConfig::getEnableDisableStr(PERCEPTION_USE_SCAN_DESKEW)
            << "\n"
            << align("Null Ray Mapping")
            << PerceptionConfig::getEnableDisableStr(
                   PERCEPTION_USE_NULL_RAY_DELETION)
            << "\n"
               " |\n"
               " +- CORE\n"
            << align("Scan Topic") << config.scan_topic << "\n"
            << align("IMU Topic") << config.imu_topic << "\n"
            << align("Map Frame ID") << this->map_frame << "\n"
            << align("Odom Frame ID") << this->odom_frame << "\n"
            << align("Robot Frame ID") << this->base_frame << "\n"
            << align("Statistics Frequency") << config.metrics_pub_freq
            << " hz\n"
               " |\n"
               " +- SCAN CROPBOX\n"
            << align("Base-link Min") << "[" << config.crop_bbox_min[0] << ", "
            << config.crop_bbox_min[1] << ", " << config.crop_bbox_min[2]
            << "] (m)\n"
            << align("Base-link Max") << "[" << config.crop_bbox_max[0] << ", "
            << config.crop_bbox_max[1] << ", " << config.crop_bbox_max[2]
            << "] (m)\n"
               " |\n"
               " +- TRAJECTORY FILTER\n"
            << align("Sample Window") << config.trjf_sample_window_s
            << " seconds\n"
            << align("Filter Window") << config.trjf_filter_window_s
            << " seconds\n"
            << align("Avg Linear Error Thresh")
            << config.trjf_avg_linear_err_thresh << " meters\n"
            << align("Avg Angular Error Thresh")
            << config.trjf_avg_angular_err_thresh << " radians\n"
            << align("Max Linear Dev Thresh")
            << config.trjf_max_linear_dev_thresh << "\n"
            << align("Max Angular Dev Thresh")
            << config.trjf_max_angular_dev_thresh << "\n";

    #if PERCEPTION_USE_LFD_PIPELINE
        msg << " |\n"
               " +- LIDAR FIDUCIAL DETECTOR\n"
            << align("Range Threshold") << config.lfd_range_thresh
            << " meters\n"
            << align("Plane Distance") << config.lfd_plane_dist << " meters\n"
            << align("Plane Eps Angle") << config.lfd_eps_angle << " radians\n"
            << align("Voxel Resolution") << config.lfd_vox_res << " meters\n"
            << align("Avg Offset") << config.lfd_avg_off << " meters\n"
            << align("Max Remaining Percentage")
            << (config.lfd_max_remaining_proportion * 100) << "%\n"
            << align("Min Points Thresh") << config.lfd_min_points_thresh
            << "\n"
            << align("Min Seg Points Thresh")
            << config.lfd_min_seg_points_thresh << "\n";
    #endif

    #if PERCEPTION_ENABLE_MAPPING
        msg << " |\n"
               " +- MAPPING\n"
            << align("Frustum Radius") << config.kfc_frustum_search_radius
            << " radians\n"
            << align("Radial Dist Thresh") << config.kfc_radial_dist_thresh
            << " meters\n"
            << align("Delete Coefficient") << config.kfc_delete_delta_coeff
            << "\n"
            << align("Delete Max Range") << config.kfc_delete_max_range
            << " meters\n"
            << align("Add Max Range") << config.kfc_add_max_range << " meters\n"
            << align("Voxel Size") << config.kfc_voxel_size << " meters\n";
    #endif

    #if PERCEPTION_ENABLE_TRAVERSIBILITY
        msg << " |\n"
               " +- TRAVERSIBILITY\n"
            << align("Horizontal Export Range")
            << this->param.map_export_horizontal_range << " meters\n"
            << align("Vertical Export Range")
            << this->param.map_export_vertical_range << " meters\n"
            << align("Normal Est Radius") << config.trav_norm_estimation_radius
            << " meters\n"
            << align("Interp Grid Res") << config.trav_interp_grid_res
            << " meters\n"
            << align("Avoid Grad Angle") << config.trav_avoid_grad_angle
            << " degrees\n"
            << align("Required Clearance") << config.trav_req_clearance
            << " meters\n"
            << align("Avoid Radius") << config.trav_avoid_radius << " meters\n"
            << align("Point Samples") << config.trav_interp_point_samples
            << "\n";
    #endif

        msg << " +\n";
    }
#else
    (void)buff;
#endif

    RCLCPP_INFO(
        this->get_logger(),
        "[CORE]: Initialization complete.%s",
        msg.str().c_str());
}





// --- THREAD LOOPS -----------------------------------------------------------

void PerceptionNode::odometry_worker()
{
    RCLCPP_INFO(
        this->get_logger(),
        "[CORE]: Odometry thread started successfully.");

    do
    {
        auto& scan = this->mt.odometry_resources.waitNewestResource();
        if (!this->state.threads_running.load())
        {
            return;
        }

        PROFILING_SYNC();
        PROFILING_NOTIFY_ALWAYS(odometry);
        this->scan_callback_internal(scan);
        PROFILING_NOTIFY_ALWAYS(odometry);
    }  //
    while (this->state.threads_running.load());

    RCLCPP_INFO(this->get_logger(), "[CORE]: Odometry thread exited cleanly");
}



#if LFD_ENABLED
void PerceptionNode::fiducial_worker()
{
    RCLCPP_INFO(
        this->get_logger(),
        "[CORE]: Lidar fiducial detection thread started successfully.");

    do
    {
        auto& buff = this->mt.fiducial_resources.waitNewestResource();
        if (!this->state.threads_running.load())
        {
            return;
        }

        PROFILING_SYNC();
        PROFILING_NOTIFY_ALWAYS(lidar_fiducial);
        this->fiducial_callback_internal(buff);
        PROFILING_NOTIFY_ALWAYS(lidar_fiducial);
    }  //
    while (this->state.threads_running.load());

    RCLCPP_INFO(
        this->get_logger(),
        "[CORE]: Lidar fiducial detection thread exited cleanly.");
}
#endif



#if MAPPING_ENABLED
void PerceptionNode::mapping_worker()
{
    RCLCPP_INFO(
        this->get_logger(),
        "[CORE]: Mapping thread started successfully.");

    do
    {
        auto& buff = this->mt.mapping_resources.waitNewestResource();
        if (!this->state.threads_running.load())
        {
            return;
        }

        PROFILING_SYNC();
        PROFILING_NOTIFY_ALWAYS(mapping);
        this->mapping_callback_internal(buff);
        PROFILING_NOTIFY_ALWAYS(mapping);
    }  //
    while (this->state.threads_running.load());

    RCLCPP_INFO(this->get_logger(), "[CORE]: Mapping thread exited cleanly.");
}
#endif



#if TRAVERSABILITY_ENABLED
void PerceptionNode::traversability_worker()
{
    RCLCPP_INFO(
        this->get_logger(),
        "[CORE]: Traversibility thread started successfully.");

    do
    {
        auto& buff = this->mt.traversibility_resources.waitNewestResource();
        if (!this->state.threads_running.load())
        {
            return;
        }

        PROFILING_SYNC();
        PROFILING_NOTIFY_ALWAYS(traversibility);
        this->traversibility_callback_internal(buff);
        PROFILING_NOTIFY_ALWAYS(traversibility);
    }  //
    while (this->state.threads_running.load());

    RCLCPP_INFO(
        this->get_logger(),
        "[CORE]: Traversibility thread exited cleanly.");
}
#endif



#if PATH_PLANNING_ENABLED
void PerceptionNode::path_planning_worker()
{
    RCLCPP_INFO(
        this->get_logger(),
        "[CORE]: Path planning thread started successfully.");

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

            PROFILING_SYNC();
            PROFILING_NOTIFY_ALWAYS(path_planning);
            this->path_planning_callback_internal(buff);
            PROFILING_NOTIFY_ALWAYS(path_planning);
        }
        else
        {
            this->mt.pplan_target_notifier.waitNewestResource();
        }
    }  //
    while (this->state.threads_running.load());

    RCLCPP_INFO(
        this->get_logger(),
        "[CORE]: Path planning thread exited cleanly.");
}
#endif

};  // namespace perception
};  // namespace csm
