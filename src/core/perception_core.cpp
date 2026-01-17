/*******************************************************************************
*   Copyright (C) 2024-2026 Cardinal Space Mining Club                         *
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
#include <utility>
#include <iostream>

#include <csm_metrics/profiling.hpp>

#include <util/geometry.hpp>
#include <util/ros_utils.hpp>


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
    "  Copyright (C) 2024-2026 Cardinal Space Mining Club\n";

struct PerceptionConfig
{
public:
    explicit PerceptionConfig(PerceptionNode& node) : node{node} {}

public:
    PerceptionNode& node;

    // core
    std::string map_frame;
    std::string odom_frame;
    std::string base_frame;
    std::string scan_topic;
    std::string imu_topic;
    float metrics_pub_freq;

    // exclusion zones
    int num_excl_zones;
    std::vector<
        std::tuple<std::string, std::vector<double>, std::vector<double>>>
        excl_zones;

    // trajectory filter
    double trjf_sample_window_s;
    double trjf_filter_window_s;
    double trjf_avg_linear_err_thresh;
    double trjf_avg_angular_err_thresh;
    double trjf_max_linear_dev_thresh;
    double trjf_max_angular_dev_thresh;

    // lidar fiducial detector
    double lfd_detection_radius;
    double lfd_plane_seg_thickness;
    double lfd_ground_seg_thickness;
    double lfd_up_vec_max_angular_dev;
    double lfd_planes_max_angular_dev;
    double lfd_vox_res;
    double lfd_max_proportion_leftover;
    int lfd_min_num_input_points;
    int lfd_min_plane_seg_points;

    // mapping
    double kfc_frustum_search_radius;
    double kfc_radial_dist_thresh;
    double kfc_surface_width;
    double kfc_delete_max_range;
    double kfc_add_max_range;
    double kfc_voxel_size;

    float map_crop_horizontal_range;
    float map_crop_vertical_range;
    float map_export_horizontal_range;
    float map_export_vertical_range;

    // traversibility
    float trav_norm_estimation_radius;
    float trav_output_res;
    float trav_grad_search_radius;
    float trav_min_grad_diff;
    float trav_avoid_grad_angle;
    float trav_avoid_radius;
    float trav_score_curvature_weight;
    float trav_score_grad_weight;
    int trav_min_vox_cell_points;
    int trav_interp_point_samples;

    // path planning
    float pplan_boundary_radius;
    float pplan_goal_thresh;
    float pplan_search_radius;
    float pplan_dist_coeff;
    float pplan_dir_coeff;
    float pplan_trav_coeff;
    float pplan_verification_range;
    float pplan_map_obstacle_merge_window;
    float pplan_map_passive_crop_horizontal_range;
    float pplan_map_passive_crop_vertical_range;
    int pplan_verification_degree;
    int pplan_max_neighbors;

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

    friend std::ostream& operator<<(
        std::ostream& os,
        const PerceptionConfig& config);
};

std::ostream& operator<<(std::ostream& os, const PerceptionConfig& config)
{
    // alignment, feat. ChatGPT lol
    constexpr int CONFIG_ALIGN_WIDTH = 25;
    auto align = [](const char* label)
    {
        std::ostringstream oss;
        oss << " |  " << std::left << std::setw(CONFIG_ALIGN_WIDTH) << label
            << ": ";
        return oss.str();
    };



    os << std::setprecision(3);
    os << "\n"
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
       << PerceptionConfig::getEnableDisableStr(PERCEPTION_ENABLE_PATH_PLANNING)
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
       << align("Map Frame ID") << config.map_frame << "\n"
       << align("Odom Frame ID") << config.odom_frame << "\n"
       << align("Robot Frame ID") << config.base_frame << "\n"
       << align("Statistics Frequency") << config.metrics_pub_freq
       << " hz\n"
          " |\n"
          " +- EXCLUSION ZONES\n";
    if (config.num_excl_zones)
    {
        for (const auto& zone : config.excl_zones)
        {
            os << " |  +- Frame \"" << std::get<0>(zone) << "\"\n"
               << align("|  Min") << "[" << std::get<1>(zone)[0] << ", "
               << std::get<1>(zone)[1] << ", " << std::get<1>(zone)[2]
               << "] (m)\n"
               << align("|  Max") << "[" << std::get<2>(zone)[0] << ", "
               << std::get<2>(zone)[1] << ", " << std::get<2>(zone)[2]
               << "] (m)\n";
        }
    }
    else
    {
        os << " |  (none)\n";
    }
    os << " |\n"
       << " +- TRAJECTORY FILTER\n"
       << align("Sample Window") << config.trjf_sample_window_s << " seconds\n"
       << align("Filter Window") << config.trjf_filter_window_s << " seconds\n"
       << align("Linear Error Thresh") << config.trjf_avg_linear_err_thresh
       << " meters\n"
       << align("Angular Error Thresh") << config.trjf_avg_angular_err_thresh
       << " radians\n"
       << align("Linear Dev Thresh") << config.trjf_max_linear_dev_thresh
       << "\n"
       << align("Angular Dev Thresh") << config.trjf_max_angular_dev_thresh
       << "\n"
       << " |\n"
          " +- ODOMETRY\n"
          " |  (not implemented)\n";

#if LFD_ENABLED
    os << " |\n"
          " +- LIDAR FIDUCIAL DETECTOR\n"
       << align("Detection Range") << config.lfd_detection_radius << " meters\n"
       << align("Plane Seg Thickness") << config.lfd_plane_seg_thickness
       << " meters\n"
       << align("Ground Seg Thickness") << config.lfd_ground_seg_thickness
       << " meters\n"
       << align("Up Vec Max Angular Dev") << config.lfd_up_vec_max_angular_dev
       << " radians\n"
       << align("Planes Max Angular Dev") << config.lfd_planes_max_angular_dev
       << " radians\n"
       << align("Voxel Resolution") << config.lfd_vox_res << " meters\n"
       << align("Max Percentage Leftover")
       << (config.lfd_max_proportion_leftover * 100) << "%\n"
       << align("Min Num Input Points") << config.lfd_min_num_input_points
       << "\n"
       << align("Min Num Seg Points") << config.lfd_min_plane_seg_points
       << "\n";
#endif

#if MAPPING_ENABLED
    os << " |\n"
          " +- MAPPING\n"
       << align("Horizontal Crop Range") << config.map_crop_horizontal_range
       << " meters\n"
       << align("Vertical Crop Range") << config.map_crop_vertical_range
       << " meters\n"
       << align("Frustum Radius") << config.kfc_frustum_search_radius
       << " radians\n"
       << align("Radial Dist Thresh") << config.kfc_radial_dist_thresh
       << " meters\n"
       << align("Surface Width") << config.kfc_surface_width << " meters\n"
       << align("Delete Max Range") << config.kfc_delete_max_range
       << " meters\n"
       << align("Add Max Range") << config.kfc_add_max_range << " meters\n"
       << align("Voxel Size") << config.kfc_voxel_size << " meters\n";
#endif

#if TRAVERSIBILITY_ENABLED
    os << " |\n"
          " +- TRAVERSIBILITY\n"
       << align("Horizontal Export Range") << config.map_export_horizontal_range
       << " meters\n"
       << align("Vertical Export Range") << config.map_export_vertical_range
       << " meters\n"
       << align("Normal Est Radius") << config.trav_norm_estimation_radius
       << " meters\n"
       << align("Output Grid Res") << config.trav_output_res << " meters\n"
       << align("Grad Search Radius") << config.trav_grad_search_radius
       << " meters\n"
       << align("Min Grad Diff") << config.trav_min_grad_diff << " meters\n"
       << align("Avoid Grad Angle") << config.trav_avoid_grad_angle
       << " degrees\n"
       << align("Avoid Radius") << config.trav_avoid_radius << " meters\n"
       << align("Curvature Trav Weight") << config.trav_score_curvature_weight
       << "\n"
       << align("Gradient Trav Weight") << config.trav_score_grad_weight << "\n"
       << align("Vox Cell Points Thresh") << config.trav_min_vox_cell_points
       << "\n"
       << align("Point Samples") << config.trav_interp_point_samples << "\n";
#endif

#if PATH_PLANNING_ENABLED
    os << " |\n"
          " +- PATH PLANNING\n"
       << align("Boundary Radius") << config.pplan_boundary_radius
       << " meters\n"
       << align("Goal Threshold") << config.pplan_goal_thresh << " meters\n"
       << align("Search Radius") << config.pplan_search_radius << " meters\n"
       << align("Distance Coeff") << config.pplan_dist_coeff << "\n"
       << align("Straightness Coeff") << config.pplan_dir_coeff << "\n"
       << align("Traversibility Coeff") << config.pplan_trav_coeff << "\n"
       << align("Verification Range") << config.pplan_verification_range << " meters\n"
       << align("Verification Degree") << config.pplan_verification_degree << " points\n"
       << align("Max Num Neighbors") << config.pplan_max_neighbors << " points\n"
       << align("Map Merge Window") << config.pplan_map_obstacle_merge_window
       << " meters\n"
       << align("Map Hrz. Crop Range")
       << config.pplan_map_passive_crop_horizontal_range << " meters\n"
       << align("Map Vrt. Crop Range")
       << config.pplan_map_passive_crop_vertical_range << " meters\n";
#endif

    os << " +\n";
    return os;
}

PerceptionNode::PerceptionNode() :
    Node("cardinal_perception"),
    tf_buffer{std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)},
    tf_listener{tf_buffer, this},
    imu_worker{*this, tf_buffer},
    localization_worker{*this, tf_buffer, imu_worker.getSampler()},
    mapping_worker{*this},
    traversibility_worker{*this, imu_worker.getSampler()},
    path_planning_worker{*this, tf_buffer},
    mining_eval_worker{*this, tf_buffer},
    generic_pub{*this, PERCEPTION_TOPIC(""), PERCEPTION_PUBSUB_QOS}
{
    PerceptionConfig config{*this};

    this->getParams(config);
    this->initPubSubs(config);
    this->printStartup(config);

    this->localization_worker.connectOutput(this->mapping_worker.getInput());
    this->mapping_worker.connectOutput(this->traversibility_worker.getInput());
    this->traversibility_worker.connectOutput(
        this->path_planning_worker.getInput());
    this->traversibility_worker.connectOutput(
        this->mining_eval_worker.getInput());

    this->localization_worker.startThreads();
    this->mapping_worker.startThreads();
    this->traversibility_worker.startThreads();
    this->path_planning_worker.startThreads();
    this->mining_eval_worker.startThreads();
}

PerceptionNode::~PerceptionNode() { this->shutdown(); }


void PerceptionNode::shutdown()
{
    this->localization_worker.stopThreads();
    this->mapping_worker.stopThreads();
    this->traversibility_worker.stopThreads();
    this->path_planning_worker.stopThreads();
    this->mining_eval_worker.stopThreads();
}



void PerceptionNode::getParams(PerceptionConfig& config)
{
    util::declare_param(this, "map_frame_id", config.map_frame, "map");
    util::declare_param(this, "odom_frame_id", config.odom_frame, "odom");
    util::declare_param(this, "base_frame_id", config.base_frame, "base_link");
    util::declare_param(this, "metrics_pub_freq", config.metrics_pub_freq, 10.);

    // --- CROP BOUNDS ---------------------------------------------------------
    util::declare_param(
        this,
        "exclusion_zones.num_zones",
        config.num_excl_zones,
        0);
    config.excl_zones.reserve(config.num_excl_zones);
    for (int i = 0; i < config.num_excl_zones; i++)
    {
        auto& zone_config = config.excl_zones.emplace_back();
        auto& frame_config = std::get<0>(zone_config);
        auto& min_config = std::get<1>(zone_config);
        auto& max_config = std::get<2>(zone_config);

        std::stringstream zone_tag;
        zone_tag << "exclusion_zones.zone" << i;
        util::declare_param(
            this,
            zone_tag.str() + ".frame_id",
            frame_config,
            "");
        util::declare_param(
            this,
            zone_tag.str() + ".min",
            min_config,
            {0., 0., 0.});
        util::declare_param(
            this,
            zone_tag.str() + ".max",
            max_config,
            {0., 0., 0.});

        if (min_config.size() >= 3 && max_config.size() >= 3)
        {
            this->localization_worker.scan_preproc.addExclusionZone(
                frame_config,
                Eigen::AlignedBox3f(
                    Eigen::Vector3f(
                        static_cast<double>(min_config[0]),
                        static_cast<double>(min_config[1]),
                        static_cast<double>(min_config[2])),
                    Eigen::Vector3f(
                        static_cast<double>(max_config[0]),
                        static_cast<double>(max_config[1]),
                        static_cast<double>(max_config[2]))));
        }
        else
        {
            config.excl_zones.pop_back();
        }
    }

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

    this->localization_worker.transform_sync.trajectoryFilter().applyParams(
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
        config.lfd_detection_radius,
        2.);
    util::declare_param(
        this,
        "fiducial_detection.plane_seg_thickness",
        config.lfd_plane_seg_thickness,
        0.01);
    util::declare_param(
        this,
        "fiducial_detection.ground_seg_thickness",
        config.lfd_ground_seg_thickness,
        0.02);
    util::declare_param(
        this,
        "fiducial_detection.up_vec_max_angular_dev",
        config.lfd_up_vec_max_angular_dev,
        0.2);
    util::declare_param(
        this,
        "fiducial_detection.planes_max_angular_dev",
        config.lfd_planes_max_angular_dev,
        0.1);
    util::declare_param(
        this,
        "fiducial_detection.vox_resolution",
        config.lfd_vox_res,
        0.03);
    util::declare_param(
        this,
        "fiducial_detection.max_proportion_leftover",
        config.lfd_max_proportion_leftover,
        0.05);
    util::declare_param(
        this,
        "fiducial_detection.min_num_input_points",
        config.lfd_min_num_input_points,
        100);
    util::declare_param(
        this,
        "fiducial_detection.min_plane_seg_points",
        config.lfd_min_plane_seg_points,
        15);

    this->localization_worker.fiducial_detector.configDetector(
        LFD_ESTIMATE_GROUND_PLANE | LFD_PREFER_USE_GROUND_SAMPLE);
    this->localization_worker.fiducial_detector.applyParams(
        config.lfd_detection_radius,
        config.lfd_plane_seg_thickness,
        config.lfd_ground_seg_thickness,
        config.lfd_up_vec_max_angular_dev,
        config.lfd_planes_max_angular_dev,
        config.lfd_vox_res,
        static_cast<size_t>(config.lfd_min_num_input_points),
        static_cast<size_t>(config.lfd_min_plane_seg_points),
        config.lfd_max_proportion_leftover);
#endif

    // --- MAPPING -------------------------------------------------------------
#if MAPPING_ENABLED
    util::declare_param(
        this,
        "mapping.crop_horizontal_range",
        config.map_crop_horizontal_range,
        0.);
    util::declare_param(
        this,
        "mapping.crop_vertical_range",
        config.map_crop_vertical_range,
        0.);
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
        "mapping.surface_width",
        config.kfc_surface_width,
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
    util::declare_param(
        this,
        "mapping.voxel_size",
        config.kfc_voxel_size,
        0.05);

    this->mapping_worker.sparse_map.applyParams(
        config.kfc_frustum_search_radius,
        config.kfc_radial_dist_thresh,
        config.kfc_surface_width,
        config.kfc_delete_max_range,
        config.kfc_add_max_range,
        config.kfc_voxel_size);
#endif

    // --- TRAVERSIBILITY ------------------------------------------------------
#if TRAVERSIBILITY_ENABLED
    util::declare_param(
        this,
        "traversibility.chunk_horizontal_range",
        config.map_export_horizontal_range,
        4.);
    util::declare_param(
        this,
        "traversibility.chunk_vertical_range",
        config.map_export_vertical_range,
        1.);
    util::declare_param(
        this,
        "traversibility.normal_estimation_radius",
        config.trav_norm_estimation_radius,
        -1.f);
    util::declare_param(
        this,
        "traversibility.output_res",
        config.trav_output_res,
        0.1);
    util::declare_param(
        this,
        "traversibility.grad_search_radius",
        config.trav_grad_search_radius,
        0.25);
    util::declare_param(
        this,
        "traversibility.min_grad_diff",
        config.trav_min_grad_diff,
        0.15);
    util::declare_param(
        this,
        "traversibility.non_trav_grad_angle",
        config.trav_avoid_grad_angle,
        45.f);
    util::declare_param(
        this,
        "traversibility.avoidance_radius",
        config.trav_avoid_radius,
        0.5f);
    util::declare_param(
        this,
        "traversibility.trav_score_curvature_weight",
        config.trav_score_curvature_weight,
        5.f);
    util::declare_param(
        this,
        "traversibility.trav_score_grad_weight",
        config.trav_score_grad_weight,
        1.f);
    util::declare_param(
        this,
        "traversibility.min_vox_cell_points",
        config.trav_min_vox_cell_points,
        3);
    util::declare_param(
        this,
        "traversibility.interp_point_samples",
        config.trav_interp_point_samples,
        7);

    if (config.trav_norm_estimation_radius <= 0.f)
    {
        config.trav_norm_estimation_radius = config.kfc_voxel_size * 2;
    }
    if (config.trav_output_res <= 0.f)
    {
        config.trav_output_res = config.kfc_voxel_size;
    }

    this->traversibility_worker.trav_gen.configure(
        config.trav_norm_estimation_radius,
        config.trav_output_res,
        config.trav_grad_search_radius,
        config.trav_min_grad_diff,
        config.trav_avoid_grad_angle,
        config.trav_avoid_radius,
        config.trav_score_curvature_weight,
        config.trav_score_grad_weight,
        config.trav_min_vox_cell_points,
        config.trav_interp_point_samples);
#endif

    // --- PATH PLANNING -------------------------------------------------------
#if PATH_PLANNING_ENABLED
    util::declare_param(
        this,
        "pplan.boundary_radius",
        config.pplan_boundary_radius,
        0.15f);
    util::declare_param(
        this,
        "pplan.goal_threshold",
        config.pplan_goal_thresh,
        0.1f);
    util::declare_param(
        this,
        "pplan.search_radius",
        config.pplan_search_radius,
        1.f);
    util::declare_param(
        this,
        "pplan.distance_coeff",
        config.pplan_dist_coeff,
        1.f);
    util::declare_param(
        this,
        "pplan.straightness_coeff",
        config.pplan_dir_coeff,
        1.f);
    util::declare_param(
        this,
        "pplan.traversibility_coeff",
        config.pplan_trav_coeff,
        1.f);
    util::declare_param(
        this,
        "pplan.verification_range",
        config.pplan_verification_range,
        1.5f);
    util::declare_param(
        this,
        "pplan.verification_degree",
        config.pplan_verification_degree,
        2);
    util::declare_param(
        this,
        "pplan.max_neighbors",
        config.pplan_max_neighbors,
        10);
    util::declare_param(
        this,
        "pplan.map_obstacle_merge_window",
        config.pplan_map_obstacle_merge_window,
        0.5f);
    util::declare_param(
        this,
        "pplan.map_passive_crop_horizontal_range",
        config.pplan_map_passive_crop_horizontal_range,
        10.f);
    util::declare_param(
        this,
        "pplan.map_passive_crop_vertical_range",
        config.pplan_map_passive_crop_vertical_range,
        5.f);

    this->path_planning_worker.path_planner.setParameters(
        config.pplan_boundary_radius,
        config.pplan_goal_thresh,
        config.pplan_search_radius,
        config.pplan_dist_coeff,
        config.pplan_dir_coeff,
        config.pplan_trav_coeff,
        config.pplan_verification_range,
        config.pplan_verification_degree,
        config.pplan_max_neighbors);
#endif

    this->imu_worker.configure(config.base_frame);
    this->localization_worker.configure(
        config.map_frame,
        config.odom_frame,
        config.base_frame);
    this->mapping_worker.configure(
        config.odom_frame,
        config.map_crop_horizontal_range,
        config.map_crop_vertical_range,
        config.map_export_horizontal_range,
        config.map_export_vertical_range);
    this->traversibility_worker.configure(config.odom_frame);
    this->path_planning_worker.configure(
        config.odom_frame,
        config.pplan_map_obstacle_merge_window,
        config.pplan_map_passive_crop_horizontal_range,
        config.pplan_map_passive_crop_vertical_range);
    this->mining_eval_worker.configure(config.odom_frame);
}



void PerceptionNode::initPubSubs(PerceptionConfig& config)
{
    util::declare_param(this, "scan_topic", config.scan_topic, "scan");
    util::declare_param(this, "imu_topic", config.imu_topic, "imu");

    this->imu_sub = this->create_subscription<ImuMsg>(
        config.imu_topic,
        PERCEPTION_PUBSUB_QOS,
        [this](ImuMsg::SharedPtr msg) { this->imu_worker.accept(*msg); });
    this->scan_sub = this->create_subscription<PointCloudMsg>(
        config.scan_topic,
        PERCEPTION_PUBSUB_QOS,
        [this](const PointCloudMsg::ConstSharedPtr& msg)
        { this->localization_worker.accept(msg); });
#if TAG_DETECTION_ENABLED
    this->detections_sub = this->create_subscription<TagsTransformMsg>(
        PERCEPTION_TOPIC("tags_detections"),
        PERCEPTION_PUBSUB_QOS,
        [this](const TagsTransformMsg::ConstSharedPtr& msg)
        { this->localization_worker.accept(msg); });
#endif

    this->alignment_state_service = this->create_service<SetBoolSrv>(
        PERCEPTION_TOPIC("set_global_alignment"),
        [this](
            SetBoolSrv::Request::SharedPtr req,
            SetBoolSrv::Response::SharedPtr resp)
        {
            resp->success =
                this->localization_worker.setGlobalAlignmentEnabled(req->data);
        });
#if PATH_PLANNING_ENABLED
    this->path_plan_service = this->create_service<UpdatePathPlanSrv>(
        PERCEPTION_TOPIC("update_path_planning"),
        [this](
            UpdatePathPlanSrv::Request::SharedPtr req,
            UpdatePathPlanSrv::Response::SharedPtr resp)
        { this->path_planning_worker.accept(req, resp); });
#endif
    this->mining_eval_service = this->create_service<UpdateMiningEvalSrv>(
        PERCEPTION_TOPIC("query_mining_eval"),
        [this](
            UpdateMiningEvalSrv::Request::SharedPtr req,
            UpdateMiningEvalSrv::Response::SharedPtr resp)
        { this->mining_eval_worker.accept(req, resp); });

    this->proc_stats_timer = this->create_wall_timer(
        std::chrono::milliseconds(
            static_cast<int>(1000.f / config.metrics_pub_freq)),
        [this]()
        {
            this->process_stats.update();
            this->generic_pub.publish(
                "process_stats",
                this->process_stats.toMsg());
        });
}



void PerceptionNode::printStartup(PerceptionConfig& config)
{
    std::ostringstream msg;
    msg << "\n" << STARTUP_SPLASH;

#if PERCEPTION_PRINT_STARTUP_CONFIGS
    msg << config;
#else
    (void)config;
#endif

    RCLCPP_INFO(
        this->get_logger(),
        "[CORE]: Initialization complete.%s",
        msg.str().c_str());
}

};  // namespace perception
};  // namespace csm
