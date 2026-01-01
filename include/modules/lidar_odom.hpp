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

#pragma once

#include <mutex>
#include <atomic>
#include <memory>
#include <vector>
#include <optional>
#include <unordered_set>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>

#include <rclcpp/rclcpp.hpp>

#include <nano_gicp/nano_gicp.hpp>

#include <util/pub_map.hpp>
#include <util/geometry.hpp>
#include <util/std_utils.hpp>


namespace csm
{
namespace perception
{

/** Provides odometry via scan-to-scan and scan-to-map registration with
 * optional IMU initialization. The core algorithm is formally known as
 * Direct Lidar Odometry - aka 'DLO' (see lisence) but has been
 * heavily modified. */
template<typename Point_T = pcl::PointXYZ>
class LidarOdometry
{
    using PointT = Point_T;
    using PointCloudT = pcl::PointCloud<PointT>;
    using PointCloudPtrT = typename PointCloudT::Ptr;
    using PointCloudConstPtrT = typename PointCloudT::ConstPtr;
    using ClockT = std::chrono::system_clock;

    using Vec3f = Eigen::Vector3f;
    using Mat4f = Eigen::Matrix4f;
    using Mat4d = Eigen::Matrix4d;
    using Quatf = Eigen::Quaternionf;

    static_assert(pcl::traits::has_xyz<PointT>::value);

public:
    struct IterationStatus
    {
        bool odom_updated  : 1;
        bool keyframe_init : 1;
        bool new_keyframe  : 1;

        uint32_t total_keyframes;

        IterationStatus() :
            odom_updated(false),
            keyframe_init(false),
            new_keyframe(false),
            total_keyframes(0)
        {
        }

        operator bool() const
        {
            return (
                odom_updated | keyframe_init | new_keyframe |
                (total_keyframes > 0));
        }
    };

    static_assert(sizeof(IterationStatus) == sizeof(std::uint64_t));

public:
    LidarOdometry(rclcpp::Node&);
    ~LidarOdometry() = default;
    DECLARE_IMMOVABLE(LidarOdometry)

public:
    /* Set the initial pose */
    bool setInitial(const util::geom::Pose3f& pose);

    /* Update the odometry using a new scan (and accumulated imu data).
     * Assumes necessary pre-filtering has already occured on the input cloud. */
    IterationStatus processScan(
        const PointCloudT& scan,
        double stamp,
        util::geom::PoseTf3f& odom_tf,
        const std::optional<Mat4f>& align_estimate = std::nullopt);

    void publishDebugScans(
        util::GenericPubMap& pub,
        IterationStatus proc_status,
        const std::string& odom_frame_id);
    void publishDebugMetrics(util::GenericPubMap& pub);

    inline double currStamp() const { return this->state.curr_frame_stamp; }
    inline double prevStamp() const { return this->state.prev_frame_stamp; }

protected:
    void initState();

    bool preprocessPoints(const PointCloudT& scan);
    void setAdaptiveParams(const PointCloudT& scan);
    void initializeInputTarget();
    void setInputSources();

    void getNextPose(const std::optional<Mat4f>& align_estimate = std::nullopt);

    void propagateS2S(const Mat4f& T);
    void propagateS2M();
    void getSubmapKeyframes();
    void computeConvexHull();
    void computeConcaveHull();
    void pushSubmapIndices(
        const std::vector<float>& dists,
        int k,
        const std::vector<int>& frames);

    void transformCurrentScan();
    void updateKeyframes();

protected:
    PointCloudPtrT current_scan, current_scan_t, target_cloud;
    PointCloudT scratch_cloud;

    pcl::VoxelGrid<PointT> vf_scan, vf_submap;
    pcl::ConvexHull<PointT> convex_hull;
    pcl::ConcaveHull<PointT> concave_hull;
    std::vector<int> keyframe_convex, keyframe_concave;

    PointCloudPtrT keyframe_cloud, keyframe_points;
    // TODO: use kdtree for positions
    std::vector<std::pair<std::pair<Vec3f, Quatf>, PointCloudPtrT>> keyframes;
    std::vector<std::vector<Mat4d, Eigen::aligned_allocator<Mat4d>>>
        keyframe_normals;

    PointCloudPtrT submap_cloud;
    std::vector<Mat4d, Eigen::aligned_allocator<Mat4d>> submap_normals;
    std::unordered_set<int> submap_kf_idx_curr, submap_kf_idx_prev;

    nano_gicp::NanoGICP<PointT, PointT> gicp_s2s, gicp;

    // std::vector<std::pair<Vec3d, Quatd>> trajectory;

protected:
    struct
    {
        std::atomic<bool> submap_hasChanged{false};

        uint32_t num_keyframes{0};

        double range_avg_lpf{-1.};
        double range_stddev_lpf{-1.};
        double adaptive_voxel_size{0.};

        double curr_frame_stamp{0.};
        double prev_frame_stamp{0.};
        double rolling_scan_delta_t{0.};
        double keyframe_thresh_dist_{0.};

        Vec3f translation{Vec3f::Zero()};
        Quatf rotq{Quatf::Identity()};
        Quatf last_rotq{Quatf::Identity()};

        Mat4f T{Mat4f::Identity()};
        Mat4f T_s2s{Mat4f::Identity()};
        Mat4f T_s2s_prev{Mat4f::Identity()};

        std::mutex mtx;
    } state;

protected:
    struct LidarOdometryParam
    {
    public:
        LidarOdometryParam(rclcpp::Node& node);

    public:
        bool use_scan_ts_as_init_;

        double keyframe_thresh_rot_;

        int submap_knn_;
        int submap_kcv_;
        int submap_kcc_;
        double submap_concave_alpha_;

        bool vf_scan_use_;
        double vf_scan_res_;

        bool vf_submap_use_;
        double vf_submap_res_;

        double adaptive_voxel_range_coeff_;
        double adaptive_voxel_stddev_coeff_;
        double adaptive_voxel_offset_;
        double adaptive_voxel_floor_;
        double adaptive_voxel_ceil_;
        double adaptive_voxel_precision_;

        bool immediate_filter_use_;
        double immediate_filter_range_;
        double immediate_filter_thresh_;

        bool adaptive_params_use_;
        double adaptive_params_lpf_coeff_;

        int gicp_num_threads_;
        int gicp_min_num_points_;

        int gicps2s_k_correspondences_;
        double gicps2s_max_corr_dist_;
        int gicps2s_max_iter_;
        double gicps2s_transformation_ep_;
        double gicps2s_euclidean_fitness_ep_;
        int gicps2s_ransac_iter_;
        double gicps2s_ransac_inlier_thresh_;

        int gicps2m_k_correspondences_;
        double gicps2m_max_corr_dist_;
        int gicps2m_max_iter_;
        double gicps2m_transformation_ep_;
        double gicps2m_euclidean_fitness_ep_;
        int gicps2m_ransac_iter_;
        double gicps2m_ransac_inlier_thresh_;
    };
    const LidarOdometryParam param;
};

};  // namespace perception
};  // namespace csm





#ifndef LIDAR_ODOM_PRECOMPILED

    #include "impl/lidar_odom_impl.hpp"

// clang-format off
#define LIDAR_ODOM_INSTANTIATE_CLASS_TEMPLATE(POINT_TYPE) \
    template class csm::perception::LidarOdometry<POINT_TYPE>;

#define LIDAR_ODOM_INSTANTIATE_NANOGICP_DEPENDENCIES(POINT_TYPE) \
    template class nanoflann::KdTreeFLANN<POINT_TYPE>; \
    template class nano_gicp::LsqRegistration<POINT_TYPE, POINT_TYPE>; \
    template class nano_gicp::NanoGICP<POINT_TYPE, POINT_TYPE>;
// clang-format on

#endif
