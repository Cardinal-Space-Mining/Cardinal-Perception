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

#include <point_def.hpp>    // needs to come before PCL includes when using custom types

#include <vector>
#include <memory>
#include <mutex>
#include <atomic>
#include <unordered_set>
#include <optional>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <nano_gicp/nano_gicp.hpp>

#include <util.hpp>
#include <pub_map.hpp>
#include <geometry.hpp>
#include <tsq.hpp>


namespace csm
{
namespace perception
{

/* Manages IMU sensor samples, providing convenience functions for looking up the delta
 * rotation between timestamps, applying gyro/acceleration biases, and computing the gravity vector. */
class ImuIntegrator
{
public:
    inline ImuIntegrator(
        bool use_orientation = true,
        double calib_time = 1.
    ) :
        use_orientation{ use_orientation },
        calib_time{ calib_time }
    {}
    ~ImuIntegrator() = default;

public:
    void addSample(const sensor_msgs::msg::Imu& imu);
    void trimSamples(double trim_ts);
    bool recalibrate(double dt, bool force = false);

    Eigen::Vector3d estimateGravity(double dt) const;
    Eigen::Quaterniond getDelta(double start, double end) const;
    bool getNormalizedOffsets(util::tsq::TSQ<Eigen::Quaterniond>& dest, double t1, double t2) const;

    inline bool isCalibrated() const { return this->is_calibrated; }
    inline bool usingOrientation() const { return this->use_orientation; }
    inline const Eigen::Vector3d& gyroBias() const { return this->calib_bias.ang_vel; }
    inline const Eigen::Vector3d& accelBias() const { return this->calib_bias.lin_accel; }

protected:
    void recalibrateRange(size_t begin, size_t end);

    struct ImuMeas
    {
        Eigen::Vector3d ang_vel{ Eigen::Vector3d::Zero() };
        Eigen::Vector3d lin_accel{ Eigen::Vector3d::Zero() };
    };

protected:
    util::tsq::TSQ<Eigen::Quaterniond> orient_buffer;
    util::tsq::TSQ<ImuMeas> raw_buffer;
    ImuMeas calib_bias;

    mutable std::mutex mtx;
    std::atomic<bool> is_calibrated{ false };
    std::atomic<bool> use_orientation;

    const double calib_time;

};

/** Provides odometry via scan-to-scan and scan-to-map registration with optional IMU initialization.
  * The core algorithm is formally known as Direct Lidar Odometry (DLO) but has been heavily modified. */
class LidarOdometry
{
    friend class PerceptionNode;
    using PointType = csm::perception::OdomPointType;
    using PointCloudType = pcl::PointCloud<PointType>;
    using ClockType = std::chrono::system_clock;

    static_assert(std::is_same<PointType, pcl::PointXYZ>::value);

public:
    LidarOdometry(rclcpp::Node&);
    ~LidarOdometry() = default;
    DECLARE_IMMOVABLE(LidarOdometry)

public:
    struct IterationStatus
    {
        union
        {
            struct
            {
                union
                {
                    struct
                    {
                        bool odom_updated : 1;
                        bool keyframe_init : 1;
                        bool new_keyframe : 1;
                    };
                    uint32_t status_bits;
                };
                uint32_t total_keyframes;
            };
            int64_t data;
        };

        inline IterationStatus(int64_t v = 0) : data{ v } {}
        inline operator int64_t() const { return this->data; }
        inline operator bool() const { return static_cast<bool>(this->data); }
    };

public:
    /* Set the initial pose */
    bool setInitial(const util::geom::Pose3f& pose);

    /* Update the odometry using a new scan (and accumulated imu data).
     * Assumes necessary pre-filtering has already occured on the input cloud. */
    IterationStatus processScan(
        const PointCloudType& scan,
        double stamp,
        util::geom::PoseTf3f& odom_tf,
        const std::optional<Eigen::Matrix4f>& align_estimate = std::nullopt );

    void publishDebugScans(
        IterationStatus proc_status,
        const std::string& odom_frame_id );

protected:
    void getParams();
    void initState();

    bool preprocessPoints(const PointCloudType& scan);
    void setAdaptiveParams(const PointCloudType& scan);
    void initializeInputTarget();
    void setInputSources();

    void getNextPose(
        const std::optional<Eigen::Matrix4f>& align_estimate = std::nullopt );

    void propagateS2S(const Eigen::Matrix4f& T);
    void propagateS2M();
    void getSubmapKeyframes();
    void computeConvexHull();
    void computeConcaveHull();
    void pushSubmapIndices(
        const std::vector<float>& dists,
        int k,
        const std::vector<int>& frames );

    void transformCurrentScan();
    void updateKeyframes();

protected:
    rclcpp::Node& node;

    PointCloudType::Ptr current_scan, current_scan_t, target_cloud;
    PointCloudType scratch_cloud;

    pcl::VoxelGrid<PointType> vf_scan, vf_submap;
    pcl::ConvexHull<PointType> convex_hull;
    pcl::ConcaveHull<PointType> concave_hull;
    std::vector<int> keyframe_convex, keyframe_concave;

    PointCloudType::Ptr keyframe_cloud, keyframe_points;
    std::vector<std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>, PointCloudType::Ptr>> keyframes;  // TODO: use kdtree for positions
    std::vector<std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>> keyframe_normals;

    PointCloudType::Ptr submap_cloud;
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> submap_normals;
    std::unordered_set<int> submap_kf_idx_curr, submap_kf_idx_prev;

    nano_gicp::NanoGICP<PointType, PointType> gicp_s2s, gicp;

    // std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> trajectory;

    util::FloatPublisherMap metrics_pub;
    util::PublisherMap<sensor_msgs::msg::PointCloud2> debug_scan_pub;

    struct
    {
        // std::atomic<bool> dlo_initialized{ false };
        // std::atomic<bool> imu_calibrated{ false };
        // std::atomic<bool> is_grav_aligned{ false };
        std::atomic<bool> submap_hasChanged{ false };

        uint32_t num_keyframes{ 0 };

        double range_avg_lpf{ -1. };
        double range_stddev_lpf{ -1. };
        double adaptive_voxel_size{ 0. };

        double curr_frame_stamp{ 0. };
        double prev_frame_stamp{ 0. };
        double rolling_scan_delta_t{ 0. };

        Eigen::Vector3f translation{ Eigen::Vector3f::Zero() };
        Eigen::Quaternionf
            rotq{ Eigen::Quaternionf::Identity() },
            last_rotq{ Eigen::Quaternionf::Identity() };

        Eigen::Matrix4f
            T{ Eigen::Matrix4f::Identity() },
            T_s2s{ Eigen::Matrix4f::Identity() },
            T_s2s_prev{ Eigen::Matrix4f::Identity() };

        std::mutex mtx;
    }
    state;

    struct
    {
        bool use_scan_ts_as_init_;

        // bool gravity_align_;

        double keyframe_thresh_dist_;
        double keyframe_thresh_rot_;

        int submap_knn_;
        int submap_kcv_;
        int submap_kcc_;
        double submap_concave_alpha_;

        // bool initial_pose_use_;
        // Eigen::Vector3d initial_position_;
        // Eigen::Quaterniond initial_orientation_;

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

        // bool imu_use_;
        // bool imu_use_orientation_;
        // int imu_calib_time_;

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
    }
    param;

};

};
};
