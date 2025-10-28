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

#include "../lidar_odom.hpp"

#include <set>
#include <queue>
#include <iostream>

#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <cloud_ops.hpp>


using namespace util::geom::cvt::ops;

using Float64Msg = std_msgs::msg::Float64;
using PointCloudMsg = sensor_msgs::msg::PointCloud2;


namespace csm
{
namespace perception
{

template<typename PT>
LidarOdometry<PT>::LidarOdometryParam::LidarOdometryParam(rclcpp::Node& node)
{
    // General
    util::declare_param(
        node,
        "dlo.use_timestamps_as_init",
        this->use_scan_ts_as_init_,
        true);

    // Keyframe Threshold
    util::declare_param(
        node,
        "dlo.keyframe.thresh_R",
        this->keyframe_thresh_rot_,
        1.0);

    // Submap
    util::declare_param(node, "dlo.keyframe.submap.knn", this->submap_knn_, 10);
    util::declare_param(node, "dlo.keyframe.submap.kcv", this->submap_kcv_, 10);
    util::declare_param(node, "dlo.keyframe.submap.kcc", this->submap_kcc_, 10);

    // Voxel Grid Filter
    util::declare_param(
        node,
        "dlo.voxel_filter.scan.use",
        this->vf_scan_use_,
        true);
    util::declare_param(
        node,
        "dlo.voxel_filter.scan.res",
        this->vf_scan_res_,
        0.05);
    util::declare_param(
        node,
        "dlo.voxel_filter.submap.use",
        this->vf_submap_use_,
        false);
    util::declare_param(
        node,
        "dlo.voxel_filter.submap.res",
        this->vf_submap_res_,
        0.1);
    util::declare_param(
        node,
        "dlo.voxel_filter.adaptive_leaf_size.range_coeff",
        this->adaptive_voxel_range_coeff_,
        0.01);
    util::declare_param(
        node,
        "dlo.voxel_filter.adaptive_leaf_size.stddev_coeff",
        this->adaptive_voxel_stddev_coeff_,
        0.005);
    util::declare_param(
        node,
        "dlo.voxel_filter.adaptive_leaf_size.offset",
        this->adaptive_voxel_offset_,
        0.);
    util::declare_param(
        node,
        "dlo.voxel_filter.adaptive_leaf_size.floor",
        this->adaptive_voxel_floor_,
        0.05);
    util::declare_param(
        node,
        "dlo.voxel_filter.adaptive_leaf_size.ceil",
        this->adaptive_voxel_ceil_,
        0.1);
    util::declare_param(
        node,
        "dlo.voxle_filter.adapative_leaf_size.precision",
        this->adaptive_voxel_precision_,
        0.01);

    // Immediate Filter
    util::declare_param(
        node,
        "dlo.immediate_filter.use",
        this->immediate_filter_use_,
        true);
    util::declare_param(
        node,
        "dlo.immediate_filter.range",
        this->immediate_filter_range_,
        0.5);
    util::declare_param(
        node,
        "dlo.immediate_filter.thresh_proportion",
        this->immediate_filter_thresh_,
        0.4);

    // Adaptive Parameters
    util::declare_param(
        node,
        "dlo.adaptive_params.use",
        this->adaptive_params_use_,
        false);
    util::declare_param(
        node,
        "dlo.adaptive_params.lpf_coeff",
        this->adaptive_params_lpf_coeff_,
        0.95);

    // GICP
    util::declare_param(
        node,
        "dlo.gicp.num_threads",
        this->gicp_num_threads_,
        4);
    util::declare_param(
        node,
        "dlo.gicp.min_num_points",
        this->gicp_min_num_points_,
        100);
    util::declare_param(
        node,
        "dlo.gicp.s2s.k_correspondences",
        this->gicps2s_k_correspondences_,
        20);
    util::declare_param(
        node,
        "dlo.gicp.s2s.max_correspondence_distance",
        this->gicps2s_max_corr_dist_,
        std::sqrt(std::numeric_limits<double>::max()));
    util::declare_param(
        node,
        "dlo.gicp.s2s.max_iterations",
        this->gicps2s_max_iter_,
        64);
    util::declare_param(
        node,
        "dlo.gicp.s2s.transformation_epsilon",
        this->gicps2s_transformation_ep_,
        0.0005);
    util::declare_param(
        node,
        "dlo.gicp.s2s.euclidean_fitness_epsilon",
        this->gicps2s_euclidean_fitness_ep_,
        -std::numeric_limits<double>::max());
    util::declare_param(
        node,
        "dlo.gicp.s2s.ransac.iterations",
        this->gicps2s_ransac_iter_,
        0);
    util::declare_param(
        node,
        "dlo.gicp.s2s.ransac.outlier_rejection_thresh",
        this->gicps2s_ransac_inlier_thresh_,
        0.05);
    util::declare_param(
        node,
        "dlo.gicp.s2m.k_correspondences",
        this->gicps2m_k_correspondences_,
        20);
    util::declare_param(
        node,
        "dlo.gicp.s2m.max_correspondence_distance",
        this->gicps2m_max_corr_dist_,
        std::sqrt(std::numeric_limits<double>::max()));
    util::declare_param(
        node,
        "dlo.gicp.s2m.max_iterations",
        this->gicps2m_max_iter_,
        64);
    util::declare_param(
        node,
        "dlo.gicp.s2m.transformation_epsilon",
        this->gicps2m_transformation_ep_,
        0.0005);
    util::declare_param(
        node,
        "dlo.gicp.s2m.euclidean_fitness_epsilon",
        this->gicps2m_euclidean_fitness_ep_,
        -std::numeric_limits<double>::max());
    util::declare_param(
        node,
        "dlo.gicp.s2m.ransac.iterations",
        this->gicps2m_ransac_iter_,
        0);
    util::declare_param(
        node,
        "dlo.gicp.s2m.ransac.outlier_rejection_thresh",
        this->gicps2m_ransac_inlier_thresh_,
        0.05);
}



template<typename PT>
LidarOdometry<PT>::LidarOdometry(rclcpp::Node& inst) :
    keyframe_cloud{std::make_shared<PointCloudT>()},
    keyframe_points{std::make_shared<PointCloudT>()},
    param(inst)
{
    util::declare_param(
        inst,
        "dlo.keyframe.thresh_D",
        this->state.keyframe_thresh_dist_,
        0.1);

    this->initState();
}


template<typename PT>
void LidarOdometry<PT>::initState()
{
    std::unique_lock scan_lock{this->state.mtx};

    this->state.submap_hasChanged = true;

    const size_t submap_size =
        (this->param.submap_knn_ + this->param.submap_kcc_ +
         this->param.submap_kcv_);
    this->submap_kf_idx_curr.reserve(submap_size);
    this->submap_kf_idx_prev.reserve(submap_size);

    this->convex_hull.setDimension(3);
    this->concave_hull.setDimension(3);
    this->concave_hull.setAlpha(this->state.keyframe_thresh_dist_);
    this->concave_hull.setKeepInformation(true);

    this->gicp_s2s.setCorrespondenceRandomness(
        this->param.gicps2s_k_correspondences_);
    this->gicp_s2s.setMaxCorrespondenceDistance(
        this->param.gicps2s_max_corr_dist_);
    this->gicp_s2s.setMaximumIterations(this->param.gicps2s_max_iter_);
    this->gicp_s2s.setTransformationEpsilon(
        this->param.gicps2s_transformation_ep_);
    this->gicp_s2s.setEuclideanFitnessEpsilon(
        this->param.gicps2s_euclidean_fitness_ep_);
    this->gicp_s2s.setRANSACIterations(this->param.gicps2s_ransac_iter_);
    this->gicp_s2s.setRANSACOutlierRejectionThreshold(
        this->param.gicps2s_ransac_inlier_thresh_);
    this->gicp_s2s.setNumThreads(this->param.gicp_num_threads_);

    this->gicp.setCorrespondenceRandomness(
        this->param.gicps2m_k_correspondences_);
    this->gicp.setMaxCorrespondenceDistance(this->param.gicps2m_max_corr_dist_);
    this->gicp.setMaximumIterations(this->param.gicps2m_max_iter_);
    this->gicp.setTransformationEpsilon(this->param.gicps2m_transformation_ep_);
    this->gicp.setEuclideanFitnessEpsilon(
        this->param.gicps2m_euclidean_fitness_ep_);
    this->gicp.setRANSACIterations(this->param.gicps2m_ransac_iter_);
    this->gicp.setRANSACOutlierRejectionThreshold(
        this->param.gicps2m_ransac_inlier_thresh_);
    this->gicp.setNumThreads(this->param.gicp_num_threads_);

    typename pcl::Registration<PointT, PointT>::KdTreeReciprocalPtr temp;
    this->gicp_s2s.setSearchMethodSource(temp, true);
    this->gicp_s2s.setSearchMethodTarget(temp, true);
    this->gicp.setSearchMethodSource(temp, true);
    this->gicp.setSearchMethodTarget(temp, true);

    this->vf_scan.setLeafSize(
        this->param.vf_scan_res_,
        this->param.vf_scan_res_,
        this->param.vf_scan_res_);
    this->vf_submap.setLeafSize(
        this->param.vf_submap_res_,
        this->param.vf_submap_res_,
        this->param.vf_submap_res_);
}


template<typename PT>
bool LidarOdometry<PT>::setInitial(const util::geom::Pose3f& pose)
{
    std::unique_lock scan_lock{this->state.mtx};

    if (!this->target_cloud)
    {
        // set known position
        this->state.T.template block<3, 1>(0, 3) =
            this->state.T_s2s.template block<3, 1>(0, 3) =
                this->state.T_s2s_prev.template block<3, 1>(0, 3) =
                    this->state.translation = pose.vec;

        // set known orientation
        this->state.last_rotq = this->state.rotq = pose.quat;
        this->state.T.template block<3, 3>(0, 0) =
            this->state.T_s2s.template block<3, 3>(0, 0) =
                this->state.T_s2s_prev.template block<3, 3>(0, 0) =
                    this->state.rotq.toRotationMatrix();

        return true;
    }
    return false;
}

template<typename PT>
typename LidarOdometry<PT>::IterationStatus LidarOdometry<PT>::processScan(
    const PointCloudT& scan,
    double stamp,
    util::geom::PoseTf3f& odom_tf,
    const std::optional<Mat4f>& align_estimate)
{
    std::unique_lock scan_lock{this->state.mtx};

    if (stamp <= this->state.prev_frame_stamp)
    {
        std::cout << "[ODOMETRY]: Scan timestamp is older than the previous by "
                  << (this->state.prev_frame_stamp - stamp) << "s!" << std::endl;
        return {};
    }

    const uint32_t prev_num_keyframes = this->state.num_keyframes;
    this->state.curr_frame_stamp = stamp;

    if (!this->preprocessPoints(scan))
    {
        return {};
    }

    // Set initial frame as target
    if (!this->target_cloud)
    {
        this->initializeInputTarget();

        odom_tf.pose.vec = this->state.translation;
        odom_tf.pose.quat = this->state.rotq;
        odom_tf.tf = this->state.T;

        this->current_scan = nullptr;

        IterationStatus status;
        status.odom_updated = true;
        status.keyframe_init = true;
        status.total_keyframes = this->state.num_keyframes;
        return status;
    }
    else if (this->state.rolling_scan_delta_t <= 0)
    {
        this->state.rolling_scan_delta_t =
            (this->state.curr_frame_stamp - this->state.prev_frame_stamp);
    }
    else
    {
        (this->state.rolling_scan_delta_t *= 0.9) +=
            ((this->state.curr_frame_stamp - this->state.prev_frame_stamp) *
             0.1);
    }

    // Set new frame as input source for both gicp objects
    this->setInputSources();

    // Get the next pose via IMU + S2S + S2M
    this->getNextPose(align_estimate);

    // Transform point cloud
    this->transformCurrentScan();

    // Update current keyframe poses and map
    this->updateKeyframes();

    // export tf
    odom_tf.pose.vec = this->state.translation;
    odom_tf.pose.quat = this->state.rotq;
    odom_tf.tf = this->state.T;

    // Update trajectory
    // this->trajectory.push_back(std::make_pair(this->state.translation, this->state.rotq));  // TODO (do this better)

    // Update next time stamp
    this->state.prev_frame_stamp = this->state.curr_frame_stamp;

    this->current_scan = nullptr;

    IterationStatus status;
    status.odom_updated = true;
    status.new_keyframe = (this->state.num_keyframes > prev_num_keyframes);
    status.total_keyframes = this->state.num_keyframes;
    return status;
}


template<typename PT>
void LidarOdometry<PT>::publishDebugScans(
    util::GenericPubMap& pub,
    IterationStatus proc_status,
    const std::string& odom_frame_id)
{
    if (proc_status)
    {
        std::unique_lock scan_lock{this->state.mtx};

        PointCloudMsg output;
        output.header.stamp = util::toTimeStamp(this->state.curr_frame_stamp);

        if (this->current_scan_t)
        {
            pcl::toROSMsg(*this->current_scan_t, output);
            output.header.frame_id = odom_frame_id;
            pub.publish("lidar_odom/voxelized_scan", output);
        }
        if (this->submap_cloud)
        {
            pcl::toROSMsg(*this->submap_cloud, output);
            output.header.frame_id = odom_frame_id;
            pub.publish("lidar_odom/submap_cloud", output);
        }
        if ((proc_status.keyframe_init || proc_status.new_keyframe) &&
            this->keyframe_cloud)
        {
            pcl::toROSMsg(*this->keyframe_cloud, output);
            output.header.frame_id = odom_frame_id;
            pub.publish("lidar_odom/keyframe_cloud", output);
        }
    }
}

template<typename PT>
void LidarOdometry<PT>::publishDebugMetrics(util::GenericPubMap& pub)
{
    std::unique_lock scan_lock{this->state.mtx};

    pub.publish(
        "lidar_odom/avg_range_lpf",
        util::to_ros_val<Float64Msg>(this->state.range_avg_lpf));
    pub.publish(
        "lidar_odom/dev_range_lpf",
        util::to_ros_val<Float64Msg>(this->state.range_stddev_lpf));
    pub.publish(
        "lidar_odom/adaptive_voxel_size",
        util::to_ros_val<Float64Msg>(this->state.adaptive_voxel_size));
}


template<typename PT>
bool LidarOdometry<PT>::preprocessPoints(const PointCloudT& scan)
{
    // Check num points (pre)
    if (int64_t x =
            (static_cast<int64_t>(scan.points.size()) <
             this->param.gicp_min_num_points_))
    {
        std::cout << "[ODOMETRY]: Input cloud does not have enough points: "
                  << x << std::endl;
        return false;
    }

    // Find new voxel size before applying filter
    if (this->param.adaptive_params_use_)
    {
        this->setAdaptiveParams(scan);
    }

    if (!this->current_scan)
    {
        this->current_scan = std::make_shared<PointCloudT>();
    }

    PointCloudConstPtrT cloud = util::wrap_unmanaged(&scan);

#if 0
    if (this->param.immediate_filter_use_)
    {
        pcl::Indices in_range;
        util::pc_filter_distance(
            scan,
            in_range,
            0.,
            this->param.immediate_filter_range_);
        if (in_range.size() >
            scan.points.size() * this->param.immediate_filter_thresh_)
        {
            util::pc_copy_inverse_selection(
                scan,
                in_range,
                *this->current_scan);
            cloud = this->current_scan;
        }

        if (int64_t x = static_cast<int64_t>(cloud->points.size()) <
                        this->param.gicp_min_num_points_)
        {
            std::cout
                << "[ODOMETRY]: Post-processed cloud does not have enough points: "
                << x << std::endl;
            return false;
        }
    }
#endif

    // Voxel Grid Filter
    if (this->param.vf_scan_use_)
    {
        this->vf_scan.setInputCloud(cloud);
        this->vf_scan.filter(*this->current_scan);
    }
    else if (cloud.get() == &scan)
    {
        *this->current_scan = scan;
    }

    return true;
}

template<typename PT>
void LidarOdometry<PT>::setAdaptiveParams(const PointCloudT& scan)
{
    // compute range of points "spaciousness"
    const size_t n_points = scan.points.size();
    constexpr static size_t DOWNSAMPLE_SHIFT = 5;
    std::vector<double> ds;
    double avg = 0.;
    ds.clear();
    ds.reserve(n_points >> DOWNSAMPLE_SHIFT);

    for (size_t i = 0; i < n_points >> DOWNSAMPLE_SHIFT; i++)
    {
        const auto& p = scan.points[i << DOWNSAMPLE_SHIFT];
        const double d = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        ds.push_back(d);
        avg = ((avg * i) + d) / (i + 1);
    }
    double dev = 0.;
    for (size_t i = 0; i < ds.size(); i++)
    {
        double diff = ds[i] - avg;
        dev = ((dev * i) + diff * diff) / (i + 1);
    }
    dev = std::sqrt(dev);

    if (this->state.range_avg_lpf < 0.)
    {
        this->state.range_avg_lpf = avg;
    }
    if (this->state.range_stddev_lpf < 0.)
    {
        this->state.range_stddev_lpf = dev;
    }
    const double avg_lpf =
        this->param.adaptive_params_lpf_coeff_ * this->state.range_avg_lpf +
        (1. - this->param.adaptive_params_lpf_coeff_) * avg;
    const double dev_lpf =
        this->param.adaptive_params_lpf_coeff_ * this->state.range_stddev_lpf +
        (1. - this->param.adaptive_params_lpf_coeff_) * dev;
    this->state.range_avg_lpf = avg_lpf;
    this->state.range_stddev_lpf = dev_lpf;

    const double leaf_size =
        (this->param.adaptive_voxel_offset_ +
         this->param.adaptive_voxel_range_coeff_ * avg_lpf +
         this->param.adaptive_voxel_stddev_coeff_ * dev_lpf);
    this->state.adaptive_voxel_size = std::clamp(
        (std::floor((leaf_size / this->param.adaptive_voxel_precision_) + 0.5) *
         this->param.adaptive_voxel_precision_),
        this->param.adaptive_voxel_floor_,
        this->param.adaptive_voxel_ceil_);

    this->vf_scan.setLeafSize(
        this->state.adaptive_voxel_size,
        this->state.adaptive_voxel_size,
        this->state.adaptive_voxel_size);
    this->vf_submap.setLeafSize(
        this->state.adaptive_voxel_size,
        this->state.adaptive_voxel_size,
        this->state.adaptive_voxel_size);

    // Set Keyframe Thresh from Spaciousness Metric
    if (avg_lpf > 25.0)
    {
        this->state.keyframe_thresh_dist_ = 10.0;
    }
    else if (avg_lpf > 12.0)
    {
        this->state.keyframe_thresh_dist_ = 5.0;
    }
    else if (avg_lpf > 6.0)
    {
        this->state.keyframe_thresh_dist_ = 1.0;
    }
    else if (avg_lpf <= 6.0)
    {
        this->state.keyframe_thresh_dist_ = 0.5;
    }

    // set concave hull alpha
    this->concave_hull.setAlpha(this->state.keyframe_thresh_dist_);
}

template<typename PT>
void LidarOdometry<PT>::initializeInputTarget()
{
    this->state.prev_frame_stamp = this->state.curr_frame_stamp;

    // Convert ros message
    this->target_cloud = this->current_scan;
    this->gicp_s2s.setInputTarget(this->target_cloud);
    this->gicp_s2s.calculateTargetCovariances();

    // initialize keyframes
    PointCloudPtrT first_keyframe = std::make_shared<PointCloudT>();
    pcl::transformPointCloud(
        *this->target_cloud,
        *first_keyframe,
        this->state.T);

    // voxelization for submap
    if (this->param.vf_submap_use_)
    {
        this->vf_submap.setInputCloud(first_keyframe);
        this->vf_submap.filter(*first_keyframe);
    }

    // keep history of keyframes
    this->keyframes.emplace_back(
        std::make_pair(this->state.translation, this->state.rotq),
        first_keyframe);
    this->keyframe_points->emplace_back(
        this->state.translation.x(),
        this->state.translation.y(),
        this->state.translation.z());

    *this->keyframe_cloud = *first_keyframe;

    // compute kdtree and keyframe normals (use gicp_s2s input source as temporary storage because it will be
    // overwritten by setInputSources())
    this->gicp_s2s.setInputSource(this->keyframe_cloud);
    this->gicp_s2s.calculateSourceCovariances();
    this->keyframe_normals.push_back(this->gicp_s2s.getSourceCovariances());

    ++this->state.num_keyframes;
}

template<typename PT>
void LidarOdometry<PT>::setInputSources()
{
    // set the input source for the S2S gicp
    // this builds the KdTree of the source cloud
    // this does not build the KdTree for s2m because force_no_update is true
    this->gicp_s2s.setInputSource(this->current_scan);

    // set pcl::Registration input source for S2M gicp using custom NanoGICP function
    this->gicp.registerInputSource(this->current_scan);

    // now set the KdTree of S2M gicp using previously built KdTree
    this->gicp.source_kdtree_ = this->gicp_s2s.source_kdtree_;
    this->gicp.source_covs_.clear();
}

template<typename PT>
void LidarOdometry<PT>::getNextPose(const std::optional<Mat4f>& align_estimate)
{
    //
    // FRAME-TO-FRAME PROCEDURE
    //

    // Align using IMU prior if available
    if (align_estimate.has_value())
    {
        this->gicp_s2s.align(this->scratch_cloud, align_estimate.value());
    }
    else
    {
        this->gicp_s2s.align(this->scratch_cloud);
    }

    if (!this->gicp_s2s.hasConverged())
    {
        std::cout << "[ODOMETRY]: S2S alignment didn't converge!" << std::endl;
    }

    // Get the local S2S transform
    Mat4f T_S2S = this->gicp_s2s.getFinalTransformation();

    // Get the global S2S transform
    this->propagateS2S(T_S2S);

    // reuse covariances from s2s for s2m
    this->gicp.source_covs_ = this->gicp_s2s.source_covs_;

    // Swap source and target (which also swaps KdTrees internally) for next S2S
    this->gicp_s2s.swapSourceAndTarget();

    //
    // FRAME-TO-SUBMAP
    //

    // Get current global submap
    this->getSubmapKeyframes();

    if (this->state.submap_hasChanged)
    {
        // Set the current global submap as the target cloud
        this->gicp.setInputTarget(this->submap_cloud);

        // Set target cloud's normals as submap normals
        this->gicp.setTargetCovariances(this->submap_normals);
    }

    // Align with current submap with global S2S transformation as initial guess
    this->gicp.align(this->scratch_cloud, this->state.T_s2s);

    if (!this->gicp.hasConverged())
    {
        std::cout << "[ODOMETRY]: S2M alignment didn't converge!" << std::endl;
    }

    // Get final transformation in global frame
    this->state.T = this->gicp.getFinalTransformation();

    // Update the S2S transform for next propagation
    this->state.T_s2s_prev = this->state.T;

    // Update next global pose
    // Both source and target clouds are in the global frame now, so tranformation is global
    this->propagateS2M();

    // Set next target cloud as current source cloud
    *this->target_cloud = *this->current_scan;
}

template<typename PT>
void LidarOdometry<PT>::propagateS2S(const Mat4f& T)
{
    this->state.T_s2s = this->state.T_s2s_prev * T;
    this->state.T_s2s_prev = this->state.T_s2s;
}

template<typename PT>
void LidarOdometry<PT>::getSubmapKeyframes()
{
    // clear vector of keyframe indices to use for submap
    this->submap_kf_idx_curr.clear();

    //
    // K NEAREST NEIGHBORS FROM ALL KEYFRAMES
    //

    // calculate distance between current pose and poses in keyframe set
    std::vector<float> ds;
    std::vector<int> keyframe_nn;
    int i = 0;
    Vec3f curr_pose = this->state.T_s2s.template block<3, 1>(0, 3);

    for (const auto& k : this->keyframes)
    {
        float d = static_cast<float>((curr_pose - k.first.first).norm());
        ds.push_back(d);
        keyframe_nn.push_back(i);
        i++;
    }

    // get indices for top K nearest neighbor keyframe poses
    this->pushSubmapIndices(ds, this->param.submap_knn_, keyframe_nn);

    //
    // K NEAREST NEIGHBORS FROM CONVEX HULL
    //

    // get convex hull indices
    this->computeConvexHull();

    // get distances for each keyframe on convex hull
    std::vector<float> convex_ds;
    convex_ds.reserve(this->keyframe_convex.size());
    for (const auto& c : this->keyframe_convex)
    {
        convex_ds.push_back(ds[c]);
    }

    // get indicies for top kNN for convex hull
    this->pushSubmapIndices(
        convex_ds,
        this->param.submap_kcv_,
        this->keyframe_convex);

    //
    // K NEAREST NEIGHBORS FROM CONCAVE HULL
    //

    // get concave hull indices
    this->computeConcaveHull();

    // get distances for each keyframe on concave hull
    std::vector<float> concave_ds;
    concave_ds.reserve(keyframe_concave.size());
    for (const auto& c : this->keyframe_concave)
    {
        concave_ds.push_back(ds[c]);
    }

    // get indicies for top kNN for convex hull
    this->pushSubmapIndices(
        concave_ds,
        this->param.submap_kcc_,
        this->keyframe_concave);

    //
    // BUILD SUBMAP
    //

    // check if submap has changed from previous iteration
    if (this->submap_kf_idx_curr == this->submap_kf_idx_prev)
    {
        this->state.submap_hasChanged = false;
    }
    else
    {
        this->state.submap_hasChanged = true;

        // reinitialize submap cloud, normals
        PointCloudPtrT submap_cloud_(std::make_shared<PointCloudT>());
        // this->submap_cloud->clear();
        this->submap_normals.clear();

        for (auto k : this->submap_kf_idx_curr)
        {
            // create current submap cloud
            *submap_cloud_ += *this->keyframes[k].second;

            // grab corresponding submap cloud's normals
            this->submap_normals.insert(
                std::end(this->submap_normals),
                std::begin(this->keyframe_normals[k]),
                std::end(this->keyframe_normals[k]));
        }

        this->submap_cloud = submap_cloud_;
        std::swap(this->submap_kf_idx_prev, this->submap_kf_idx_curr);
    }
}

template<typename PT>
void LidarOdometry<PT>::pushSubmapIndices(
    const std::vector<float>& dists,
    int k,
    const std::vector<int>& frames)
{
    // make sure dists is not empty
    if (dists.empty())
    {
        return;
    }

    const auto comp =
        [](const std::pair<float, int>& a, const std::pair<float, int>& b)
    { return a.first < b.first; };
    std::priority_queue<
        std::pair<float, int>,
        std::vector<std::pair<float, int>>,
        decltype(comp)>
        pq{comp};

    for (size_t i = 0; i < dists.size(); i++)
    {
        if (static_cast<int>(pq.size()) >= k && pq.top().first > dists[i])
        {
            pq.pop();
            pq.emplace(dists[i], static_cast<int>(i));
        }
        else if (static_cast<int>(pq.size()) < k)
        {
            pq.emplace(dists[i], static_cast<int>(i));
        }
    }

    if (static_cast<int>(pq.size()) > k)
    {
        throw std::logic_error("logic error in priority queue size!");
    }

    for (int i = 0; i < k; i++)
    {
        this->submap_kf_idx_curr.insert(frames[pq.top().second]);
        pq.pop();
    }
}

template<typename PT>
void LidarOdometry<PT>::computeConvexHull()
{
    // at least 4 keyframes for convex hull
    if (this->state.num_keyframes < 4)
    {
        return;
    }

    // calculate the convex hull of the point cloud
    this->convex_hull.setInputCloud(this->keyframe_points);

    // get the indices of the keyframes on the convex hull
    this->convex_hull.reconstruct(this->scratch_cloud);

    pcl::PointIndices::Ptr convex_hull_point_idx =
        std::make_shared<pcl::PointIndices>();
    this->convex_hull.getHullPointIndices(*convex_hull_point_idx);

    std::swap(this->keyframe_convex, convex_hull_point_idx->indices);
}

template<typename PT>
void LidarOdometry<PT>::computeConcaveHull()
{
    // at least 5 keyframes for concave hull
    if (this->state.num_keyframes < 5)
    {
        return;
    }

    // calculate the concave hull of the point cloud
    this->concave_hull.setInputCloud(this->keyframe_points);

    // get the indices of the keyframes on the concave hull
    this->concave_hull.reconstruct(this->scratch_cloud);

    pcl::PointIndices::Ptr concave_hull_point_idx =
        std::make_shared<pcl::PointIndices>();
    this->concave_hull.getHullPointIndices(*concave_hull_point_idx);

    std::swap(this->keyframe_concave, concave_hull_point_idx->indices);
}

template<typename PT>
void LidarOdometry<PT>::propagateS2M()
{
    this->state.translation = this->state.T.template block<3, 1>(0, 3);
    Eigen::Quaternionf q{this->state.T.template block<3, 3>(0, 0)};
    q.normalize();

    this->state.rotq = q;

    // handle sign flip
    q = this->state.last_rotq.conjugate() * this->state.rotq;
    if (q.w() < 0)
    {
        this->state.rotq.w() = -this->state.rotq.w();
        this->state.rotq.vec() = -this->state.rotq.vec();
    }
    this->state.last_rotq = this->state.rotq;
}

template<typename PT>
void LidarOdometry<PT>::transformCurrentScan()
{
    this->current_scan_t = std::make_shared<PointCloudT>();
    pcl::transformPointCloud(
        *this->current_scan,
        *this->current_scan_t,
        this->state.T);
}

template<typename PT>
void LidarOdometry<PT>::updateKeyframes()
{
    // calculate difference in pose and rotation to all poses in trajectory
    double closest_d = std::numeric_limits<double>::infinity();
    int closest_idx = 0;
    int keyframes_idx = 0;
    int num_nearby = 0;

    for (const auto& k : this->keyframes)
    {
        // calculate distance between current pose and pose in keyframes
        const double delta_d = (this->state.translation - k.first.first).norm();

        // count the number nearby current pose
        {
            if (delta_d <= this->state.keyframe_thresh_dist_ * 1.5)
            {
                num_nearby++;
            }
        }
        // store into variable
        if (delta_d < closest_d)
        {
            closest_d = delta_d;
            closest_idx = keyframes_idx;
        }

        keyframes_idx++;
    }

    // calculate difference in orientation
    double theta_rad = this->state.rotq.angularDistance(
        this->keyframes[closest_idx].first.second);
    double theta_deg = theta_rad * (180.0 / M_PI);

    // update keyframe
    const bool keyframe_close =
        abs(closest_d) > this->state.keyframe_thresh_dist_;

    const bool theta_rotated =
        abs(theta_deg) > this->param.keyframe_thresh_rot_ && num_nearby <= 1;

    if (keyframe_close || theta_rotated)
    {
        this->state.num_keyframes++;

        // voxelization for submap
        if (this->param.vf_submap_use_ && !this->param.adaptive_params_use_)
        {
            this->vf_submap.setInputCloud(this->current_scan_t);
            this->vf_submap.filter(*this->current_scan_t);
        }

        // update keyframe vector
        this->keyframes.emplace_back(
            std::make_pair(this->state.translation, this->state.rotq),
            this->current_scan_t);
        this->keyframe_points->emplace_back(
            this->state.translation.x(),
            this->state.translation.y(),
            this->state.translation.z());

        // compute kdtree and keyframe normals (use gicp_s2s input source as temporary storage because it will be
        // overwritten by setInputSources())
        *this->keyframe_cloud = *this->current_scan_t;

        this->gicp_s2s.setInputSource(this->keyframe_cloud);
        this->gicp_s2s.calculateSourceCovariances();
        this->keyframe_normals.push_back(this->gicp_s2s.getSourceCovariances());
    }
}

};  // namespace perception
};  // namespace csm
