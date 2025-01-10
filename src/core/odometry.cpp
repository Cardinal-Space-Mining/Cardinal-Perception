/*******************************************************************************
*   Copyright (C) 2024-2025 Cardinal Space Mining Club                         *
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
*                $$$: XXXXXXXXXXXXXXXXXXXXXX; :XXXXX; X$$;                     *
*                X$$X XXXXXXXXXXXXXXXXXXX; .+XXXXXXX; $$$                      *
*                $$$$ ;XXXXXXXXXXXXXXX+  +XXXXXXXXx+ X$$$+                     *
*              x$$$$$X ;XXXXXXXXXXX+ :xXXXXXXXX+   .;$$$$$$                    *
*             +$$$$$$$$ ;XXXXXXx;;+XXXXXXXXX+    : +$$$$$$$$                   *
*              +$$$$$$$$: xXXXXXXXXXXXXXX+      ; X$$$$$$$$                    *
*               :$$$$$$$$$. +XXXXXXXXX;      ;: x$$$$$$$$$                     *
*               ;x$$$$XX$$$$+ .;+X+      :;: :$$$$$xX$$$X                      *
*              ;;;;;;;;;;X$$$$$$$+      :X$$$$$$&.                             *
*              ;;;;;;;:;;;;;x$$$$$$$$$$$$$$$$x.                                *
*              :;;;;;;;;;;;;.  :$$$$$$$$$$X                                    *
*               .;;;;;;;;:;;    +$$$$$$$$$                                     *
*                 .;;;;;;.       X$$$$$$$:                                     *
*                                                                              *
*******************************************************************************/

#include "./perception.hpp"

#include "cloud_ops.hpp"

#include <queue>
#include <set>

#include <pcl_conversions/pcl_conversions.h>


namespace csm
{
namespace perception
{

PerceptionNode::LidarOdometry::LidarOdometry(PerceptionNode * inst) :
    pnode{ inst }
{
    // RCLCPP_INFO(this->pnode->get_logger(), "DLO CONSTRUCTOR INIT");

    this->state.imu_mtx.lock();
    this->state.scan_mtx.lock();

    this->state.dlo_initialized = false;
    this->state.imu_calibrated = false;
    this->state.submap_hasChanged = true;

    this->state.origin.setZero();

    this->state.T.setIdentity();
    this->state.T_s2s.setIdentity();
    this->state.T_s2s_prev.setIdentity();

    this->state.pose.setZero();
    this->state.rotq.setIdentity();

    this->state.imu_SE3.setIdentity();

    this->state.imu_bias.gyro.x = 0.;
    this->state.imu_bias.gyro.y = 0.;
    this->state.imu_bias.gyro.z = 0.;
    this->state.imu_bias.accel.x = 0.;
    this->state.imu_bias.accel.y = 0.;
    this->state.imu_bias.accel.z = 0.;

    this->state.imu_meas.stamp = 0.;
    this->state.imu_meas.ang_vel.x = 0.;
    this->state.imu_meas.ang_vel.y = 0.;
    this->state.imu_meas.ang_vel.z = 0.;
    this->state.imu_meas.lin_accel.x = 0.;
    this->state.imu_meas.lin_accel.y = 0.;
    this->state.imu_meas.lin_accel.z = 0.;

    this->getParams();

    this->imu_buffer.set_capacity(this->param.imu_buffer_size_);
    this->state.first_imu_time = 0.;

    // this->source_cloud = nullptr;
    this->current_scan = nullptr;
    this->current_scan_t = nullptr;
    this->target_cloud = nullptr;

    this->keyframe_cloud = std::make_shared<PointCloudType>();
    this->keyframe_points = std::make_shared<PointCloudType>();
    this->state.num_keyframes = 0;

    this->state.range_avg_lpf = -1.;
    this->state.range_stddev_lpf = -1.;
    this->state.adaptive_voxel_size = 0.;

    this->submap_cloud = nullptr;
    const size_t submap_size = this->param.submap_knn_ + this->param.submap_kcc_ + this->param.submap_kcv_;
    this->submap_kf_idx_curr.reserve(submap_size);
    this->submap_kf_idx_prev.reserve(submap_size);

    this->convex_hull.setDimension(3);
    this->concave_hull.setDimension(3);
    this->concave_hull.setAlpha(this->param.keyframe_thresh_dist_);
    this->concave_hull.setKeepInformation(true);

    this->gicp_s2s.setCorrespondenceRandomness(this->param.gicps2s_k_correspondences_);
    this->gicp_s2s.setMaxCorrespondenceDistance(this->param.gicps2s_max_corr_dist_);
    this->gicp_s2s.setMaximumIterations(this->param.gicps2s_max_iter_);
    this->gicp_s2s.setTransformationEpsilon(this->param.gicps2s_transformation_ep_);
    this->gicp_s2s.setEuclideanFitnessEpsilon(this->param.gicps2s_euclidean_fitness_ep_);
    this->gicp_s2s.setRANSACIterations(this->param.gicps2s_ransac_iter_);
    this->gicp_s2s.setRANSACOutlierRejectionThreshold(this->param.gicps2s_ransac_inlier_thresh_);

    this->gicp.setCorrespondenceRandomness(this->param.gicps2m_k_correspondences_);
    this->gicp.setMaxCorrespondenceDistance(this->param.gicps2m_max_corr_dist_);
    this->gicp.setMaximumIterations(this->param.gicps2m_max_iter_);
    this->gicp.setTransformationEpsilon(this->param.gicps2m_transformation_ep_);
    this->gicp.setEuclideanFitnessEpsilon(this->param.gicps2m_euclidean_fitness_ep_);
    this->gicp.setRANSACIterations(this->param.gicps2m_ransac_iter_);
    this->gicp.setRANSACOutlierRejectionThreshold(this->param.gicps2m_ransac_inlier_thresh_);

    pcl::Registration<PointType, PointType>::KdTreeReciprocalPtr temp;
    this->gicp_s2s.setSearchMethodSource(temp, true);
    this->gicp_s2s.setSearchMethodTarget(temp, true);
    this->gicp.setSearchMethodSource(temp, true);
    this->gicp.setSearchMethodTarget(temp, true);

    this->vf_scan.setLeafSize(this->param.vf_scan_res_, this->param.vf_scan_res_, this->param.vf_scan_res_);
    this->vf_submap.setLeafSize(this->param.vf_submap_res_, this->param.vf_submap_res_, this->param.vf_submap_res_);

    this->state.imu_mtx.unlock();
    this->state.scan_mtx.unlock();

    // RCLCPP_INFO(this->pnode->get_logger(), "DLO CONSTRUCTOR EXIT");
}

void PerceptionNode::LidarOdometry::getParams()
{
    if(!this->pnode) return;

    // General
    util::declare_param(this->pnode, "dlo.use_timestamps_as_init", this->param.use_scan_ts_as_init_, true);

    // Gravity alignment
    util::declare_param(this->pnode, "dlo.gravity_align", this->param.gravity_align_, false);

    // Keyframe Threshold
    util::declare_param(this->pnode, "dlo.keyframe.thresh_D", this->param.keyframe_thresh_dist_, 0.1);
    util::declare_param(this->pnode, "dlo.keyframe.thresh_R", this->param.keyframe_thresh_rot_, 1.0);

    // Submap
    util::declare_param(this->pnode, "dlo.keyframe.submap.knn", this->param.submap_knn_, 10);
    util::declare_param(this->pnode, "dlo.keyframe.submap.kcv", this->param.submap_kcv_, 10);
    util::declare_param(this->pnode, "dlo.keyframe.submap.kcc", this->param.submap_kcc_, 10);

    // Initial Position
    util::declare_param(this->pnode, "dlo.initial_pose.use", this->param.initial_pose_use_, false);

    std::vector<double> pos, quat;
    util::declare_param(this->pnode, "dlo.initial_pose.position", pos, {0., 0., 0.});
    util::declare_param(this->pnode, "dlo.initial_pose.orientation", quat, {1., 0., 0., 0.});
    this->param.initial_position_ = Eigen::Vector3d(pos[0], pos[1], pos[2]);
    this->param.initial_orientation_ = Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]);

    // Voxel Grid Filter
    util::declare_param(this->pnode, "dlo.voxel_filter.scan.use", this->param.vf_scan_use_, true);
    util::declare_param(this->pnode, "dlo.voxel_filter.scan.res", this->param.vf_scan_res_, 0.05);
    util::declare_param(this->pnode, "dlo.voxel_filter.submap.use", this->param.vf_submap_use_, false);
    util::declare_param(this->pnode, "dlo.voxel_filter.submap.res", this->param.vf_submap_res_, 0.1);
    util::declare_param(this->pnode, "dlo.voxel_filter.adaptive_leaf_size.range_coeff",
                        this->param.adaptive_voxel_range_coeff_, 0.01);
    util::declare_param(this->pnode, "dlo.voxel_filter.adaptive_leaf_size.stddev_coeff",
                        this->param.adaptive_voxel_stddev_coeff_, 0.005);
    util::declare_param(this->pnode, "dlo.voxel_filter.adaptive_leaf_size.offset",
                        this->param.adaptive_voxel_offset_, 0.);
    util::declare_param(this->pnode, "dlo.voxel_filter.adaptive_leaf_size.floor",
                        this->param.adaptive_voxel_floor_, 0.05);
    util::declare_param(this->pnode, "dlo.voxel_filter.adaptive_leaf_size.ceil",
                        this->param.adaptive_voxel_ceil_, 0.1);
    util::declare_param(this->pnode, "dlo.voxle_filter.adapative_leaf_size.precision",
                        this->param.adaptive_voxel_precision_, 0.01);

    // Immediate Filter
    util::declare_param(this->pnode, "dlo.immediate_filter.use", this->param.immediate_filter_use_, true);
    util::declare_param(this->pnode, "dlo.immediate_filter.range", this->param.immediate_filter_range_, 0.5);
    util::declare_param(this->pnode, "dlo.immediate_filter.thresh_proportion",
                        this->param.immediate_filter_thresh_, 0.4);

    // Adaptive Parameters
    util::declare_param(this->pnode, "dlo.adaptive_params.use", this->param.adaptive_params_use_, false);
    util::declare_param(this->pnode, "dlo.adaptive_params.lpf_coeff", this->param.adaptive_params_lpf_coeff_, 0.95);

    // IMU
    util::declare_param(this->pnode, "dlo.imu.use", this->param.imu_use_, false);
    util::declare_param(this->pnode, "dlo.imu.use_orientation", this->param.imu_use_orientation_, false);
    util::declare_param(this->pnode, "dlo.imu.calib_time", this->param.imu_calib_time_, 3);
    util::declare_param(this->pnode, "dlo.imu.buffer_size", this->param.imu_buffer_size_, 2000);

    // GICP
    util::declare_param(this->pnode, "dlo.gicp.min_num_points", this->param.gicp_min_num_points_, 100);
    util::declare_param(this->pnode, "dlo.gicp.s2s.k_correspondences", this->param.gicps2s_k_correspondences_, 20);
    util::declare_param(this->pnode, "dlo.gicp.s2s.max_correspondence_distance",
                        this->param.gicps2s_max_corr_dist_, std::sqrt(std::numeric_limits<double>::max()));
    util::declare_param(this->pnode, "dlo.gicp.s2s.max_iterations", this->param.gicps2s_max_iter_, 64);
    util::declare_param(this->pnode, "dlo.gicp.s2s.transformation_epsilon",
                        this->param.gicps2s_transformation_ep_, 0.0005);
    util::declare_param(this->pnode, "dlo.gicp.s2s.euclidean_fitness_epsilon",
                        this->param.gicps2s_euclidean_fitness_ep_, -std::numeric_limits<double>::max());
    util::declare_param(this->pnode, "dlo.gicp.s2s.ransac.iterations", this->param.gicps2s_ransac_iter_, 0);
    util::declare_param(this->pnode, "dlo.gicp.s2s.ransac.outlier_rejection_thresh",
                        this->param.gicps2s_ransac_inlier_thresh_, 0.05);
    util::declare_param(this->pnode, "dlo.gicp.s2m.k_correspondences", this->param.gicps2m_k_correspondences_, 20);
    util::declare_param(this->pnode, "dlo.gicp.s2m.max_correspondence_distance",
                        this->param.gicps2m_max_corr_dist_, std::sqrt(std::numeric_limits<double>::max()));
    util::declare_param(this->pnode, "dlo.gicp.s2m.max_iterations", this->param.gicps2m_max_iter_, 64);
    util::declare_param(this->pnode, "dlo.gicp.s2m.transformation_epsilon",
                        this->param.gicps2m_transformation_ep_, 0.0005);
    util::declare_param(this->pnode, "dlo.gicp.s2m.euclidean_fitness_epsilon",
                        this->param.gicps2m_euclidean_fitness_ep_, -std::numeric_limits<double>::max());
    util::declare_param(this->pnode, "dlo.gicp.s2m.ransac.iterations", this->param.gicps2m_ransac_iter_, 0);
    util::declare_param(this->pnode, "dlo.gicp.s2m.ransac.outlier_rejection_thresh",
                        this->param.gicps2m_ransac_inlier_thresh_, 0.05);
}


void PerceptionNode::LidarOdometry::processImu(const sensor_msgs::msg::Imu& imu)
{
    if(!this->param.imu_use_)
    {
        return;
    }

    if(this->param.imu_use_orientation_ && imu.orientation_covariance[0]  == -1.)
    {
        // message topic source doesn't support orientation
        this->param.imu_use_orientation_ = false;
    }

    double stamp = util::toFloatSeconds(imu.header.stamp);

    this->state.imu_mtx.lock();     // TODO: timeout

    if(this->state.first_imu_time == 0.)
    {
        this->state.first_imu_time = stamp;
    }

    if(this->param.imu_use_orientation_)
    {
        using namespace util::geom::cvt::ops;
        Eigen::Quaterniond q;
        q << imu.orientation;
        const size_t idx = util::tsq::binarySearchIdx(this->orient_buffer, stamp);
        this->orient_buffer.emplace(this->orient_buffer.begin() + idx, stamp, q);
        util::tsq::trimToStamp(this->orient_buffer, this->state.prev_frame_stamp);
    }
    else
    {
        // Get IMU samples
        double ang_vel[3], lin_accel[3];
        ang_vel[0] = imu.angular_velocity.x;
        ang_vel[1] = imu.angular_velocity.y;
        ang_vel[2] = imu.angular_velocity.z;

        lin_accel[0] = imu.linear_acceleration.x;
        lin_accel[1] = imu.linear_acceleration.y;
        lin_accel[2] = imu.linear_acceleration.z;

        // IMU calibration procedure - do for three seconds
        if(!this->state.imu_calibrated)
        {

            static int num_samples = 0;
            // static bool print = true;

            if((stamp - this->state.first_imu_time) < this->param.imu_calib_time_)
            {

                num_samples++;

                this->state.imu_bias.gyro.x += ang_vel[0];
                this->state.imu_bias.gyro.y += ang_vel[1];
                this->state.imu_bias.gyro.z += ang_vel[2];

                this->state.imu_bias.accel.x += lin_accel[0];
                this->state.imu_bias.accel.y += lin_accel[1];
                this->state.imu_bias.accel.z += lin_accel[2];

                // if(print)
                // {
                //     std::cout << "Calibrating IMU for " << this->param.imu_calib_time_ << " seconds... ";
                //     std::cout.flush();
                //     print = false;
                // }
                // TODO
            }
            else
            {

                this->state.imu_bias.gyro.x /= num_samples;
                this->state.imu_bias.gyro.y /= num_samples;
                this->state.imu_bias.gyro.z /= num_samples;

                this->state.imu_bias.accel.x /= num_samples;
                this->state.imu_bias.accel.y /= num_samples;
                this->state.imu_bias.accel.z /= num_samples;

                this->state.imu_calibrated = true;

                // std::cout << "done" << std::endl;
                // std::cout << "  Gyro biases [xyz]: " << this->state.imu_bias.gyro.x << ", " << this->state.imu_bias.gyro.y << ", "
                //           << this->state.imu_bias.gyro.z << std::endl
                //           << std::endl;
                // TODO
            }
        }
        else
        {
            // Apply the calibrated bias to the new IMU measurements
            this->state.imu_meas.stamp = stamp;

            this->state.imu_meas.ang_vel.x = ang_vel[0] - this->state.imu_bias.gyro.x;
            this->state.imu_meas.ang_vel.y = ang_vel[1] - this->state.imu_bias.gyro.y;
            this->state.imu_meas.ang_vel.z = ang_vel[2] - this->state.imu_bias.gyro.z;

            this->state.imu_meas.lin_accel.x = lin_accel[0];
            this->state.imu_meas.lin_accel.y = lin_accel[1];
            this->state.imu_meas.lin_accel.z = lin_accel[2];

            // Store into circular buffer
            this->imu_buffer.push_front(this->state.imu_meas);
        }
    }

    this->state.imu_mtx.unlock();
}


PerceptionNode::LidarOdometry::IterationStatus PerceptionNode::LidarOdometry::processScan(
    const PointCloudType& scan,
    double stamp,
    util::geom::PoseTf3f& odom_tf )
{
    std::unique_lock _lock{ this->state.scan_mtx };
    const uint32_t prev_num_keyframes = this->state.num_keyframes;
    this->state.curr_frame_stamp = stamp;

    // DLO Initialization procedures (IMU calib, gravity align)
    if(!this->state.dlo_initialized)
    {
        this->initializeDLO();
        if(!this->state.dlo_initialized) return 0;  // uninitialized
    }

    if(!this->preprocessPoints(scan)) return 0;

    // Set initial frame as target
    if(this->target_cloud == nullptr)
    {
        this->initializeInputTarget();

        odom_tf.pose.vec = this->state.pose;
        odom_tf.pose.quat = this->state.rotq;
        odom_tf.tf = this->state.T;

        this->current_scan = nullptr;

        IterationStatus status;
        status.odom_updated = true;
        status.keyframe_init = true;
        status.total_keyframes = this->state.num_keyframes;
        return status;
    }
    else if(this->state.rolling_scan_delta_t <= 0)
    {
        this->state.rolling_scan_delta_t = (this->state.curr_frame_stamp = this->state.prev_frame_stamp);
    }
    else
    {
        (this->state.rolling_scan_delta_t *= 0.9) +=
            ((this->state.curr_frame_stamp = this->state.prev_frame_stamp) * 0.1);
    }

    // Set source frame
    // this->source_cloud = std::make_shared<PointCloudType>();
    // this->source_cloud = this->current_scan;

    // Set new frame as input source for both gicp objects
    this->setInputSources();

    // Get the next pose via IMU + S2S + S2M
    this->getNextPose();

    // Update current keyframe poses and map
    this->updateKeyframes();

    // export tf
    odom_tf.pose.vec = this->state.pose;
    odom_tf.pose.quat = this->state.rotq;
    odom_tf.tf = this->state.T;

    // Update trajectory
    // this->trajectory.push_back(std::make_pair(this->state.pose, this->state.rotq));  // TODO (do this better)

    // Update next time stamp
    this->state.prev_frame_stamp = this->state.curr_frame_stamp;

    this->current_scan = nullptr;

    IterationStatus status;
    status.odom_updated = true;
    status.new_keyframe = (this->state.num_keyframes > prev_num_keyframes);
    status.total_keyframes = this->state.num_keyframes;
    return status;
}


void PerceptionNode::LidarOdometry::publishDebugScans(IterationStatus proc_status)
{
    if(proc_status)
    {
        try
        {
            sensor_msgs::msg::PointCloud2 output;
            output.header.stamp = util::toTimeStamp(this->state.curr_frame_stamp);

            if(this->current_scan_t)
            {
                pcl::toROSMsg(*this->current_scan_t, output);
                output.header.frame_id = this->pnode->odom_frame;
                this->pnode->scan_pub.publish("dlo/voxelized_scan", output);
            }
            if(this->submap_cloud)
            {
                pcl::toROSMsg(*this->submap_cloud, output);
                output.header.frame_id = this->pnode->odom_frame;
                this->pnode->scan_pub.publish("dlo/submap_cloud", output);
            }
            if((proc_status.keyframe_init || proc_status.new_keyframe) && this->keyframe_cloud)
            {
                pcl::toROSMsg(*this->keyframe_cloud, output);
                output.header.frame_id = this->pnode->odom_frame;
                this->pnode->scan_pub.publish("dlo/keyframe_cloud", output);
            }
        }
        catch(const std::exception& e)
        {
            RCLCPP_INFO(this->pnode->get_logger(), "[DLO]: Failed to publish debug scans -- what():\n\t%s", e.what());
        }
    }
}


void PerceptionNode::LidarOdometry::initializeDLO()
{
    // Calibrate IMU
    if(!this->state.imu_calibrated && this->param.imu_use_ && !this->param.imu_use_orientation_)
    {
        return;
    }

    // Gravity Align
    if(this->param.gravity_align_ && this->param.imu_use_ &&
        this->state.imu_calibrated && !this->param.initial_pose_use_ && !this->param.imu_use_orientation_)
    {
        // std::cout << "Aligning to gravity... ";
        // std::cout.flush();
        this->gravityAlign();
    }

    // TODO: option for initializing off of imu orientation

    // Use initial known pose
    if(this->param.initial_pose_use_)
    {
        // std::cout << "Setting known initial pose... ";
        // std::cout.flush();

        // set known position
        this->state.T.block<3, 1>(0, 3) =
            this->state.T_s2s.block<3, 1>(0, 3) =
            this->state.T_s2s_prev.block<3, 1>(0, 3) =
            this->state.pose =
            this->state.origin =
            this->param.initial_position_.template cast<float>();

        // set known orientation
        this->state.rotq = this->param.initial_orientation_.template cast<float>();
        this->state.T.block<3, 3>(0, 0) =
            this->state.T_s2s.block<3, 3>(0, 0) =
            this->state.T_s2s_prev.block<3, 3>(0, 0) =
            this->state.rotq.toRotationMatrix();

        // std::cout << "done" << std::endl << std::endl;
    }

    this->state.dlo_initialized = true;
    // std::cout << "DLO initialized! Starting localization..." << std::endl;   // TODO
}

void PerceptionNode::LidarOdometry::gravityAlign()
{
    // get average acceleration vector for 1 second and normalize
    Eigen::Vector3d lin_accel = Eigen::Vector3d::Zero();
    auto then = PerceptionNode::ClockType::now();
    size_t n = 0;
    while(1. > std::chrono::duration<double>(PerceptionNode::ClockType::now() - then).count())
    {
        this->state.imu_mtx.lock();
        lin_accel[0] += this->state.imu_meas.lin_accel.x;
        lin_accel[1] += this->state.imu_meas.lin_accel.y;
        lin_accel[2] += this->state.imu_meas.lin_accel.z;
        this->state.imu_mtx.unlock();
        ++n;

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    lin_accel[0] /= n;
    lin_accel[1] /= n;
    lin_accel[2] /= n;

    // normalize
    lin_accel.normalize();

    // set gravity aligned orientation
    this->state.rotq =
        Eigen::Quaterniond::FromTwoVectors(
            lin_accel,
            Eigen::Vector3d{ 0., 0., 1. } )
        .normalized()
        .template cast<float>();

    this->state.T.block<3, 3>(0, 0) =
        this->state.T_s2s.block<3, 3>(0, 0) =
        this->state.T_s2s_prev.block<3, 3>(0, 0) =
        this->state.rotq.toRotationMatrix();

    // rpy
    // auto euler = grav_q.toRotationMatrix().eulerAngles(2, 1, 0);
    // double yaw = euler[0] * (180.0 / M_PI);
    // double pitch = euler[1] * (180.0 / M_PI);
    // double roll = euler[2] * (180.0 / M_PI);

    // std::cout << "done" << std::endl;
    // std::cout << "  Roll [deg]: " << roll << std::endl;
    // std::cout << "  Pitch [deg]: " << pitch << std::endl << std::endl;
}

bool PerceptionNode::LidarOdometry::preprocessPoints(const PointCloudType& scan)
{
    // Check num points (pre)
    if(int64_t x = static_cast<int64_t>(scan.points.size()) < this->param.gicp_min_num_points_)
    {
        RCLCPP_INFO(this->pnode->get_logger(), "[DLO]: Input cloud does not have enough points: %ld", x);
        return false;
    }

    // Find new voxel size before applying filter
    if(this->param.adaptive_params_use_)
    {
        this->setAdaptiveParams(scan);
    }

    if(!this->current_scan)
    {
        this->current_scan = std::make_shared<PointCloudType>();
    }
    PointCloudType::ConstPtr cloud = util::wrap_unmanaged(&scan);

    if(this->param.immediate_filter_use_)
    {
        pcl::Indices in_range;
        util::pc_filter_distance(scan, in_range, 0., this->param.immediate_filter_range_);
        if(in_range.size() > scan.points.size() * this->param.immediate_filter_thresh_)
        {
            util::pc_copy_inverse_selection(scan, in_range, *this->current_scan);
            cloud = this->current_scan;
        }

        if(int64_t x = static_cast<int64_t>(cloud->points.size()) < this->param.gicp_min_num_points_)
        {
            RCLCPP_INFO(this->pnode->get_logger(), "[DLO]: Post-processed cloud does not have enough points: %ld", x);
            return false;
        }
    }

    // Voxel Grid Filter
    if(this->param.vf_scan_use_)
    {
        this->vf_scan.setInputCloud(cloud);
        this->vf_scan.filter(*this->current_scan);
    }
    else if(cloud.get() == &scan)
    {
        *this->current_scan = scan;
    }

    return true;
}

void PerceptionNode::LidarOdometry::setAdaptiveParams(const PointCloudType& scan)
{
    // compute range of points "spaciousness"
    const size_t n_points = scan.points.size();
    constexpr static size_t DOWNSAMPLE_SHIFT = 5;
    std::vector<double> ds;
    double avg = 0.;
    ds.clear();
    ds.reserve(n_points >> DOWNSAMPLE_SHIFT);

    for(size_t i = 0; i < n_points >> DOWNSAMPLE_SHIFT; i++)
    {
        auto& p = scan.points[i << DOWNSAMPLE_SHIFT];
        const double d = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        ds.push_back(d);
        avg = ((avg * i) + d) / (i + 1);
    }
    double dev = 0.;
    for(size_t i = 0; i < ds.size(); i++)
    {
        double diff = ds[i] - avg;
        dev = ((dev * i) + diff * diff) / (i + 1);
    }
    dev = std::sqrt(dev);

    if(this->state.range_avg_lpf < 0.) this->state.range_avg_lpf = avg;
    if(this->state.range_stddev_lpf < 0.) this->state.range_stddev_lpf = dev;
    const double avg_lpf = this->param.adaptive_params_lpf_coeff_ * this->state.range_avg_lpf +
                        (1. - this->param.adaptive_params_lpf_coeff_) * avg;
    const double dev_lpf = this->param.adaptive_params_lpf_coeff_ * this->state.range_stddev_lpf +
                        (1. - this->param.adaptive_params_lpf_coeff_) * dev;
    this->state.range_avg_lpf = avg_lpf;
    this->state.range_stddev_lpf = dev_lpf;

    this->pnode->metrics_pub.publish("dlo/avg_range_lpf", avg_lpf);
    this->pnode->metrics_pub.publish("dlo/dev_range_lpf", dev_lpf);

    const double leaf_size =
        this->param.adaptive_voxel_offset_ +
        this->param.adaptive_voxel_range_coeff_ * avg_lpf +
        this->param.adaptive_voxel_stddev_coeff_ * dev_lpf;
    this->state.adaptive_voxel_size = std::clamp(
        std::floor((leaf_size / this->param.adaptive_voxel_precision_) + 0.5) * this->param.adaptive_voxel_precision_,
        this->param.adaptive_voxel_floor_, this->param.adaptive_voxel_ceil_ );

    this->pnode->metrics_pub.publish("dlo/adaptive_voxel_size", this->state.adaptive_voxel_size);

    this->vf_scan.setLeafSize(this->state.adaptive_voxel_size, this->state.adaptive_voxel_size, this->state.adaptive_voxel_size);
    this->vf_submap.setLeafSize(this->state.adaptive_voxel_size, this->state.adaptive_voxel_size, this->state.adaptive_voxel_size);

    // Set Keyframe Thresh from Spaciousness Metric
    if(avg_lpf > 25.0)
    {
        this->param.keyframe_thresh_dist_ = 10.0;
    }
    else if(avg_lpf > 12.0)
    {
        this->param.keyframe_thresh_dist_ = 5.0;
    }
    else if(avg_lpf > 6.0)
    {
        this->param.keyframe_thresh_dist_ = 1.0;
    }
    else if(avg_lpf <= 6.0)
    {
        this->param.keyframe_thresh_dist_ = 0.5;
    }

    // set concave hull alpha
    this->concave_hull.setAlpha(this->param.keyframe_thresh_dist_);
}

void PerceptionNode::LidarOdometry::initializeInputTarget()
{
    this->state.prev_frame_stamp = this->state.curr_frame_stamp;

    // Convert ros message
    // this->target_cloud = std::make_shared<PointCloudType>();     // A
    // this->target_cloud = nullptr;
    this->target_cloud = this->current_scan;
    this->gicp_s2s.setInputTarget(this->target_cloud);
    this->gicp_s2s.calculateTargetCovariances();

    // initialize keyframes
    PointCloudType::Ptr first_keyframe = std::make_shared<PointCloudType>();
    pcl::transformPointCloud(*this->target_cloud, *first_keyframe, this->state.T);

    // voxelization for submap
    if(this->param.vf_submap_use_)
    {
        this->vf_submap.setInputCloud(first_keyframe);
        this->vf_submap.filter(*first_keyframe);
    }

    // keep history of keyframes
    this->keyframes.emplace_back(
        std::make_pair(this->state.pose, this->state.rotq),
        first_keyframe );
    // pcl::PointXYZL pt;
    // pt.x = this->state.pose.x();
    // pt.y = this->state.pose.y();
    // pt.z = this->state.pose.z();
    // pt.label = this->keyframes.size() - 1;
    this->keyframe_points->emplace_back(
        this->state.pose.x(),
        this->state.pose.y(),
        this->state.pose.z() );
    // this->keyframe_points_kdtree.Add_Point(pt, false);
    // *this->keyframes_cloud += *first_keyframe;
    *this->keyframe_cloud = *first_keyframe;

    // compute kdtree and keyframe normals (use gicp_s2s input source as temporary storage because it will be
    // overwritten by setInputSources())
    this->gicp_s2s.setInputSource(this->keyframe_cloud);
    this->gicp_s2s.calculateSourceCovariances();
    this->keyframe_normals.push_back(this->gicp_s2s.getSourceCovariances());

    // this->publish_keyframe_thread = std::thread(&OdomNode::publishKeyframe, this);
    // this->publish_keyframe_thread.detach();

    ++this->state.num_keyframes;
}

void PerceptionNode::LidarOdometry::setInputSources()
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

void PerceptionNode::LidarOdometry::getNextPose()
{
    //
    // FRAME-TO-FRAME PROCEDURE
    //

    // Align using IMU prior if available
    // PointCloudType::Ptr aligned = std::make_shared<PointCloudType>();

    // RCLCPP_INFO(this->pnode->get_logger(), "DLO: SCAN PROCESSING EXHIBIT E-1");
    if(this->param.imu_use_)
    {
        this->integrateIMU();
    // RCLCPP_INFO(this->pnode->get_logger(), "DLO: SCAN PROCESSING EXHIBIT E-2a");
        this->gicp_s2s.align(this->scratch_cloud, this->state.imu_SE3);
    }
    else
    {
        this->gicp_s2s.align(this->scratch_cloud);
    // RCLCPP_INFO(this->pnode->get_logger(), "DLO: SCAN PROCESSING EXHIBIT E-2b");
    }
    // RCLCPP_INFO(this->pnode->get_logger(), "DLO: SCAN PROCESSING EXHIBIT E-3");

    // Get the local S2S transform
    Eigen::Matrix4f T_S2S = this->gicp_s2s.getFinalTransformation();

    // RCLCPP_INFO(this->pnode->get_logger(), "DLO: SCAN PROCESSING EXHIBIT E-4");

    // Get the global S2S transform
    this->propagateS2S(T_S2S);

    // RCLCPP_INFO(this->pnode->get_logger(), "DLO: SCAN PROCESSING EXHIBIT E-5");

    // reuse covariances from s2s for s2m
    this->gicp.source_covs_ = this->gicp_s2s.source_covs_;

    // Swap source and target (which also swaps KdTrees internally) for next S2S
    this->gicp_s2s.swapSourceAndTarget();

    // RCLCPP_INFO(this->pnode->get_logger(), "DLO: SCAN PROCESSING EXHIBIT E-6");

    //
    // FRAME-TO-SUBMAP
    //

    // Get current global submap
    this->getSubmapKeyframes();

    // RCLCPP_INFO(this->pnode->get_logger(), "DLO: SCAN PROCESSING EXHIBIT E-7");

    if(this->state.submap_hasChanged)
    {

        // Set the current global submap as the target cloud
        this->gicp.setInputTarget(this->submap_cloud);

        // Set target cloud's normals as submap normals
        this->gicp.setTargetCovariances(this->submap_normals);
    }

    // RCLCPP_INFO(this->pnode->get_logger(), "DLO: SCAN PROCESSING EXHIBIT E-8");

    // Align with current submap with global S2S transformation as initial guess
    this->gicp.align(this->scratch_cloud, this->state.T_s2s);

    // RCLCPP_INFO(this->pnode->get_logger(), "DLO: SCAN PROCESSING EXHIBIT E-9");

    // Get final transformation in global frame
    this->state.T = this->gicp.getFinalTransformation();

    // RCLCPP_INFO(this->pnode->get_logger(), "DLO: SCAN PROCESSING EXHIBIT E-10");

    // Update the S2S transform for next propagation
    this->state.T_s2s_prev = this->state.T;

    // Update next global pose
    // Both source and target clouds are in the global frame now, so tranformation is global
    this->propagateS2M();

    // RCLCPP_INFO(this->pnode->get_logger(), "DLO: SCAN PROCESSING EXHIBIT E-11");

    // Set next target cloud as current source cloud
    *this->target_cloud = *this->current_scan;
}

void PerceptionNode::LidarOdometry::integrateIMU()
{
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();

    const double
        init_stamp = this->param.use_scan_ts_as_init_ ?
            this->state.curr_frame_stamp :
            this->state.prev_frame_stamp,
        end_stamp = this->param.use_scan_ts_as_init_ ?
            this->state.curr_frame_stamp + this->state.rolling_scan_delta_t :
            this->state.curr_frame_stamp;

    if(this->param.imu_use_orientation_)
    {
        this->state.imu_mtx.lock();
        const size_t
            init_idx = util::tsq::binarySearchIdx(this->orient_buffer, init_stamp),
            end_idx = util::tsq::binarySearchIdx(this->orient_buffer, end_stamp);

        Eigen::Quaterniond
            prev_rotation = Eigen::Quaterniond::Identity(),
            curr_rotation = Eigen::Quaterniond::Identity();

        // get interpolated previous pose
        if(init_idx == 0)
        {
            prev_rotation = this->orient_buffer[0].second;
        }
        else if(init_idx == this->orient_buffer.size())
        {
            prev_rotation = this->orient_buffer.back().second;
        }
        else if(util::tsq::validLerpIdx(this->orient_buffer, init_idx))
        {
            const auto&
                pre = this->orient_buffer[init_idx],
                post = this->orient_buffer[init_idx - 1];
            prev_rotation = pre.second.slerp(
                ((init_stamp - pre.first) / (post.first - pre.first)), post.second);
        }

        // get interpolated current imu pose
        if(end_idx == 0)
        {
            curr_rotation = this->orient_buffer[0].second;
        }
        else if(end_idx == this->orient_buffer.size())
        {
            curr_rotation = this->orient_buffer.back().second;
        }
        else if(util::tsq::validLerpIdx(this->orient_buffer, end_idx))
        {
            const auto&
                pre = this->orient_buffer[end_idx],
                post = this->orient_buffer[end_idx - 1];
            curr_rotation = pre.second.slerp(
                ((end_stamp - pre.first) / (post.first - pre.first)), post.second);
        }
        this->state.imu_mtx.unlock();

        q = prev_rotation.inverse() * curr_rotation;
    }
    else
    {
        // Extract IMU data between the two frames
        std::vector<ImuMeas> imu_frame;

        this->state.imu_mtx.lock();
        for(const auto & i : this->imu_buffer)
        {
            // IMU data between two frames is when:
            //   current frame's timestamp minus imu timestamp is positive
            //   previous frame's timestamp minus imu timestamp is negative
            double end_frame_imu_dt = end_stamp - i.stamp;
            double init_frame_imu_dt = init_stamp - i.stamp;

            if(end_frame_imu_dt >= 0. && init_frame_imu_dt <= 0.)
            {
                imu_frame.push_back(i);
            }
        }
        this->state.imu_mtx.unlock();

        // Sort measurements by time
        std::sort(imu_frame.begin(), imu_frame.end(), this->comparatorImu);

        // Relative IMU integration of gyro and accelerometer
        double curr_imu_stamp = 0.;
        double prev_imu_stamp = 0.;
        double dt;

        for(uint32_t i = 0; i < imu_frame.size(); ++i)
        {

            if(prev_imu_stamp == 0.)
            {
                prev_imu_stamp = imu_frame[i].stamp;
                continue;
            }

            // Calculate difference in imu measurement times IN SECONDS
            curr_imu_stamp = imu_frame[i].stamp;
            dt = curr_imu_stamp - prev_imu_stamp;
            prev_imu_stamp = curr_imu_stamp;

            // Relative gyro propagation quaternion dynamics
            Eigen::Quaterniond qq = q;
            q.w() -= 0.5 *
                (qq.x() * imu_frame[i].ang_vel.x + qq.y() * imu_frame[i].ang_vel.y + qq.z() * imu_frame[i].ang_vel.z) * dt;
            q.x() += 0.5 *
                (qq.w() * imu_frame[i].ang_vel.x - qq.z() * imu_frame[i].ang_vel.y + qq.y() * imu_frame[i].ang_vel.z) * dt;
            q.y() += 0.5 *
                (qq.z() * imu_frame[i].ang_vel.x + qq.w() * imu_frame[i].ang_vel.y - qq.x() * imu_frame[i].ang_vel.z) * dt;
            q.z() += 0.5 *
                (qq.x() * imu_frame[i].ang_vel.y - qq.y() * imu_frame[i].ang_vel.x + qq.w() * imu_frame[i].ang_vel.z) * dt;
        }

        q.normalize();
    }

    // Store IMU guess
    // this->state.imu_SE3.setIdentity();
    this->state.imu_SE3.block<3, 3>(0, 0) = q.template cast<float>().toRotationMatrix();
}

void PerceptionNode::LidarOdometry::propagateS2S(const Eigen::Matrix4f& T)
{
    this->state.T_s2s = this->state.T_s2s_prev * T;
    this->state.T_s2s_prev = this->state.T_s2s;
}

void PerceptionNode::LidarOdometry::getSubmapKeyframes()
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
    Eigen::Vector3f curr_pose = this->state.T_s2s.block<3, 1>(0, 3);

    for(const auto & k : this->keyframes)
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
    for(const auto & c : this->keyframe_convex) { convex_ds.push_back(ds[c]); }

    // get indicies for top kNN for convex hull
    this->pushSubmapIndices(convex_ds, this->param.submap_kcv_, this->keyframe_convex);

    //
    // K NEAREST NEIGHBORS FROM CONCAVE HULL
    //

    // get concave hull indices
    this->computeConcaveHull();

    // get distances for each keyframe on concave hull
    std::vector<float> concave_ds;
    for(const auto & c : this->keyframe_concave) { concave_ds.push_back(ds[c]); }

    // get indicies for top kNN for convex hull
    this->pushSubmapIndices(concave_ds, this->param.submap_kcc_, this->keyframe_concave);

    //
    // BUILD SUBMAP
    //

    // concatenate all submap clouds and normals
    // std::sort(this->submap_kf_idx_curr.begin(), this->submap_kf_idx_curr.end());
    // auto last = std::unique(this->submap_kf_idx_curr.begin(), this->submap_kf_idx_curr.end());
    // this->submap_kf_idx_curr.erase(last, this->submap_kf_idx_curr.end());

    // sort current and previous submap kf list of indices
    // std::sort(this->submap_kf_idx_curr.begin(), this->submap_kf_idx_curr.end());
    // std::sort(this->submap_kf_idx_prev.begin(), this->submap_kf_idx_prev.end());

    // check if submap has changed from previous iteration
    if(this->submap_kf_idx_curr == this->submap_kf_idx_prev)
    {
        this->state.submap_hasChanged = false;
    }
    else
    {
        this->state.submap_hasChanged = true;

        // reinitialize submap cloud, normals
        PointCloudType::Ptr submap_cloud_(std::make_shared<PointCloudType>());
        // this->submap_cloud->clear();
        this->submap_normals.clear();

        for(auto k : this->submap_kf_idx_curr)
        {

            // create current submap cloud
            *submap_cloud_ += *this->keyframes[k].second;
            // *this->submap_cloud += *this->keyframes[k].second;

            // grab corresponding submap cloud's normals
            this->submap_normals.insert(
                std::end(this->submap_normals),
                std::begin(this->keyframe_normals[k]),
                std::end(this->keyframe_normals[k]) );
        }

        this->submap_cloud = submap_cloud_;
        std::swap(this->submap_kf_idx_prev, this->submap_kf_idx_curr);
    }
}

void PerceptionNode::LidarOdometry::pushSubmapIndices(const std::vector<float>& dists, int k, const std::vector<int>& frames)
{
    // make sure dists is not empty
    if(!dists.size()) return;

    const auto comp =
        [](const std::pair<float, int>& a, const std::pair<float, int>& b)
        {
            return a.first < b.first;
        };
    std::priority_queue<
        std::pair<float, int>,
        std::vector<std::pair<float, int>>,
        decltype(comp) > pq{ comp };

    for(size_t i = 0; i < dists.size(); i++)
    {
        if(static_cast<int>(pq.size()) >= k && pq.top().first > dists[i])
        {
            pq.pop();
            pq.emplace(dists[i], static_cast<int>(i));
        }
        else if(static_cast<int>(pq.size()) < k)
        {
            pq.emplace(dists[i], static_cast<int>(i));
        }
    }

    if(static_cast<int>(pq.size()) > k) throw std::logic_error("logic error in priority queue size!");

    for(int i = 0; i < k; i++)
    {
        this->submap_kf_idx_curr.insert(frames[pq.top().second]);
        pq.pop();
    }
}

void PerceptionNode::LidarOdometry::computeConvexHull()
{
    // at least 4 keyframes for convex hull
    if(this->state.num_keyframes < 4) return;

    // calculate the convex hull of the point cloud
    this->convex_hull.setInputCloud(this->keyframe_points);

    // get the indices of the keyframes on the convex hull
    // PointCloudType::Ptr convex_points = std::make_shared<PointCloudType>();
    this->convex_hull.reconstruct(this->scratch_cloud);

    pcl::PointIndices::Ptr convex_hull_point_idx = std::make_shared<pcl::PointIndices>();
    this->convex_hull.getHullPointIndices(*convex_hull_point_idx);

    std::swap(this->keyframe_convex, convex_hull_point_idx->indices);
}

void PerceptionNode::LidarOdometry::computeConcaveHull()
{
    // at least 5 keyframes for concave hull
    if(this->state.num_keyframes < 5) return;

    // calculate the concave hull of the point cloud
    this->concave_hull.setInputCloud(this->keyframe_points);

    // get the indices of the keyframes on the concave hull
    // PointCloudType::Ptr concave_points = std::make_shared<PointCloudType>();
    this->concave_hull.reconstruct(this->scratch_cloud);

    pcl::PointIndices::Ptr concave_hull_point_idx = std::make_shared<pcl::PointIndices>();
    this->concave_hull.getHullPointIndices(*concave_hull_point_idx);

    std::swap(this->keyframe_concave, concave_hull_point_idx->indices);
}

void PerceptionNode::LidarOdometry::propagateS2M()
{
    this->state.pose = this->state.T.block<3, 1>(0, 3);
    this->state.rotSO3 = this->state.T.block<3, 3>(0, 0);

    Eigen::Quaternionf q{ this->state.rotSO3 };
    q.normalize();

    this->state.rotq = q;

    // handle sign flip
    q = this->state.last_rotq.conjugate() * this->state.rotq;
    if(q.w() < 0)
    {
        this->state.rotq.w() = -this->state.rotq.w();
        this->state.rotq.vec() = -this->state.rotq.vec();
    }
    this->state.last_rotq = this->state.rotq;
}

void PerceptionNode::LidarOdometry::updateKeyframes()
{
    // transform point cloud
    this->transformCurrentScan();

    // calculate difference in pose and rotation to all poses in trajectory
    double closest_d = std::numeric_limits<double>::infinity();
    int closest_idx = 0;
    int keyframes_idx = 0;
    int num_nearby = 0;

    for(const auto& k : this->keyframes)
    {
        // calculate distance between current pose and pose in keyframes
        const double delta_d = (this->state.pose - k.first.first).norm();

        // count the number nearby current pose
        if(delta_d <= this->param.keyframe_thresh_dist_ * 1.5) num_nearby++;

        // store into variable
        if(delta_d < closest_d)
        {
            closest_d = delta_d;
            closest_idx = keyframes_idx;
        }

        keyframes_idx++;
    }

    // calculate difference in orientation
    double theta_rad = this->state.rotq.angularDistance(this->keyframes[closest_idx].first.second);
    double theta_deg = theta_rad * (180.0 / M_PI);

    // update keyframe
    bool newKeyframe = false;

    if(abs(closest_d) > this->param.keyframe_thresh_dist_)
    {
        newKeyframe = true;
    }
    else if(abs(theta_deg) > this->param.keyframe_thresh_rot_ && num_nearby <= 1)
    {
        newKeyframe = true;
    }

    if(newKeyframe)
    {

        ++this->state.num_keyframes;

        // voxelization for submap
        if(this->param.vf_submap_use_ && !this->param.adaptive_params_use_)
        {
            this->vf_submap.setInputCloud(this->current_scan_t);
            this->vf_submap.filter(*this->current_scan_t);
        }

        // update keyframe vector
        this->keyframes.emplace_back(
            std::make_pair(this->state.pose, this->state.rotq),
            this->current_scan_t );
        // pcl::PointXYZL pt;
        // pt.x = this->state.pose.x();
        // pt.y = this->state.pose.y();
        // pt.z = this->state.pose.z();
        // pt.label = this->keyframes.size() - 1;
        this->keyframe_points->emplace_back(
            this->state.pose.x(),
            this->state.pose.y(),
            this->state.pose.z() );
        // this->keyframe_points_kdtree.Add_Point(pt, false);

        // compute kdtree and keyframe normals (use gicp_s2s input source as temporary storage because it will be
        // overwritten by setInputSources())
        // *this->keyframes_cloud += *this->current_scan_t;
        *this->keyframe_cloud = *this->current_scan_t;

        this->gicp_s2s.setInputSource(this->keyframe_cloud);
        this->gicp_s2s.calculateSourceCovariances();
        this->keyframe_normals.push_back(this->gicp_s2s.getSourceCovariances());

        // this->publish_keyframe_thread = std::thread(&dlo::OdomNode::publishKeyframe, this);
        // this->publish_keyframe_thread.detach();
    }
}

void PerceptionNode::LidarOdometry::transformCurrentScan()
{
    this->current_scan_t = std::make_shared<PointCloudType>();
    pcl::transformPointCloud(*this->current_scan, *this->current_scan_t, this->state.T);
}

};
};
