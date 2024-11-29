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

    this->state.origin = Eigen::Vector3d(0., 0., 0.);

    this->state.T = Eigen::Matrix4d::Identity();
    this->state.T_s2s = Eigen::Matrix4d::Identity();
    this->state.T_s2s_prev = Eigen::Matrix4d::Identity();

    this->state.pose = Eigen::Vector3d(0., 0., 0.);
    this->state.rotq = Eigen::Quaterniond(1., 0., 0., 0.);

    this->state.imu_SE3 = Eigen::Matrix4d::Identity();

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
    this->filtered_scan = nullptr;
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

    this->crop.setNegative(true);
    this->crop.setMin(this->param.crop_min_);
    this->crop.setMax(this->param.crop_max_);

    this->vf_scan.setLeafSize(this->param.vf_scan_res_, this->param.vf_scan_res_, this->param.vf_scan_res_);
    this->vf_submap.setLeafSize(this->param.vf_submap_res_, this->param.vf_submap_res_, this->param.vf_submap_res_);

    this->state.imu_mtx.unlock();
    this->state.scan_mtx.unlock();

    // RCLCPP_INFO(this->pnode->get_logger(), "DLO CONSTRUCTOR EXIT");
}

void PerceptionNode::LidarOdometry::getParams()
{
    if(!this->pnode) return;

    // Debug
    util::declare_param(this->pnode, "dlo.debug.publish_scans", this->param.publish_debug_scans_, false);

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

    // Crop Box Filter
    util::declare_param(this->pnode, "dlo.crop_filter.use", this->param.crop_use_, false);
    std::vector<double> _min, _max;
    util::declare_param(this->pnode, "dlo.crop_filter.min", _min, {-1.0, -1.0, -1.0});
    util::declare_param(this->pnode, "dlo.crop_filter.max", _max, {1.0, 1.0, 1.0});
    this->param.crop_min_ = Eigen::Vector4f{(float)_min[0], (float)_min[1], (float)_min[2], 1.f};
    this->param.crop_max_ = Eigen::Vector4f{(float)_max[0], (float)_max[1], (float)_max[2], 1.f};

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


void PerceptionNode::LidarOdometry::publishDebugScans()
{
    // if(this->param.publish_debug_scans_)
    // {
    //     try
    //     {
    //         sensor_msgs::msg::PointCloud2 output;
    //         pcl::toROSMsg(*this->current_scan, output);
    //         output.header = scan->header;
    //         this->pnode->scan_pub.publish("dlo/voxelized_scan", output);

    //         pcl::toROSMsg(*this->submap_cloud, output);
    //         output.header.stamp = scan->header.stamp;
    //         output.header.frame_id = this->pnode->odom_frame;
    //         this->pnode->scan_pub.publish("dlo/submap_cloud", output);

    //         if(this->state.num_keyframes > prev_num_keyframes)
    //         {
    //             pcl::toROSMsg(*this->keyframe_cloud, output);
    //             output.header.stamp = scan->header.stamp;
    //             output.header.frame_id = this->pnode->odom_frame;
    //             this->pnode->scan_pub.publish("dlo/keyframe_cloud", output);
    //         }
    //     }
    //     catch(const std::exception& e)
    //     {
    //         RCLCPP_INFO(this->pnode->get_logger(), "[DLO]: Failed to publish debug scans -- what():\n\t%s", e.what());
    //     }
    // }
}


/** Returned integer contains status bits as well as the number of keyframes.
 * Bit 0 is set when new odometry was exported, bit 1 is set when the first keyframe is added,
 * bit 2 is set when a non-initial new keyframe is added, and the highest
 * 32 bits contain the (signed) number of keyframes. */
int64_t PerceptionNode::LidarOdometry::processScan(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan,
    util::geom::PoseTf3d& odom_tf,
    PointCloudType::Ptr& filtered_scan)
{
    std::unique_lock _lock{ this->state.scan_mtx };
    const int prev_num_keyframes = this->state.num_keyframes;

    this->state.curr_frame_stamp = util::toFloatSeconds(scan->header.stamp);

    // DLO Initialization procedures (IMU calib, gravity align)
    if(!this->state.dlo_initialized)
    {
        this->initializeDLO();
        if(!this->state.dlo_initialized) return 0;  // uninitialized
    }

    this->current_scan = std::make_shared<PointCloudType>();
    pcl::fromROSMsg(*scan, *this->current_scan);

    this->preprocessPoints();
    if(filtered_scan) std::swap(filtered_scan, this->filtered_scan);

    // Exit if insufficient points
    if((int64_t)this->current_scan->points.size() < this->param.gicp_min_num_points_)
    {
        RCLCPP_INFO(this->pnode->get_logger(), "[DLO]: Post-processed cloud does not have enough points!");
        return 0;   // failure
    }

    // Set initial frame as target
    if(this->target_cloud == nullptr)
    {
        this->initializeInputTarget();

        odom_tf.pose.vec = this->state.pose;
        odom_tf.pose.quat = this->state.rotq;
        odom_tf.tf = this->state.T;

        return (1 << 0) |
            (1 << 1) |
            ((int64_t)this->state.num_keyframes << 32);
        // ^ exported new odom and has new (initial) keyframe, append number of keyframes
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

    if(this->param.publish_debug_scans_)
    {
        try
        {
            sensor_msgs::msg::PointCloud2 output;
            pcl::toROSMsg(*this->current_scan, output);
            output.header = scan->header;
            this->pnode->scan_pub.publish("dlo/voxelized_scan", output);

            pcl::toROSMsg(*this->submap_cloud, output);
            output.header.stamp = scan->header.stamp;
            output.header.frame_id = this->pnode->odom_frame;
            this->pnode->scan_pub.publish("dlo/submap_cloud", output);

            if(this->state.num_keyframes > prev_num_keyframes)
            {
                pcl::toROSMsg(*this->keyframe_cloud, output);
                output.header.stamp = scan->header.stamp;
                output.header.frame_id = this->pnode->odom_frame;
                this->pnode->scan_pub.publish("dlo/keyframe_cloud", output);
            }
        }
        catch(const std::exception& e)
        {
            RCLCPP_INFO(this->pnode->get_logger(), "[DLO]: Failed to publish debug scans -- what():\n\t%s", e.what());
        }
    }

    return (1 << 0) |
        ((this->state.num_keyframes > prev_num_keyframes) << 2) |
        ((int64_t)this->state.num_keyframes << 32);
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
        this->orient_buffer.emplace_front(stamp, q);
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
        this->state.pose = this->param.initial_position_;
        this->state.origin = this->param.initial_position_;
        this->state.T.block<3, 1>(0, 3) = this->state.pose;
        this->state.T_s2s.block<3, 1>(0, 3) = this->state.pose;
        this->state.T_s2s_prev.block<3, 1>(0, 3) = this->state.pose;

        // set known orientation
        this->state.rotq = this->param.initial_orientation_;
        this->state.T.block<3, 3>(0, 0) = this->state.rotq.toRotationMatrix();  // TODO: one line?
        this->state.T_s2s.block<3, 3>(0, 0) = this->state.rotq.toRotationMatrix();
        this->state.T_s2s_prev.block<3, 3>(0, 0) = this->state.rotq.toRotationMatrix();

        // std::cout << "done" << std::endl << std::endl;
    }

    this->state.dlo_initialized = true;
    // std::cout << "DLO initialized! Starting localization..." << std::endl;   // TODO
}

void PerceptionNode::LidarOdometry::gravityAlign()
{
    // get average acceleration vector for 1 second and normalize
    Eigen::Vector3d lin_accel = Eigen::Vector3d::Zero();
    const double then = this->pnode->now().seconds();
    int n = 0;
    while((this->pnode->now().seconds() - then) < 1.)
    {
        lin_accel[0] += this->state.imu_meas.lin_accel.x;
        lin_accel[1] += this->state.imu_meas.lin_accel.y;
        lin_accel[2] += this->state.imu_meas.lin_accel.z;
        ++n;
    }
    lin_accel[0] /= n;
    lin_accel[1] /= n;
    lin_accel[2] /= n;

    // normalize
    double lin_norm = sqrt(pow(lin_accel[0], 2) + pow(lin_accel[1], 2) + pow(lin_accel[2], 2));
    lin_accel[0] /= lin_norm;
    lin_accel[1] /= lin_norm;
    lin_accel[2] /= lin_norm;

    // define gravity vector (assume point downwards)
    Eigen::Vector3d grav;
    grav << 0, 0, 1;

    // calculate angle between the two vectors
    Eigen::Quaterniond grav_q = Eigen::Quaterniond::FromTwoVectors(lin_accel, grav);

    // normalize
    double grav_norm =
        sqrt(grav_q.w() * grav_q.w() + grav_q.x() * grav_q.x() + grav_q.y() * grav_q.y() + grav_q.z() * grav_q.z());
    grav_q.w() /= grav_norm;
    grav_q.x() /= grav_norm;
    grav_q.y() /= grav_norm;
    grav_q.z() /= grav_norm;

    // set gravity aligned orientation
    this->state.rotq = grav_q;
    this->state.T.block<3, 3>(0, 0) = this->state.rotq.toRotationMatrix();
    this->state.T_s2s.block<3, 3>(0, 0) = this->state.rotq.toRotationMatrix();
    this->state.T_s2s_prev.block<3, 3>(0, 0) = this->state.rotq.toRotationMatrix();

    // rpy
    // auto euler = grav_q.toRotationMatrix().eulerAngles(2, 1, 0);
    // double yaw = euler[0] * (180.0 / M_PI);
    // double pitch = euler[1] * (180.0 / M_PI);
    // double roll = euler[2] * (180.0 / M_PI);

    // std::cout << "done" << std::endl;
    // std::cout << "  Roll [deg]: " << roll << std::endl;
    // std::cout << "  Pitch [deg]: " << pitch << std::endl << std::endl;
}

void PerceptionNode::LidarOdometry::preprocessPoints()
{
    // Remove NaNs
    std::vector<int> idx;
    this->current_scan->is_dense = false;
    if(!this->filtered_scan) this->filtered_scan = std::make_shared<PointCloudType>();
    pcl::removeNaNFromPointCloud(*this->current_scan, *this->filtered_scan, idx);

    // Crop Box Filter
    if(this->param.crop_use_)
    {
        this->crop.setInputCloud(this->filtered_scan);  // interesting crash here
        this->crop.filter(*this->filtered_scan);
    }

    // Don't bother continuing if not enough points
    if((int64_t)this->filtered_scan->points.size() < this->param.gicp_min_num_points_)
    {
        return;
    }

    // Find new voxel size before applying filter
    if(this->param.adaptive_params_use_)
    {
        this->setAdaptiveParams();
    }

    // Voxel Grid Filter
    if(this->param.vf_scan_use_)
    {
        this->vf_scan.setInputCloud(this->filtered_scan);
        this->vf_scan.filter(*this->current_scan);
    }
    else
    {
        *this->current_scan = *this->filtered_scan;
    }
}

void PerceptionNode::LidarOdometry::setAdaptiveParams()
{
    // compute range of points "spaciousness"
    const size_t n_points = this->filtered_scan->points.size();
    constexpr static size_t DOWNSAMPLE_SHIFT = 5;
    thread_local std::vector<double> ds;
    double avg = 0.;
    ds.clear();
    ds.reserve(n_points >> DOWNSAMPLE_SHIFT);

    for(size_t i = 0; i < n_points >> DOWNSAMPLE_SHIFT; i++)
    {
        auto& p = this->filtered_scan->points[i << DOWNSAMPLE_SHIFT];
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
    this->keyframes.push_back(std::make_pair(std::make_pair(this->state.pose, this->state.rotq), first_keyframe));
    // pcl::PointXYZL pt;
    // pt.x = this->state.pose.x();
    // pt.y = this->state.pose.y();
    // pt.z = this->state.pose.z();
    // pt.label = this->keyframes.size() - 1;
    this->keyframe_points->emplace_back(
        (float)this->state.pose.x(),
        (float)this->state.pose.y(),
        (float)this->state.pose.z() );
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
    thread_local PointCloudType::Ptr aligned = std::make_shared<PointCloudType>();

    // RCLCPP_INFO(this->pnode->get_logger(), "DLO: SCAN PROCESSING EXHIBIT E-1");
    if(this->param.imu_use_)
    {
        this->integrateIMU();
    // RCLCPP_INFO(this->pnode->get_logger(), "DLO: SCAN PROCESSING EXHIBIT E-2a");
        this->gicp_s2s.align(*aligned, this->state.imu_SE3.template cast<float>());
    }
    else
    {
        this->gicp_s2s.align(*aligned);
    // RCLCPP_INFO(this->pnode->get_logger(), "DLO: SCAN PROCESSING EXHIBIT E-2b");
    }
    // RCLCPP_INFO(this->pnode->get_logger(), "DLO: SCAN PROCESSING EXHIBIT E-3");

    // Get the local S2S transform
    Eigen::Matrix4d T_S2S = this->gicp_s2s.getFinalTransformation().template cast<double>();

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
    this->gicp.align(*aligned, this->state.T_s2s.template cast<float>());

    // RCLCPP_INFO(this->pnode->get_logger(), "DLO: SCAN PROCESSING EXHIBIT E-9");

    // Get final transformation in global frame
    this->state.T = this->gicp.getFinalTransformation().template cast<double>();

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

    if(this->param.imu_use_orientation_)
    {
        this->state.imu_mtx.lock();
        const size_t
            curr_idx = util::tsq::binarySearchIdx(this->orient_buffer, this->state.curr_frame_stamp),
            prev_idx = util::tsq::binarySearchIdx(this->orient_buffer, this->state.prev_frame_stamp);

        Eigen::Quaterniond
            prev_rotation = Eigen::Quaterniond::Identity(),
            curr_rotation = Eigen::Quaterniond::Identity();

        // get interpolated current imu pose
        if(curr_idx == 0)
        {
            curr_rotation = this->orient_buffer[0].second;
        }
        else if(curr_idx == this->orient_buffer.size())
        {
            curr_rotation = this->orient_buffer.back().second;
        }
        else if(util::tsq::validLerpIdx(this->orient_buffer, curr_idx))
        {
            const auto&
                pre = this->orient_buffer[curr_idx],
                post = this->orient_buffer[curr_idx - 1];
            curr_rotation = pre.second.slerp(
                ((this->state.curr_frame_stamp - pre.first) / (post.first - pre.first)), post.second);
        }

        // get interpolated previous pose
        if(prev_idx == 0)
        {
            prev_rotation = this->orient_buffer[0].second;
        }
        else if(prev_idx == this->orient_buffer.size())
        {
            prev_rotation = this->orient_buffer.back().second;
        }
        else if(util::tsq::validLerpIdx(this->orient_buffer, prev_idx))
        {
            const auto&
                pre = this->orient_buffer[prev_idx],
                post = this->orient_buffer[prev_idx - 1];
            prev_rotation = pre.second.slerp(
                ((this->state.prev_frame_stamp - pre.first) / (post.first - pre.first)), post.second);
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
            double curr_frame_imu_dt = this->state.curr_frame_stamp - i.stamp;
            double prev_frame_imu_dt = this->state.prev_frame_stamp - i.stamp;

            if(curr_frame_imu_dt >= 0. && prev_frame_imu_dt <= 0.)
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

        // Normalize quaternion
        double norm = sqrt(q.w() * q.w() + q.x() * q.x() + q.y() * q.y() + q.z() * q.z());
        q.w() /= norm;
        q.x() /= norm;
        q.y() /= norm;
        q.z() /= norm;
    }

    // Store IMU guess
    this->state.imu_SE3 = Eigen::Matrix4d::Identity();
    this->state.imu_SE3.block<3, 3>(0, 0) = q.toRotationMatrix();
}

void PerceptionNode::LidarOdometry::propagateS2S(const Eigen::Matrix4d& T)
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
    Eigen::Vector3d curr_pose = this->state.T_s2s.block<3, 1>(0, 3);

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
            this->submap_normals.insert(std::end(this->submap_normals), std::begin(this->keyframe_normals[k]),
                                        std::end(this->keyframe_normals[k]));
        }

        this->submap_cloud = submap_cloud_;
        std::swap(this->submap_kf_idx_prev, this->submap_kf_idx_curr);
    }
}

void PerceptionNode::LidarOdometry::pushSubmapIndices(const std::vector<float>& dists, int k, const std::vector<int>& frames)
{
    // make sure dists is not empty
    if(!dists.size())
    {
        return;
    }

    const auto comp = [](const std::pair<float, int>& a, const std::pair<float, int>& b){ return a.first < b.first; };
    std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>, decltype(comp)> pq{ comp };
    for(size_t i = 0; i < dists.size(); i++)
    {
        if((int)pq.size() >= k && pq.top().first > dists[i])
        {
            pq.pop();
            pq.emplace(dists[i], static_cast<int>(i));
        }
        else if((int)pq.size() < k)
        {
            pq.emplace(dists[i], static_cast<int>(i));
        }
    }

    if((int)pq.size() > k) throw std::logic_error("logic error in priority queue size!");

    for(int i = 0; i < k; i++)
    {
        this->submap_kf_idx_curr.insert(frames[pq.top().second]);
        pq.pop();
    }
}

void PerceptionNode::LidarOdometry::computeConvexHull()
{
    // at least 4 keyframes for convex hull
    if(this->state.num_keyframes < 4)
    {
        return;
    }

    // calculate the convex hull of the point cloud
    this->convex_hull.setInputCloud(this->keyframe_points);

    // get the indices of the keyframes on the convex hull
    PointCloudType::Ptr convex_points = std::make_shared<PointCloudType>();
    this->convex_hull.reconstruct(*convex_points);

    pcl::PointIndices::Ptr convex_hull_point_idx = std::make_shared<pcl::PointIndices>();
    this->convex_hull.getHullPointIndices(*convex_hull_point_idx);

    std::swap(this->keyframe_convex, convex_hull_point_idx->indices);
}

void PerceptionNode::LidarOdometry::computeConcaveHull()
{
    // at least 5 keyframes for concave hull
    if(this->state.num_keyframes < 5)
    {
        return;
    }

    // calculate the concave hull of the point cloud
    this->concave_hull.setInputCloud(this->keyframe_points);

    // get the indices of the keyframes on the concave hull
    PointCloudType::Ptr concave_points = std::make_shared<PointCloudType>();
    this->concave_hull.reconstruct(*concave_points);

    pcl::PointIndices::Ptr concave_hull_point_idx = std::make_shared<pcl::PointIndices>();
    this->concave_hull.getHullPointIndices(*concave_hull_point_idx);

    std::swap(this->keyframe_concave, concave_hull_point_idx->indices);
}

void PerceptionNode::LidarOdometry::propagateS2M()
{
    this->state.pose << this->state.T(0, 3), this->state.T(1, 3), this->state.T(2, 3);
    this->state.rotSO3 << this->state.T(0, 0), this->state.T(0, 1), this->state.T(0, 2), this->state.T(1, 0), this->state.T(1, 1), this->state.T(1, 2),
        this->state.T(2, 0), this->state.T(2, 1), this->state.T(2, 2);

    Eigen::Quaterniond q(this->state.rotSO3);
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
        this->keyframes.push_back(std::make_pair(std::make_pair(this->state.pose, this->state.rotq), this->current_scan_t));
        // pcl::PointXYZL pt;
        // pt.x = this->state.pose.x();
        // pt.y = this->state.pose.y();
        // pt.z = this->state.pose.z();
        // pt.label = this->keyframes.size() - 1;
        this->keyframe_points->emplace_back(
            (float)this->state.pose.x(),
            (float)this->state.pose.y(),
            (float)this->state.pose.z() );
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
