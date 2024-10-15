#include "./perception.hpp"

#include <queue>

#include <pcl_conversions/pcl_conversions.h>


PerceptionNode::DLOdom::DLOdom(PerceptionNode * inst) :
    pnode{ inst }
{
    // RCLCPP_INFO(this->pnode->get_logger(), "DLO CONSTRUCTOR INIT");

    this->state.imu_mtx.lock();
    this->state.scan_mtx.lock();

    this->state.dlo_initialized = false;
    this->state.imu_calibrated = false;

    this->state.origin = Eigen::Vector3d(0., 0., 0.);

    this->state.T = Eigen::Matrix4d::Identity();
    this->state.T_s2s = Eigen::Matrix4d::Identity();
    this->state.T_s2s_prev = Eigen::Matrix4d::Identity();

    // this->state.pose_s2s = Eigen::Vector3d(0., 0., 0.);
    // this->state.rotq_s2s = Eigen::Quaterniond(1., 0., 0., 0.);

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

    this->source_cloud = nullptr;
    this->target_cloud = nullptr;
    this->export_scan = nullptr;
    this->current_scan = std::make_shared<pcl::PointCloud<PointType>>();
    this->current_scan_t = std::make_shared<pcl::PointCloud<PointType>>();

    this->keyframe_cloud = std::make_shared<pcl::PointCloud<PointType>>();
    // this->keyframes_cloud = std::make_shared<pcl::PointCloud<PointType>>();  // originally used for export
    this->state.num_keyframes = 0;

    this->submap_cloud = std::make_shared<pcl::PointCloud<PointType>>();
    this->state.submap_hasChanged = true;
    this->submap_kf_idx_prev.clear();

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

void PerceptionNode::DLOdom::getParams()
{
    if(!this->pnode) return;

    // Gravity alignment
    util::declare_param(this->pnode, "dlo.gravity_align", this->param.gravity_align_, false);

    // Keyframe Threshold
    util::declare_param(this->pnode, "dlo.keyframe.thresh_D", this->param.keyframe_thresh_dist_, 0.1);
    util::declare_param(this->pnode, "dlo.keyframe.thresh_R", this->param.keyframe_thresh_rot_, 1.0);

    // Submap
    util::declare_param(this->pnode, "dlo.submap.keyframe.knn", this->param.submap_knn_, 10);
    util::declare_param(this->pnode, "dlo.submap.keyframe.kcv", this->param.submap_kcv_, 10);
    util::declare_param(this->pnode, "dlo.submap.keyframe.kcc", this->param.submap_kcc_, 10);

    // Initial Position
    util::declare_param(this->pnode, "dlo.initial_pose.use", this->param.initial_pose_use_, false);

    std::vector<double> pos, quat;
    util::declare_param(this->pnode, "dlo.initial_pose.position", pos, {0., 0., 0.});
    util::declare_param(this->pnode, "dlo.initial_pose.orientation", quat, {1., 0., 0., 0.});
    this->param.initial_position_ = Eigen::Vector3d(pos[0], pos[1], pos[2]);
    this->param.initial_orientation_ = Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]);

    // Crop Box Filter
    util::declare_param(this->pnode, "dlo.preprocessing.crop_filter.use", this->param.crop_use_, false);
    std::vector<double> _min, _max;
    util::declare_param(this->pnode, "dlo.preprocessing.crop_filter.min", _min, {-1.0, -1.0, -1.0});
    util::declare_param(this->pnode, "dlo.preprocessing.crop_filter.max", _max, {1.0, 1.0, 1.0});
    this->param.crop_min_ = Eigen::Vector4f{(float)_min[0], (float)_min[1], (float)_min[2], 1.f};
    this->param.crop_max_ = Eigen::Vector4f{(float)_max[0], (float)_max[1], (float)_max[2], 1.f};

    // Voxel Grid Filter
    util::declare_param(this->pnode, "dlo.preprocessing.voxel_filter.scan.use", this->param.vf_scan_use_, true);
    util::declare_param(this->pnode, "dlo.preprocessing.voxel_filter.scan.res", this->param.vf_scan_res_, 0.05);
    util::declare_param(this->pnode, "dlo.preprocessing.voxel_filter.submap.use", this->param.vf_submap_use_,
                        false);
    util::declare_param(this->pnode, "dlo.preprocessing.voxel_filter.submap.res", this->param.vf_submap_res_,
                        0.1);

    // Adaptive Parameters
    util::declare_param(this->pnode, "dlo.adaptive_params", this->param.adaptive_params_use_, false);

    // IMU
    util::declare_param(this->pnode, "dlo.imu.use", this->param.imu_use_, false);
    util::declare_param(this->pnode, "dlo.imu.use_orientation", this->param.imu_use_orientation_, false);
    util::declare_param(this->pnode, "dlo.imu.calib_time", this->param.imu_calib_time_, 3);
    util::declare_param(this->pnode, "dlo.imu.buffer_size", this->param.imu_buffer_size_, 2000);

    // GICP
    util::declare_param(this->pnode, "dlo.gicp.min_num_points", this->param.gicp_min_num_points_, 100);
    util::declare_param(this->pnode, "dlo.gicp.s2s.k_correspondences", this->param.gicps2s_k_correspondences_,
                        20);
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
    util::declare_param(this->pnode, "dlo.gicp.s2m.k_correspondences", this->param.gicps2m_k_correspondences_,
                        20);
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


/** Returned integer contains status bits as well as the number of keyframes.
 * Bit 0 is set when new odometry was exported, bit 1 is set when the first keyframe is added,
 * bit 2 is set when a non-initial new keyframe is added, and the highest
 * 32 bits contain the (signed) number of keyframes. */
int64_t PerceptionNode::DLOdom::processScan(
    const sensor_msgs::msg::PointCloud2::SharedPtr& scan,
    pcl::PointCloud<PointType>::Ptr& filtered_scan,
    util::geom::PoseTf3d& odom_tf)
{
    std::unique_lock _lock{ this->state.scan_mtx };
    const int prev_num_keyframes = this->state.num_keyframes;

    // double then = this->pnode->now().seconds();
    // this->state.scan_stamp = scan->header.stamp;
    this->state.curr_frame_stamp = util::toFloatSeconds(scan->header.stamp);

    // DLO Initialization procedures (IMU calib, gravity align)
    if(!this->state.dlo_initialized)
    {
        this->initializeDLO();
        if(!this->state.dlo_initialized) return 0;  // uninitialized
    }

    // If there are too few points in the pointcloud, try again
    this->current_scan = std::make_shared<pcl::PointCloud<PointType>>();
    pcl::fromROSMsg(*scan, *this->current_scan);
    if((int64_t)this->current_scan->points.size() < this->param.gicp_min_num_points_)
    {
        RCLCPP_INFO(this->pnode->get_logger(), "DLO: Low number of points!");
        return 0;   // failure
    }

    // Preprocess points
    this->export_scan = filtered_scan;
    this->preprocessPoints();
    this->export_scan = nullptr;

    // Set Adaptive Parameters
    if(this->param.adaptive_params_use_)
    {
        this->setAdaptiveParams();
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
    this->source_cloud = std::make_shared<pcl::PointCloud<PointType>>();
    this->source_cloud = this->current_scan;

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

    return (1 << 0) |
        ((this->state.num_keyframes > prev_num_keyframes) << 2) |
        ((int64_t)this->state.num_keyframes << 32);
}

void PerceptionNode::DLOdom::processImu(const sensor_msgs::msg::Imu& imu)
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
    double ang_vel[3], lin_accel[3];

    // Get IMU samples
    ang_vel[0] = imu.angular_velocity.x;
    ang_vel[1] = imu.angular_velocity.y;
    ang_vel[2] = imu.angular_velocity.z;

    lin_accel[0] = imu.linear_acceleration.x;
    lin_accel[1] = imu.linear_acceleration.y;
    lin_accel[2] = imu.linear_acceleration.z;

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


void PerceptionNode::DLOdom::preprocessPoints()
{
    // Remove NaNs
    std::vector<int> idx;
    this->current_scan->is_dense = false;
    pcl::removeNaNFromPointCloud(*this->current_scan, *this->current_scan, idx);

    // Crop Box Filter
    if(this->param.crop_use_)
    {
        this->crop.setInputCloud(this->current_scan);
        this->crop.filter(*this->current_scan);
    }

    // Filtered "environment" scan for export
    if(this->export_scan) *this->export_scan = *this->current_scan;

    // Voxel Grid Filter
    if(this->param.vf_scan_use_)
    {
        this->vf_scan.setInputCloud(this->current_scan);
        this->vf_scan.filter(*this->current_scan);
    }
}

void PerceptionNode::DLOdom::initializeInputTarget()
{
    this->state.prev_frame_stamp = this->state.curr_frame_stamp;

    // Convert ros message
    this->target_cloud = std::make_shared<pcl::PointCloud<PointType>>();
    this->target_cloud = this->current_scan;
    this->gicp_s2s.setInputTarget(this->target_cloud);
    this->gicp_s2s.calculateTargetCovariances();

    // initialize keyframes
    pcl::PointCloud<PointType>::Ptr first_keyframe = std::make_shared<pcl::PointCloud<PointType>>();
    pcl::transformPointCloud(*this->target_cloud, *first_keyframe, this->state.T);

    // voxelization for submap
    if(this->param.vf_submap_use_)
    {
        this->vf_submap.setInputCloud(first_keyframe);
        this->vf_submap.filter(*first_keyframe);
    }

    // keep history of keyframes
    this->keyframes.push_back(std::make_pair(std::make_pair(this->state.pose, this->state.rotq), first_keyframe));
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

void PerceptionNode::DLOdom::setInputSources()
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

void PerceptionNode::DLOdom::initializeDLO()
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
        this->state.T.block(0, 3, 3, 1) = this->state.pose;
        this->state.T_s2s.block(0, 3, 3, 1) = this->state.pose;
        this->state.T_s2s_prev.block(0, 3, 3, 1) = this->state.pose;

        // set known orientation
        this->state.rotq = this->param.initial_orientation_;
        this->state.T.block(0, 0, 3, 3) = this->state.rotq.toRotationMatrix();  // TODO: one line?
        this->state.T_s2s.block(0, 0, 3, 3) = this->state.rotq.toRotationMatrix();
        this->state.T_s2s_prev.block(0, 0, 3, 3) = this->state.rotq.toRotationMatrix();

        // std::cout << "done" << std::endl << std::endl;
    }

    this->state.dlo_initialized = true;
    // std::cout << "DLO initialized! Starting localization..." << std::endl;   // TODO
}

void PerceptionNode::DLOdom::gravityAlign()
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
    this->state.T.block(0, 0, 3, 3) = this->state.rotq.toRotationMatrix();
    this->state.T_s2s.block(0, 0, 3, 3) = this->state.rotq.toRotationMatrix();
    this->state.T_s2s_prev.block(0, 0, 3, 3) = this->state.rotq.toRotationMatrix();

    // rpy
    // auto euler = grav_q.toRotationMatrix().eulerAngles(2, 1, 0);
    // double yaw = euler[0] * (180.0 / M_PI);
    // double pitch = euler[1] * (180.0 / M_PI);
    // double roll = euler[2] * (180.0 / M_PI);

    // std::cout << "done" << std::endl;
    // std::cout << "  Roll [deg]: " << roll << std::endl;
    // std::cout << "  Pitch [deg]: " << pitch << std::endl << std::endl;
}

void PerceptionNode::DLOdom::getNextPose()
{
    //
    // FRAME-TO-FRAME PROCEDURE
    //

    // Align using IMU prior if available
    pcl::PointCloud<PointType>::Ptr aligned = std::make_shared<pcl::PointCloud<PointType>>();

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
    *this->target_cloud = *this->source_cloud;
}

void PerceptionNode::DLOdom::integrateIMU()
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
    this->state.imu_SE3.block(0, 0, 3, 3) = q.toRotationMatrix();
}

void PerceptionNode::DLOdom::propagateS2S(const Eigen::Matrix4d& T)
{
    this->state.T_s2s = this->state.T_s2s_prev * T;
    this->state.T_s2s_prev = this->state.T_s2s;

    // this->state.pose_s2s << this->state.T_s2s(0, 3), this->state.T_s2s(1, 3), this->state.T_s2s(2, 3);
    // this->state.rotSO3_s2s << this->state.T_s2s(0, 0), this->state.T_s2s(0, 1), this->state.T_s2s(0, 2),
    //     this->state.T_s2s(1, 0), this->state.T_s2s(1, 1), this->state.T_s2s(1, 2), this->state.T_s2s(2, 0),
    //     this->state.T_s2s(2, 1), this->state.T_s2s(2, 2);

    // Eigen::Quaterniond q(this->state.rotSO3_s2s);

    // Normalize quaternion
    // double norm = sqrt(q.w() * q.w() + q.x() * q.x() + q.y() * q.y() + q.z() * q.z());
    // q.w() /= norm;
    // q.x() /= norm;
    // q.y() /= norm;
    // q.z() /= norm;
    // this->state.rotq_s2s = q;
}

void PerceptionNode::DLOdom::propagateS2M()
{
    this->state.pose << this->state.T(0, 3), this->state.T(1, 3), this->state.T(2, 3);
    this->state.rotSO3 << this->state.T(0, 0), this->state.T(0, 1), this->state.T(0, 2), this->state.T(1, 0), this->state.T(1, 1), this->state.T(1, 2),
        this->state.T(2, 0), this->state.T(2, 1), this->state.T(2, 2);

    Eigen::Quaterniond q(this->state.rotSO3);

    // Normalize quaternion
    double norm = sqrt(q.w() * q.w() + q.x() * q.x() + q.y() * q.y() + q.z() * q.z());
    q.w() /= norm;
    q.x() /= norm;
    q.y() /= norm;
    q.z() /= norm;
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

void PerceptionNode::DLOdom::setAdaptiveParams()
{
    // compute range of points "spaciousness"
    std::vector<float> ds;

    for(size_t i = 0; i <= this->current_scan->points.size(); i++)
    {
        float d = std::sqrt(pow(this->current_scan->points[i].x, 2) + pow(this->current_scan->points[i].y, 2) +
                            pow(this->current_scan->points[i].z, 2));
        ds.push_back(d);
    }

    // median
    std::nth_element(ds.begin(), ds.begin() + ds.size() / 2, ds.end());
    float median_curr = ds[ds.size() / 2];
    static float median_prev = median_curr;
    float median_lpf = 0.95 * median_prev + 0.05 * median_curr;
    median_prev = median_lpf;

    // Set Keyframe Thresh from Spaciousness Metric
    if(median_lpf > 20.0)
    {
        this->param.keyframe_thresh_dist_ = 10.0;
    }
    else if(median_lpf > 10.0)
    {
        this->param.keyframe_thresh_dist_ = 5.0;
    }
    else if(median_lpf > 5.0)
    {
        this->param.keyframe_thresh_dist_ = 1.0;
    }
    else if(median_lpf <= 5.0)
    {
        this->param.keyframe_thresh_dist_ = 0.5;
    }

    // set concave hull alpha
    this->concave_hull.setAlpha(this->param.keyframe_thresh_dist_);
}

void PerceptionNode::DLOdom::transformCurrentScan()
{
    this->current_scan_t = std::make_shared<pcl::PointCloud<PointType>>();
    pcl::transformPointCloud(*this->current_scan, *this->current_scan_t, this->state.T);
}

void PerceptionNode::DLOdom::updateKeyframes()
{
    // transform point cloud
    this->transformCurrentScan();

    // calculate difference in pose and rotation to all poses in trajectory
    float closest_d = std::numeric_limits<float>::infinity();
    int closest_idx = 0;
    int keyframes_idx = 0;

    int num_nearby = 0;

    for(const auto & k : this->keyframes)
    {

        // calculate distance between current pose and pose in keyframes
        float delta_d = sqrt(pow(this->state.pose[0] - k.first.first[0], 2) + pow(this->state.pose[1] - k.first.first[1], 2) +
                             pow(this->state.pose[2] - k.first.first[2], 2));

        // count the number nearby current pose
        if(delta_d <= this->param.keyframe_thresh_dist_ * 1.5)
        {
            ++num_nearby;
        }

        // store into variable
        if(delta_d < closest_d)
        {
            closest_d = delta_d;
            closest_idx = keyframes_idx;
        }

        keyframes_idx++;
    }

    // get closest pose and corresponding rotation
    Eigen::Vector3d closest_pose = this->keyframes[closest_idx].first.first;
    Eigen::Quaterniond closest_pose_r = this->keyframes[closest_idx].first.second;

    // calculate distance between current pose and closest pose from above
    float dd = sqrt(pow(this->state.pose[0] - closest_pose[0], 2) + pow(this->state.pose[1] - closest_pose[1], 2) +
                    pow(this->state.pose[2] - closest_pose[2], 2));

    // calculate difference in orientation
    Eigen::Quaterniond dq = this->state.rotq * (closest_pose_r.inverse());

    float theta_rad = 2. * atan2(sqrt(pow(dq.x(), 2) + pow(dq.y(), 2) + pow(dq.z(), 2)), dq.w());
    float theta_deg = theta_rad * (180.0 / M_PI);

    // update keyframe
    bool newKeyframe = false;

    if(abs(dd) > this->param.keyframe_thresh_dist_ || abs(theta_deg) > this->param.keyframe_thresh_rot_)
    {
        newKeyframe = true;
    }
    if(abs(dd) <= this->param.keyframe_thresh_dist_)
    {
        newKeyframe = false;
    }
    if(abs(dd) <= this->param.keyframe_thresh_dist_ && abs(theta_deg) > this->param.keyframe_thresh_rot_ && num_nearby <= 1)
    {
        newKeyframe = true;
    }

    if(newKeyframe)
    {

        ++this->state.num_keyframes;

        // voxelization for submap
        if(this->param.vf_submap_use_)
        {
            this->vf_submap.setInputCloud(this->current_scan_t);
            this->vf_submap.filter(*this->current_scan_t);
        }

        // update keyframe vector
        this->keyframes.push_back(std::make_pair(std::make_pair(this->state.pose, this->state.rotq), this->current_scan_t));

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

void PerceptionNode::DLOdom::computeConvexHull()
{
    // at least 4 keyframes for convex hull
    if(this->state.num_keyframes < 4)
    {
        return;
    }

    // create a pointcloud with points at keyframes
    pcl::PointCloud<PointType>::Ptr cloud = std::make_shared<pcl::PointCloud<PointType>>();

    for(const auto & k : this->keyframes)
    {
        PointType pt;
        pt.x = k.first.first[0];
        pt.y = k.first.first[1];
        pt.z = k.first.first[2];
        cloud->push_back(pt);
    }

    // calculate the convex hull of the point cloud
    this->convex_hull.setInputCloud(cloud);

    // get the indices of the keyframes on the convex hull
    pcl::PointCloud<PointType>::Ptr convex_points = std::make_shared<pcl::PointCloud<PointType>>();
    this->convex_hull.reconstruct(*convex_points);

    pcl::PointIndices::Ptr convex_hull_point_idx = std::make_shared<pcl::PointIndices>();
    this->convex_hull.getHullPointIndices(*convex_hull_point_idx);

    this->keyframe_convex.clear();
    for(size_t i = 0; i < convex_hull_point_idx->indices.size(); ++i)
    {
        this->keyframe_convex.push_back(convex_hull_point_idx->indices[i]);
    }
}

void PerceptionNode::DLOdom::computeConcaveHull()
{
    // at least 5 keyframes for concave hull
    if(this->state.num_keyframes < 5)
    {
        return;
    }

    // create a pointcloud with points at keyframes
    pcl::PointCloud<PointType>::Ptr cloud = std::make_shared<pcl::PointCloud<PointType>>();

    for(const auto & k : this->keyframes)
    {
        PointType pt;
        pt.x = k.first.first[0];
        pt.y = k.first.first[1];
        pt.z = k.first.first[2];
        cloud->push_back(pt);
    }

    // calculate the concave hull of the point cloud
    this->concave_hull.setInputCloud(cloud);

    // get the indices of the keyframes on the concave hull
    pcl::PointCloud<PointType>::Ptr concave_points = std::make_shared<pcl::PointCloud<PointType>>();
    this->concave_hull.reconstruct(*concave_points);

    pcl::PointIndices::Ptr concave_hull_point_idx = std::make_shared<pcl::PointIndices>();
    this->concave_hull.getHullPointIndices(*concave_hull_point_idx);

    this->keyframe_concave.clear();
    for(size_t i = 0; i < concave_hull_point_idx->indices.size(); ++i)
    {
        this->keyframe_concave.push_back(concave_hull_point_idx->indices[i]);
    }
}

void PerceptionNode::DLOdom::pushSubmapIndices(const std::vector<float>& dists, int k, const std::vector<int>& frames)
{
    // make sure dists is not empty
    if(!dists.size())
    {
        return;
    }

    // maintain max heap of at most k elements
    std::priority_queue<float> pq;

    for(const auto d : dists)
    {
        if((int64_t)pq.size() >= k && pq.top() > d)
        {
            pq.push(d);
            pq.pop();
        }
        else if((int64_t)pq.size() < k)
        {
            pq.push(d);
        }
    }

    // get the kth smallest element, which should be at the top of the heap
    float kth_element = pq.top();

    // get all elements smaller or equal to the kth smallest element
    for(size_t i = 0; i < dists.size(); ++i)
    {
        if(dists[i] <= kth_element)
            this->submap_kf_idx_curr.push_back(frames[i]);
    }
}

void PerceptionNode::DLOdom::getSubmapKeyframes()
{
    // clear vector of keyframe indices to use for submap
    this->submap_kf_idx_curr.clear();

    //
    // TOP K NEAREST NEIGHBORS FROM ALL KEYFRAMES
    //

    // calculate distance between current pose and poses in keyframe set
    std::vector<float> ds;
    std::vector<int> keyframe_nn;
    int i = 0;
    Eigen::Vector3d curr_pose = this->state.T_s2s.block(0, 3, 3, 1);

    for(const auto & k : this->keyframes)
    {
        float d = sqrt(pow(curr_pose[0] - k.first.first[0], 2) + pow(curr_pose[1] - k.first.first[1], 2) +
                       pow(curr_pose[2] - k.first.first[2], 2));
        ds.push_back(d);
        keyframe_nn.push_back(i);
        i++;
    }

    // get indices for top K nearest neighbor keyframe poses
    this->pushSubmapIndices(ds, this->param.submap_knn_, keyframe_nn);

    //
    // TOP K NEAREST NEIGHBORS FROM CONVEX HULL
    //

    // get convex hull indices
    this->computeConvexHull();

    // get distances for each keyframe on convex hull
    std::vector<float> convex_ds;
    for(const auto & c : this->keyframe_convex) { convex_ds.push_back(ds[c]); }

    // get indicies for top kNN for convex hull
    this->pushSubmapIndices(convex_ds, this->param.submap_kcv_, this->keyframe_convex);

    //
    // TOP K NEAREST NEIGHBORS FROM CONCAVE HULL
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
    std::sort(this->submap_kf_idx_curr.begin(), this->submap_kf_idx_curr.end());
    auto last = std::unique(this->submap_kf_idx_curr.begin(), this->submap_kf_idx_curr.end());
    this->submap_kf_idx_curr.erase(last, this->submap_kf_idx_curr.end());

    // sort current and previous submap kf list of indices
    std::sort(this->submap_kf_idx_curr.begin(), this->submap_kf_idx_curr.end());
    std::sort(this->submap_kf_idx_prev.begin(), this->submap_kf_idx_prev.end());

    // check if submap has changed from previous iteration
    if(this->submap_kf_idx_curr == this->submap_kf_idx_prev)
    {
        this->state.submap_hasChanged = false;
    }
    else
    {
        this->state.submap_hasChanged = true;

        // reinitialize submap cloud, normals
        pcl::PointCloud<PointType>::Ptr submap_cloud_(std::make_shared<pcl::PointCloud<PointType>>());
        this->submap_normals.clear();

        for(auto k : this->submap_kf_idx_curr)
        {

            // create current submap cloud
            *submap_cloud_ += *this->keyframes[k].second;

            // grab corresponding submap cloud's normals
            this->submap_normals.insert(std::end(this->submap_normals), std::begin(this->keyframe_normals[k]),
                                        std::end(this->keyframe_normals[k]));
        }

        this->submap_cloud = submap_cloud_;
        this->submap_kf_idx_prev = this->submap_kf_idx_curr;
    }
}
