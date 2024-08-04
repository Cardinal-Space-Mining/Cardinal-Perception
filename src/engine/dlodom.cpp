#include "perception.hpp"


DLOdom::DLOdom(PerceptionNode * inst) :
    pnode{ inst }
{
    this->dlo_initialized = false;
    this->imu_calibrated = false;

    this->origin = Eigen::Vector3f(0., 0., 0.);

    this->T = Eigen::Matrix4f::Identity();
    this->T_s2s = Eigen::Matrix4f::Identity();
    this->T_s2s_prev = Eigen::Matrix4f::Identity();

    this->pose_s2s = Eigen::Vector3f(0., 0., 0.);
    this->rotq_s2s = Eigen::Quaternionf(1., 0., 0., 0.);

    this->pose = Eigen::Vector3f(0., 0., 0.);
    this->rotq = Eigen::Quaternionf(1., 0., 0., 0.);

    this->imu_SE3 = Eigen::Matrix4f::Identity();

    this->imu_bias.gyro.x = 0.;
    this->imu_bias.gyro.y = 0.;
    this->imu_bias.gyro.z = 0.;
    this->imu_bias.accel.x = 0.;
    this->imu_bias.accel.y = 0.;
    this->imu_bias.accel.z = 0.;

    this->imu_meas.stamp = 0.;
    this->imu_meas.ang_vel.x = 0.;
    this->imu_meas.ang_vel.y = 0.;
    this->imu_meas.ang_vel.z = 0.;
    this->imu_meas.lin_accel.x = 0.;
    this->imu_meas.lin_accel.y = 0.;
    this->imu_meas.lin_accel.z = 0.;

    this->getParams();

    this->imu_buffer.set_capacity(this->param.imu_buffer_size_);
    this->first_imu_time = 0.;

    this->export_scan = std::make_shared<pcl::PointCloud<PointType>>();
    this->current_scan = std::make_shared<pcl::PointCloud<PointType>>();
    this->current_scan_t = std::make_shared<pcl::PointCloud<PointType>>();

    this->keyframe_cloud = std::make_shared<pcl::PointCloud<PointType>>();
    this->keyframes_cloud = std::make_shared<pcl::PointCloud<PointType>>();
    this->num_keyframes = 0;

    this->submap_cloud = std::make_shared<pcl::PointCloud<PointType>>();
    this->submap_hasChanged = true;
    this->submap_kf_idx_prev.clear();

    this->source_cloud = nullptr;
    this->target_cloud = nullptr;

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

    this->spaciousness.push_back(0.);
}

void DLOdom::getParams()
{
    if(!this->pnode) return;

    // Gravity alignment
    util::declare_param(this->pnode, "dlo.gravityAlign", this->param.gravity_align_, false);

    // Keyframe Threshold
    util::declare_param(this->pnode, "dlo.keyframe.threshD", this->param.keyframe_thresh_dist_, 0.1);
    util::declare_param(this->pnode, "dlo.keyframe.threshR", this->param.keyframe_thresh_rot_, 1.0);

    // Submap
    util::declare_param(this->pnode, "dlo.submap.keyframe.knn", this->param.submap_knn_, 10);
    util::declare_param(this->pnode, "dlo.submap.keyframe.kcv", this->param.submap_kcv_, 10);
    util::declare_param(this->pnode, "dlo.submap.keyframe.kcc", this->param.submap_kcc_, 10);

    // Initial Position
    util::declare_param(this->pnode, "dlo.initialPose.use", this->param.initial_pose_use_, false);

    std::vector<double> pos, quat;
    util::declare_param(this->pnode, "dlo.initialPose.position", pos, {0., 0., 0.});
    util::declare_param(this->pnode, "dlo.initialPose.orientation", quat, {1., 0., 0., 0.});
    this->param.initial_position_ = Eigen::Vector3f((float)pos[0], (float)pos[1], (float)pos[2]);
    this->param.initial_orientation_ =
        Eigen::Quaternionf((float)quat[0], (float)quat[1], (float)quat[2], (float)quat[3]);

    // Crop Box Filter
    util::declare_param(this->pnode, "dlo.preprocessing.cropBoxFilter.use", this->param.crop_use_, false);
    std::vector<double> _min, _max;
    util::declare_param(this->pnode, "dlo.preprocessing.cropBoxFilter.min", _min, {-1.0, -1.0, -1.0});
    util::declare_param(this->pnode, "dlo.preprocessing.cropBoxFilter.max", _max, {1.0, 1.0, 1.0});
    this->param.crop_min_ = Eigen::Vector4f{(float)_min[0], (float)_min[1], (float)_min[2], 1.f};
    this->param.crop_max_ = Eigen::Vector4f{(float)_max[0], (float)_max[1], (float)_max[2], 1.f};

    // Voxel Grid Filter
    util::declare_param(this->pnode, "dlo.preprocessing.voxelFilter.scan.use", this->param.vf_scan_use_, true);
    util::declare_param(this->pnode, "dlo.preprocessing.voxelFilter.scan.res", this->param.vf_scan_res_, 0.05);
    util::declare_param(this->pnode, "dlo.preprocessing.voxelFilter.submap.use", this->param.vf_submap_use_,
                        false);
    util::declare_param(this->pnode, "dlo.preprocessing.voxelFilter.submap.res", this->param.vf_submap_res_,
                        0.1);

    // Adaptive Parameters
    util::declare_param(this->pnode, "dlo.adaptiveParams", this->param.adaptive_params_use_, false);

    // IMU
    util::declare_param(this->pnode, "dlo.imu", this->param.imu_use_, false);
    util::declare_param(this->pnode, "dlo.imu.calibTime", this->param.imu_calib_time_, 3);
    util::declare_param(this->pnode, "dlo.imu.bufferSize", this->param.imu_buffer_size_, 2000);

    // GICP
    util::declare_param(this->pnode, "dlo.gicp.minNumPoints", this->param.gicp_min_num_points_, 100);
    util::declare_param(this->pnode, "dlo.gicp.s2s.kCorrespondences", this->param.gicps2s_k_correspondences_,
                        20);
    util::declare_param(this->pnode, "dlo.gicp.s2s.maxCorrespondenceDistance",
                        this->param.gicps2s_max_corr_dist_, std::sqrt(std::numeric_limits<double>::max()));
    util::declare_param(this->pnode, "dlo.gicp.s2s.maxIterations", this->param.gicps2s_max_iter_, 64);
    util::declare_param(this->pnode, "dlo.gicp.s2s.transformationEpsilon",
                        this->param.gicps2s_transformation_ep_, 0.0005);
    util::declare_param(this->pnode, "dlo.gicp.s2s.euclideanFitnessEpsilon",
                        this->param.gicps2s_euclidean_fitness_ep_, -std::numeric_limits<double>::max());
    util::declare_param(this->pnode, "dlo.gicp.s2s.ransac.iterations", this->param.gicps2s_ransac_iter_, 0);
    util::declare_param(this->pnode, "dlo.gicp.s2s.ransac.outlierRejectionThresh",
                        this->param.gicps2s_ransac_inlier_thresh_, 0.05);
    util::declare_param(this->pnode, "dlo.gicp.s2m.kCorrespondences", this->param.gicps2m_k_correspondences_,
                        20);
    util::declare_param(this->pnode, "dlo.gicp.s2m.maxCorrespondenceDistance",
                        this->param.gicps2m_max_corr_dist_, std::sqrt(std::numeric_limits<double>::max()));
    util::declare_param(this->pnode, "dlo.gicp.s2m.maxIterations", this->param.gicps2m_max_iter_, 64);
    util::declare_param(this->pnode, "dlo.gicp.s2m.transformationEpsilon",
                        this->param.gicps2m_transformation_ep_, 0.0005);
    util::declare_param(this->pnode, "dlo.gicp.s2m.euclideanFitnessEpsilon",
                        this->param.gicps2m_euclidean_fitness_ep_, -std::numeric_limits<double>::max());
    util::declare_param(this->pnode, "dlo.gicp.s2m.ransac.iterations", this->param.gicps2m_ransac_iter_, 0);
    util::declare_param(this->pnode, "dlo.gicp.s2m.ransac.outlierRejectionThresh",
                        this->param.gicps2m_ransac_inlier_thresh_, 0.05);
}


void DLOdom::processScan(const sensor_msgs::msg::PointCloud2::SharedPtr& scan)
{
    double then = this->now().seconds();
    this->scan_stamp = scan->header.stamp;
    this->curr_frame_stamp = rclcpp::Time(scan->header.stamp).seconds();

    // If there are too few points in the pointcloud, try again
    this->current_scan = std::make_shared<pcl::PointCloud<PointType>>();
    pcl::fromROSMsg(*scan, *this->current_scan);
    if(this->current_scan->points.size() < this->param.gicp_min_num_points_)
    {
        // RCLCPP_FATAL(this->get_logger(), "Low number of points!");
        return;
    }

    // DLO Initialization procedures (IMU calib, gravity align)
    if(!this->dlo_initialized)
    {
        this->initializeDLO();
        return;
    }

    // Preprocess points
    this->preprocessPoints();

    // Compute Metrics
    // this->metrics_thread = std::thread(&dlo::OdomNode::computeMetrics, this);
    // this->metrics_thread.detach();

    // Set Adaptive Parameters
    if(this->param.adaptive_params_use_)
    {
        this->setAdaptiveParams();
    }

    // Set initial frame as target
    if(this->target_cloud == nullptr)
    {
        this->initializeInputTarget();
        return;
    }

    // Set source frame
    // this->source_cloud = std::make_shared<pcl::PointCloud<PointType>>();
    this->source_cloud = this->current_scan;

    // Set new frame as input source for both gicp objects
    this->setInputSources();

    // Get the next pose via IMU + S2S + S2M
    this->getNextPose();

    // Update current keyframe poses and map
    this->updateKeyframes();

    // Update trajectory
    this->trajectory.push_back(std::make_pair(this->pose, this->rotq));

    // Update next time stamp
    this->prev_frame_stamp = this->curr_frame_stamp;

    // Update some statistics
    this->comp_times.push_back(this->now().seconds() - then);

    // Publish stuff to ROS
    // this->publish_thread = std::thread(&dlo::OdomNode::publishToROS, this);
    // this->publish_thread.detach();

    // Debug statements and publish custom DLO message
    // this->debug_thread = std::thread(&dlo::OdomNode::debug, this);
    // this->debug_thread.detach();
}

void DLOdom::processImu(const sensor_msgs::msg::Imu::SharedPtr& imu)
{
    if(!this->param.imu_use_)
    {
        return;
    }

    double ang_vel[3], lin_accel[3];

    // Get IMU samples
    ang_vel[0] = imu->angular_velocity.x;
    ang_vel[1] = imu->angular_velocity.y;
    ang_vel[2] = imu->angular_velocity.z;

    lin_accel[0] = imu->linear_acceleration.x;
    lin_accel[1] = imu->linear_acceleration.y;
    lin_accel[2] = imu->linear_acceleration.z;

    if(this->first_imu_time == 0.)
    {
        this->first_imu_time = rclcpp::Time(imu->header.stamp).seconds();
    }

    // IMU calibration procedure - do for three seconds
    if(!this->imu_calibrated)
    {

        static int num_samples = 0;
        // static bool print = true;

        if((rclcpp::Time(imu->header.stamp).seconds() - this->first_imu_time) < this->param.imu_calib_time_)
        {

            num_samples++;

            this->imu_bias.gyro.x += ang_vel[0];
            this->imu_bias.gyro.y += ang_vel[1];
            this->imu_bias.gyro.z += ang_vel[2];

            this->imu_bias.accel.x += lin_accel[0];
            this->imu_bias.accel.y += lin_accel[1];
            this->imu_bias.accel.z += lin_accel[2];

            // if(print)
            // {
            //     std::cout << "Calibrating IMU for " << this->imu_calib_time_ << " seconds... ";
            //     std::cout.flush();
            //     print = false;
            // }
        }
        else
        {

            this->imu_bias.gyro.x /= num_samples;
            this->imu_bias.gyro.y /= num_samples;
            this->imu_bias.gyro.z /= num_samples;

            this->imu_bias.accel.x /= num_samples;
            this->imu_bias.accel.y /= num_samples;
            this->imu_bias.accel.z /= num_samples;

            this->imu_calibrated = true;

            // std::cout << "done" << std::endl;
            // std::cout << "  Gyro biases [xyz]: " << this->imu_bias.gyro.x << ", " << this->imu_bias.gyro.y << ", "
            //           << this->imu_bias.gyro.z << std::endl
            //           << std::endl;
        }
    }
    else
    {

        // Apply the calibrated bias to the new IMU measurements
        this->imu_meas.stamp = rclcpp::Time(imu->header.stamp).seconds();

        this->imu_meas.ang_vel.x = ang_vel[0] - this->imu_bias.gyro.x;
        this->imu_meas.ang_vel.y = ang_vel[1] - this->imu_bias.gyro.y;
        this->imu_meas.ang_vel.z = ang_vel[2] - this->imu_bias.gyro.z;

        this->imu_meas.lin_accel.x = lin_accel[0];
        this->imu_meas.lin_accel.y = lin_accel[1];
        this->imu_meas.lin_accel.z = lin_accel[2];

        // Store into circular buffer
        this->mtx_imu.lock();
        this->imu_buffer.push_front(this->imu_meas);
        this->mtx_imu.unlock();
    }
}


void DLOdom::preprocessPoints()
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
    *this->export_scan = pcl::PointCloud<PointType>{*this->current_scan};

    // Voxel Grid Filter
    if(this->param.vf_scan_use_)
    {
        this->vf_scan.setInputCloud(this->current_scan);
        this->vf_scan.filter(*this->current_scan);
    }
}

void DLOdom::initializeInputTarget()
{
    this->prev_frame_stamp = this->curr_frame_stamp;

    // Convert ros message
    // this->target_cloud = std::make_shared<pcl::PointCloud<PointType>>();
    this->target_cloud = this->current_scan;
    this->gicp_s2s.setInputTarget(this->target_cloud);
    this->gicp_s2s.calculateTargetCovariances();

    // initialize keyframes
    pcl::PointCloud<PointType>::Ptr first_keyframe = std::make_shared<pcl::PointCloud<PointType>>();
    pcl::transformPointCloud(*this->target_cloud, *first_keyframe, this->T);

    // voxelization for submap
    if(this->param.vf_submap_use_)
    {
        this->vf_submap.setInputCloud(first_keyframe);
        this->vf_submap.filter(*first_keyframe);
    }

    // keep history of keyframes
    this->keyframes.push_back(std::make_pair(std::make_pair(this->pose, this->rotq), first_keyframe));
    *this->keyframes_cloud += *first_keyframe;
    *this->keyframe_cloud = *first_keyframe;

    // compute kdtree and keyframe normals (use gicp_s2s input source as temporary storage because it will be
    // overwritten by setInputSources())
    this->gicp_s2s.setInputSource(this->keyframe_cloud);
    this->gicp_s2s.calculateSourceCovariances();
    this->keyframe_normals.push_back(this->gicp_s2s.getSourceCovariances());

    // this->publish_keyframe_thread = std::thread(&OdomNode::publishKeyframe, this);
    // this->publish_keyframe_thread.detach();

    ++this->num_keyframes;
}

void DLOdom::setInputSources()
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

void DLOdom::initializeDLO()
{
    // Calibrate IMU
    if(!this->imu_calibrated && this->param.imu_use_)
    {
        return;
    }

    // Gravity Align
    if(this->param.gravity_align_ && this->param.imu_use_ &&
        this->imu_calibrated && !this->param.initial_pose_use_)
    {
        // std::cout << "Aligning to gravity... ";
        // std::cout.flush();
        this->gravityAlign();
    }

    // Use initial known pose
    if(this->param.initial_pose_use_)
    {
        // std::cout << "Setting known initial pose... ";
        // std::cout.flush();

        // set known position
        this->pose = this->param.initial_position_;
        this->T.block(0, 3, 3, 1) = this->pose;
        this->T_s2s.block(0, 3, 3, 1) = this->pose;
        this->T_s2s_prev.block(0, 3, 3, 1) = this->pose;
        this->origin = this->param.initial_position_;

        // set known orientation
        this->rotq = this->param.initial_orientation_;
        this->T.block(0, 0, 3, 3) = this->rotq.toRotationMatrix();
        this->T_s2s.block(0, 0, 3, 3) = this->rotq.toRotationMatrix();
        this->T_s2s_prev.block(0, 0, 3, 3) = this->rotq.toRotationMatrix();

        // std::cout << "done" << std::endl << std::endl;
    }

    this->dlo_initialized = true;
    // std::cout << "DLO initialized! Starting localization..." << std::endl;
}

void DLOdom::gravityAlign()
{
    // get average acceleration vector for 1 second and normalize
    Eigen::Vector3f lin_accel = Eigen::Vector3f::Zero();
    const double then = this->now().seconds();
    int n = 0;
    while((this->now().seconds() - then) < 1.)
    {
        lin_accel[0] += this->imu_meas.lin_accel.x;
        lin_accel[1] += this->imu_meas.lin_accel.y;
        lin_accel[2] += this->imu_meas.lin_accel.z;
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
    Eigen::Vector3f grav;
    grav << 0, 0, 1;

    // calculate angle between the two vectors
    Eigen::Quaternionf grav_q = Eigen::Quaternionf::FromTwoVectors(lin_accel, grav);

    // normalize
    double grav_norm =
        sqrt(grav_q.w() * grav_q.w() + grav_q.x() * grav_q.x() + grav_q.y() * grav_q.y() + grav_q.z() * grav_q.z());
    grav_q.w() /= grav_norm;
    grav_q.x() /= grav_norm;
    grav_q.y() /= grav_norm;
    grav_q.z() /= grav_norm;

    // set gravity aligned orientation
    this->rotq = grav_q;
    this->T.block(0, 0, 3, 3) = this->rotq.toRotationMatrix();
    this->T_s2s.block(0, 0, 3, 3) = this->rotq.toRotationMatrix();
    this->T_s2s_prev.block(0, 0, 3, 3) = this->rotq.toRotationMatrix();

    // rpy
    // auto euler = grav_q.toRotationMatrix().eulerAngles(2, 1, 0);
    // double yaw = euler[0] * (180.0 / M_PI);
    // double pitch = euler[1] * (180.0 / M_PI);
    // double roll = euler[2] * (180.0 / M_PI);

    // std::cout << "done" << std::endl;
    // std::cout << "  Roll [deg]: " << roll << std::endl;
    // std::cout << "  Pitch [deg]: " << pitch << std::endl << std::endl;
}

void DLOdom::getNextPose()
{
    //
    // FRAME-TO-FRAME PROCEDURE
    //

    // Align using IMU prior if available
    pcl::PointCloud<PointType>::Ptr aligned = std::make_shared<pcl::PointCloud<PointType>>();

    if(this->param.imu_use_)
    {
        this->integrateIMU();
        this->gicp_s2s.align(*aligned, this->imu_SE3);
    }
    else
    {
        this->gicp_s2s.align(*aligned);
    }

    // Get the local S2S transform
    Eigen::Matrix4f T_S2S = this->gicp_s2s.getFinalTransformation();

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

    if(this->submap_hasChanged)
    {

        // Set the current global submap as the target cloud
        this->gicp.setInputTarget(this->submap_cloud);

        // Set target cloud's normals as submap normals
        this->gicp.setTargetCovariances(this->submap_normals);
    }

    // Align with current submap with global S2S transformation as initial guess
    this->gicp.align(*aligned, this->T_s2s);

    // Get final transformation in global frame
    this->T = this->gicp.getFinalTransformation();

    // Update the S2S transform for next propagation
    this->T_s2s_prev = this->T;

    // Update next global pose
    // Both source and target clouds are in the global frame now, so tranformation is global
    this->propagateS2M();

    // Set next target cloud as current source cloud
    *this->target_cloud = *this->source_cloud;
}

void DLOdom::integrateIMU()
{
    // Extract IMU data between the two frames
    std::vector<ImuMeas> imu_frame;

    this->mtx_imu.lock();
    for(const auto & i : this->imu_buffer)
    {

        // IMU data between two frames is when:
        //   current frame's timestamp minus imu timestamp is positive
        //   previous frame's timestamp minus imu timestamp is negative
        double curr_frame_imu_dt = this->curr_frame_stamp - i.stamp;
        double prev_frame_imu_dt = this->prev_frame_stamp - i.stamp;

        if(curr_frame_imu_dt >= 0. && prev_frame_imu_dt <= 0.)
        {
            imu_frame.push_back(i);
        }
    }
    this->mtx_imu.unlock();

    // Sort measurements by time
    std::sort(imu_frame.begin(), imu_frame.end(), this->comparatorImu);

    // Relative IMU integration of gyro and accelerometer
    double curr_imu_stamp = 0.;
    double prev_imu_stamp = 0.;
    double dt;

    Eigen::Quaternionf q = Eigen::Quaternionf::Identity();

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
        Eigen::Quaternionf qq = q;
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

    // Store IMU guess
    this->imu_SE3 = Eigen::Matrix4f::Identity();
    this->imu_SE3.block(0, 0, 3, 3) = q.toRotationMatrix();
}

void DLOdom::propegateS2S(Eigen::Matrix4f T)
{
    this->T_s2s = this->T_s2s_prev * T;
    this->T_s2s_prev = this->T_s2s;

    this->pose_s2s << this->T_s2s(0, 3), this->T_s2s(1, 3), this->T_s2s(2, 3);
    this->rotSO3_s2s << this->T_s2s(0, 0), this->T_s2s(0, 1), this->T_s2s(0, 2), this->T_s2s(1, 0), this->T_s2s(1, 1),
        this->T_s2s(1, 2), this->T_s2s(2, 0), this->T_s2s(2, 1), this->T_s2s(2, 2);

    Eigen::Quaternionf q(this->rotSO3_s2s);

    // Normalize quaternion
    double norm = sqrt(q.w() * q.w() + q.x() * q.x() + q.y() * q.y() + q.z() * q.z());
    q.w() /= norm;
    q.x() /= norm;
    q.y() /= norm;
    q.z() /= norm;
    this->rotq_s2s = q;
}

void DLOdom::propegateS2M()
{
    this->pose << this->T(0, 3), this->T(1, 3), this->T(2, 3);
    this->rotSO3 << this->T(0, 0), this->T(0, 1), this->T(0, 2), this->T(1, 0), this->T(1, 1), this->T(1, 2),
        this->T(2, 0), this->T(2, 1), this->T(2, 2);

    Eigen::Quaternionf q(this->rotSO3);

    // Normalize quaternion
    double norm = sqrt(q.w() * q.w() + q.x() * q.x() + q.y() * q.y() + q.z() * q.z());
    q.w() /= norm;
    q.x() /= norm;
    q.y() /= norm;
    q.z() /= norm;
    this->rotq = q;
}

void DLOdom::setAdaptiveParams()
{
    // compute range of points "spaciousness"
    std::vector<float> ds;

    for(int i = 0; i <= this->current_scan->points.size(); i++)
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

void DLOdom::transformCurrentScan()
{
    this->current_scan_t = std::make_shared<pcl::PointCloud<PointType>>();
    pcl::transformPointCloud(*this->current_scan, *this->current_scan_t, this->T);
}

void DLOdom::updateKeyframes()
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
        float delta_d = sqrt(pow(this->pose[0] - k.first.first[0], 2) + pow(this->pose[1] - k.first.first[1], 2) +
                             pow(this->pose[2] - k.first.first[2], 2));

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
    Eigen::Vector3f closest_pose = this->keyframes[closest_idx].first.first;
    Eigen::Quaternionf closest_pose_r = this->keyframes[closest_idx].first.second;

    // calculate distance between current pose and closest pose from above
    float dd = sqrt(pow(this->pose[0] - closest_pose[0], 2) + pow(this->pose[1] - closest_pose[1], 2) +
                    pow(this->pose[2] - closest_pose[2], 2));

    // calculate difference in orientation
    Eigen::Quaternionf dq = this->rotq * (closest_pose_r.inverse());

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

        ++this->num_keyframes;

        // voxelization for submap
        if(this->param.vf_submap_use_)
        {
            this->vf_submap.setInputCloud(this->current_scan_t);
            this->vf_submap.filter(*this->current_scan_t);
        }

        // update keyframe vector
        this->keyframes.push_back(std::make_pair(std::make_pair(this->pose, this->rotq), this->current_scan_t));

        // compute kdtree and keyframe normals (use gicp_s2s input source as temporary storage because it will be
        // overwritten by setInputSources())
        *this->keyframes_cloud += *this->current_scan_t;
        *this->keyframe_cloud = *this->current_scan_t;

        this->gicp_s2s.setInputSource(this->keyframe_cloud);
        this->gicp_s2s.calculateSourceCovariances();
        this->keyframe_normals.push_back(this->gicp_s2s.getSourceCovariances());

        // this->publish_keyframe_thread = std::thread(&dlo::OdomNode::publishKeyframe, this);
        // this->publish_keyframe_thread.detach();
    }
}

void DLOdom::computeConvexHull()
{
    // at least 4 keyframes for convex hull
    if(this->num_keyframes < 4)
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
    for(int i = 0; i < convex_hull_point_idx->indices.size(); ++i)
    {
        this->keyframe_convex.push_back(convex_hull_point_idx->indices[i]);
    }
}

void DLOdom::computeConcaveHull()
{
    // at least 5 keyframes for concave hull
    if(this->num_keyframes < 5)
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
    for(int i = 0; i < concave_hull_point_idx->indices.size(); ++i)
    {
        this->keyframe_concave.push_back(concave_hull_point_idx->indices[i]);
    }
}

void DLOdom::pushSubmapIndices(std::vector<float> dists, int k, std::vector<int> frames)
{
    // make sure dists is not empty
    if(!dists.size())
    {
        return;
    }

    // maintain max heap of at most k elements
    std::priority_queue<float> pq;

    for(auto d : dists)
    {
        if(pq.size() >= k && pq.top() > d)
        {
            pq.push(d);
            pq.pop();
        }
        else if(pq.size() < k)
        {
            pq.push(d);
        }
    }

    // get the kth smallest element, which should be at the top of the heap
    float kth_element = pq.top();

    // get all elements smaller or equal to the kth smallest element
    for(int i = 0; i < dists.size(); ++i)
    {
        if(dists[i] <= kth_element)
            this->submap_kf_idx_curr.push_back(frames[i]);
    }
}

void DLOdom::getSubmapKeyframes()
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
    Eigen::Vector3f curr_pose = this->T_s2s.block(0, 3, 3, 1);

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
        this->submap_hasChanged = false;
    }
    else
    {
        this->submap_hasChanged = true;

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
