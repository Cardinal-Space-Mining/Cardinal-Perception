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


void DLOdom::preprocessPoints()
{

}

void DLOdom::initializeInputTarget()
{

}

void DLOdom::setInputSources()
{

}

void DLOdom::initializeDLO()
{

}

void DLOdom::gravityAlign()
{

}

void DLOdom::getNextPose()
{

}

void DLOdom::integrateIMU()
{

}

void DLOdom::propegateS2S(Eigen::Matrix4f T)
{

}

void DLOdom::propegateS2M()
{

}

void DLOdom::setAdaptiveParams()
{

}

void DLOdom::computeMetrics()
{

}

void DLOdom::computeSpaciousness()
{

}

void DLOdom::transformCurrentScan()
{

}

void DLOdom::updateKeyframes()
{

}

void DLOdom::computeConvexHull()
{

}

void DLOdom::computeConcaveHull()
{

}

void DLOdom::pushSubmapIndices(std::vector<float> dists, int k, std::vector<int> frames)
{

}

void DLOdom::getSubmapKeyframes()
{

}
