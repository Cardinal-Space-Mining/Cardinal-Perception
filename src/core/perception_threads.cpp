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

#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <imu_transform.hpp>

#include <cloud_ops.hpp>


using namespace util::geom::cvt::ops;


namespace csm
{
namespace perception
{

// --- DIRECT ROS CALLBACK WORKERS --------------------------------------------

void PerceptionNode::imu_worker(const ImuMsg::SharedPtr& imu)
{
    PROFILING_SYNC();
    PROFILING_NOTIFY_ALWAYS(imu);

    try
    {
        auto tf = this->tf_buffer.lookupTransform(
            this->base_frame,
            imu->header.frame_id,
            util::toTf2TimePoint(imu->header.stamp));

        tf2::doTransform(*imu, *imu, tf);

        this->imu_samples.addSample(*imu);
    }
    catch (const std::exception& e)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "[IMU CALLBACK]: Failed to process imu measurment.\n\twhat(): %s",
            e.what());
    }

#if PERCEPTION_PUBLISH_GRAV_ESTIMATION > 0
    double stddev, delta_r;
    Eigen::Vector3d grav_vec =
        this->imu_samples.estimateGravity(1., &stddev, &delta_r);

    PoseStampedMsg grav_pub;
    grav_pub.header.stamp = imu->header.stamp;
    grav_pub.header.frame_id = this->base_frame;
    grav_pub.pose.orientation << Eigen::Quaterniond::FromTwoVectors(
        Eigen::Vector3d{1., 0., 0.},
        grav_vec);
    this->pose_pub.publish("gravity_estimation", grav_pub);

    this->metrics_pub.publish("gravity_estimation/acc_stddev", stddev);
    this->metrics_pub.publish("gravity_estimation/delta_rotation", delta_r);
#endif

    PROFILING_NOTIFY_ALWAYS(imu);
}



#if TAG_DETECTION_ENABLED
void PerceptionNode::detection_worker(
    const TagsTransformMsg::ConstSharedPtr& detection_group)
{
    PROFILING_SYNC();
    PROFILING_NOTIFY_ALWAYS(tags_detection);

    const geometry_msgs::msg::TransformStamped& tf =
        detection_group->estimated_tf;
    if (tf.header.frame_id == this->map_frame &&
        tf.child_frame_id == this->base_frame &&
        (this->param.tag_usage_mode > 0 ||
         (this->param.tag_usage_mode < 0 &&
          detection_group->filter_mask >= 31)))
    {
        TagDetection::Ptr td = std::make_shared<TagDetection>();
        td->pose << tf.transform;
        td->time_point = util::toFloatSeconds(tf.header.stamp);
        td->pix_area = detection_group->pix_area;
        td->avg_range = detection_group->avg_range;
        td->rms = detection_group->rms;
        td->num_tags = detection_group->num_tags;
        this->transform_sync.endMeasurementIterationSuccess(td, td->time_point);

        // RCLCPP_INFO(this->get_logger(), "[DETECTION CB]: Recv - Base delta: %f", util::toFloatSeconds(this->get_clock()->now()) - td->time_point);
    }

    PROFILING_NOTIFY_ALWAYS(tags_detection);
}
#endif





// --- INTERNAL UTILITIES -----------------------------------------------------

int PerceptionNode::preprocess_scan(
    const PointCloudMsg::ConstSharedPtr& scan,
    util::geom::PoseTf3f& lidar_to_base_tf,
    OdomPointCloudType& lo_cloud,
    std::vector<RayDirectionType>& null_vecs,
    pcl::Indices& nan_indices,
    pcl::Indices& remove_indices)
{
    // get lidar --> base link transform
    try
    {
        this->tf_buffer
                .lookupTransform(
                    this->base_frame,
                    scan->header.frame_id,
                    util::toTf2TimePoint(scan->header.stamp))
                .transform >>
            lidar_to_base_tf.pose >> lidar_to_base_tf.tf;
    }
    catch (const std::exception& e)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "[SCAN CALLBACK]: Failed to get transform from '%s' to '%s'\n\twhat(): %s",
            this->base_frame.c_str(),
            scan->header.frame_id.c_str(),
            e.what());
        return -1;
    }

    thread_local pcl::PointCloud<csm::perception::PointSDir> dir_cloud;
#if PERCEPTION_USE_SCAN_DESKEW
    thread_local OdomPointCloudType tmp_cloud;
#else
    OdomPointCloudType& tmp_cloud = lo_cloud;
#endif
    thread_local pcl::Indices bbox_indices;

    // convert and transform to base link while extracting NaN indices
    pcl::fromROSMsg(*scan, lo_cloud);
    util::transformAndFilterNull(
        lo_cloud,
        tmp_cloud,
        nan_indices,
        lidar_to_base_tf.tf.matrix());

    // apply crop box and accumulate removal indices
    if (this->param.use_crop_filter)
    {
        util::cropbox_filter(
            tmp_cloud,
            bbox_indices,
            this->param.base_link_crop_min,
            this->param.base_link_crop_max);

        util::pc_combine_sorted(nan_indices, bbox_indices, remove_indices);
    }
    else
    {
        remove_indices = nan_indices;
    }

#if PERCEPTION_USE_NULL_RAY_DELETION
    if (!nan_indices.empty())
    {
        thread_local pcl::PointCloud<pcl::PointXYZ> dbg_cloud;

        pcl::fromROSMsg(*scan, dir_cloud);

        null_vecs.clear();
        null_vecs.reserve(nan_indices.size());
        for (pcl::index_t i : nan_indices)
        {
            PointSDir& s = dir_cloud[i];
            s.elevation = -s.elevation + (3.1415926f / 2.f);
            // s.elevation *= (-3.1415926f / 90.f);
            // s.azimuth *= (3.1415926f / 90.f);

            const float se = std::sin(s.elevation);
            null_vecs.emplace_back(
                std::cos(s.azimuth) * se,
                std::sin(s.azimuth) * se,
                std::cos(s.elevation));
        }

        dbg_cloud.points.clear();
        dbg_cloud.points.reserve(null_vecs.size());
        for (const RayDirectionType& r : null_vecs)
        {
            dbg_cloud.points.emplace_back(
                r.normal_x * 10.f,
                r.normal_y * 10.f,
                r.normal_z * 10.f);
        }
        dbg_cloud.width = dbg_cloud.points.size();
        dbg_cloud.height = 1;

        PointCloudMsg dbg_points_out;
        pcl::toROSMsg(dbg_cloud, dbg_points_out);
        dbg_points_out.header.stamp = util::toTimeStamp(scan->header.stamp);
        dbg_points_out.header.frame_id = scan->header.frame_id;
        this->scan_pub.publish("null_point_directions", dbg_points_out);
    }
#else
    (void)null_vecs;
#endif

    // deskew procedure
#if PERCEPTION_USE_SCAN_DESKEW
    thread_local pcl::PointCloud<csm::perception::PointT_32HL> ts_cloud;

    ts_cloud.clear();
    pcl::fromROSMsg(*scan, ts_cloud);

    if (lo_cloud.size() == ts_cloud.size())
    {
        uint64_t min_ts = ts_cloud[0].t, max_ts = ts_cloud[0].t;
        for (size_t i = 1; i < ts_cloud.size(); i++)
        {
            if (ts_cloud[i].t < min_ts)
            {
                min_ts = ts_cloud[i].t;
            }
            if (ts_cloud[i].t > max_ts)
            {
                max_ts = ts_cloud[i].t;
            }
        }

        const double ts_diff = static_cast<double>(max_ts - min_ts);
        const double beg_range = util::toFloatSeconds(scan->header.stamp);
        const double end_range = beg_range + ts_diff * 1e-6;

        util::tsq::TSQ<Eigen::Quaterniond> offsets;
        if (imu_samples.getNormalizedOffsets(offsets, beg_range, end_range) &&
            offsets.front().second.angularDistance(offsets.back().second) >=
                1e-3)  // only process if rotation >= 1 mrad
        {
            // for (auto& sample : offsets)
            // {
            //     sample.second = sample.second.conjugate();
            // }

            // std::cout <<
            //     "[DESKEW]: Obtained " << offsets.size() <<
            //     " samples (" << offsets.back().second.angularDistance(offsets.front().second) << " rad).";
            // for(size_t i = 0; i < offsets.size(); i++)
            // {
            //     Eigen::Quaterniond& q = offsets[i].second;
            //     std::cout <<
            //         "\n\t" << offsets[i].first <<
            //         " : { " << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << " } (" <<
            //         q.norm() << ')';
            // }
            // std::cout << std::endl;

            size_t skip_i = 0;
    #if PERCEPTION_USE_NULL_RAY_DELETION
            size_t nan_i = 0;
    #endif
            for (size_t i = 0; i < lo_cloud.size(); i++)
            {
                pcl::Vector4fMap v{nullptr};
    #if PERCEPTION_USE_NULL_RAY_DELETION
                if (!nan_indices.empty() &&
                    static_cast<size_t>(nan_indices[nan_i]) == i)
                {
                    // v = null_vecs[nan_i].getNormalVector4fMap();
                    new(&v) pcl::Vector4fMap{null_vecs[nan_i].data_n};
                    nan_i++;
                    skip_i++;
                }
                else
    #endif
                    if (!remove_indices.empty() &&
                        static_cast<size_t>(remove_indices[skip_i]) == i)
                {
                    skip_i++;
                    continue;
                }
                else
                {
                    // v = lo_cloud[i].getVector4fMap();
                    new(&v) pcl::Vector4fMap{lo_cloud[i].data};
                }

                const double t =
                    static_cast<double>(ts_cloud[i].t - min_ts) / ts_diff;
                Eigen::Quaterniond q;
                const size_t idx = util::tsq::binarySearchIdx(offsets, t);
                if (offsets[idx].first == t)
                {
                    q = offsets[idx].second;
                }
                else
                {
                    const auto& a = offsets[idx];
                    const auto& b = offsets[idx - 1];

                    q = a.second.slerp(
                        (t - a.first) / (b.first - a.first),
                        b.second);
                }

                Eigen::Matrix4f rot = Eigen::Matrix4f::Identity();
                rot.block<3, 3>(0, 0) =
                    q.template cast<float>().toRotationMatrix();

                v = rot * v;
            }

            // remove bbox and null points
            util::pc_remove_selection(lo_cloud, remove_indices);
            lo_cloud.is_dense = true;

            // transform deskewed cloud
            pcl::transformPointCloud(
                lo_cloud,
                lo_cloud,
                lidar_to_base_tf.tf.matrix());

            return 0;
        }
    }

    // deskewing failed so copy transformed/filtered original
    lo_cloud.swap(tmp_cloud);
    util::pc_remove_selection(lo_cloud, remove_indices);
    lo_cloud.is_dense = true;

    return 1;
#else
    // deskewing was disabled - still need to remove bbox and null points
    util::pc_remove_selection(lo_cloud, remove_indices);
    lo_cloud.is_dense = true;

    return 0;
#endif
}





// --- INTERNAL CALLBACK WORKERS ----------------------------------------------

void PerceptionNode::scan_callback_internal(
    const PointCloudMsg::ConstSharedPtr& scan)
{
    PROFILING_NOTIFY(odometry_init);

    thread_local OdomPointCloudType lo_cloud;
    thread_local std::vector<RayDirectionType> null_vecs;
    thread_local pcl::Indices nan_indices, remove_indices;
    util::geom::PoseTf3f lidar_to_base_tf, base_to_odom_tf;
    const auto scan_stamp = scan->header.stamp;

    PROFILING_NOTIFY2(odometry_init, odometry_preprocess);

    lo_cloud.clear();
    if (this->preprocess_scan(
            scan,
            lidar_to_base_tf,
            lo_cloud,
            null_vecs,
            nan_indices,
            remove_indices) < 0)
    {
        return;
    }

    PROFILING_NOTIFY2(odometry_preprocess, odometry_lfd_export);

    const uint32_t iteration_token =
        this->transform_sync.beginOdometryIteration();

#if LFD_ENABLED
    // send data to fiducial thread to begin asynchronous localization
    FiducialResources& f = this->mt.fiducial_resources.lockInput();
    f.lidar_to_base = lidar_to_base_tf;
    f.raw_scan = scan;
    if (!f.nan_indices || f.nan_indices.use_count() > 1)
    {
        f.nan_indices = std::make_shared<pcl::Indices>();
    }
    if (!f.remove_indices || f.remove_indices.use_count() > 1)
    {
        f.remove_indices = std::make_shared<pcl::Indices>();
    }
    const_cast<pcl::Indices&>(*f.nan_indices).swap(nan_indices);
    const_cast<pcl::Indices&>(*f.remove_indices).swap(remove_indices);
    const auto& nan_indices_ptr = f.nan_indices;
    const auto& remove_indices_ptr = f.remove_indices;
    f.iteration_count = iteration_token;
    this->mt.fiducial_resources.unlockInputAndNotify(f);
#else
    (void)iteration_token;
#endif

    PROFILING_NOTIFY2(odometry_lfd_export, odometry_lio);

    // apply sensor origin
    lo_cloud.sensor_origin_ << lidar_to_base_tf.pose.vec, 1.f;
    const double new_odom_stamp = util::toFloatSeconds(scan_stamp);

    // get imu estimated rotation
    Eigen::Matrix4f imu_rot = Eigen::Matrix4f::Identity();
    if (this->imu_samples.hasSamples())
    {
        imu_rot.block<3, 3>(0, 0) =
            this->imu_samples
                .getDelta(
                    this->lidar_odom.state.prev_frame_stamp,
                    new_odom_stamp)
                .template cast<float>()
                .toRotationMatrix();
    }

    // iterate odometry
    auto lo_status = this->lidar_odom.processScan(
        lo_cloud,
        new_odom_stamp,
        base_to_odom_tf,
        imu_rot);

    PROFILING_NOTIFY(odometry_lio);

    // on failure >>>
    if (!lo_status)
    {
        this->transform_sync.endOdometryIterationFailure();
    }
    // on success >>>
    else
    {
        PROFILING_NOTIFY(odometry_map_export);

#if MAPPING_ENABLED
        {
            // Send data to mapping thread
            MappingResources& m = this->mt.mapping_resources.lockInput();
            m.lidar_to_base = lidar_to_base_tf;
            m.base_to_odom = base_to_odom_tf;
            m.raw_scan = scan;
            m.lo_buff.swap(lo_cloud);
    #if PERCEPTION_USE_NULL_RAY_DELETION
            m.null_vecs.swap(null_vecs);
    #endif
    #if LFD_ENABLED  // LFD doesnt share removal indices
            m.nan_indices = nan_indices_ptr;
            m.remove_indices = remove_indices_ptr;
    #else
            if (!m.nan_indices)
            {
                m.nan_indices = std::make_shared<pcl::Indices>();
            }
            if (!m.remove_indices)
            {
                m.remove_indices = std::make_shared<pcl::Indices>();
            }
            const_cast<pcl::Indices&>(*m.nan_indices).swap(nan_indices);
            const_cast<pcl::Indices&>(*m.remove_indices).swap(remove_indices);
    #endif
            this->mt.mapping_resources.unlockInputAndNotify(m);
        }
#endif

        PROFILING_NOTIFY2(odometry_map_export, odometry_debpub);

        util::geom::PoseTf3d prev_odom_tf, new_odom_tf;
        const double prev_odom_stamp =
            this->transform_sync.getOdomTf(prev_odom_tf);

        // Update odom tf
        this->transform_sync.endOdometryIterationSuccess(
            base_to_odom_tf,
            new_odom_stamp);

        // Publish velocity
        const double t_diff =
            this->transform_sync.getOdomTf(new_odom_tf) - prev_odom_stamp;
        Eigen::Vector3d l_vel =
            (new_odom_tf.pose.vec - prev_odom_tf.pose.vec) / t_diff;
        Eigen::Vector3d r_vel =
            (prev_odom_tf.pose.quat.inverse() * new_odom_tf.pose.quat)
                .toRotationMatrix()
                .eulerAngles(0, 1, 2) /
            t_diff;

        TwistStampedMsg odom_vel;
        odom_vel.twist.linear << l_vel;
        odom_vel.twist.angular << r_vel;
        odom_vel.header.frame_id = this->odom_frame;
        odom_vel.header.stamp = scan_stamp;
        this->generic_pub.publish("odom_velocity", odom_vel);

        // Publish LO debug
#if PERCEPTION_PUBLISH_LIO_DEBUG > 0
        this->lidar_odom.publishDebugScans(lo_status, this->odom_frame);
#endif

        // Publish filtering debug
#if PERCEPTION_PUBLISH_TRJF_DEBUG > 0
        const auto& trjf = this->transform_sync.trajectoryFilter();

        TrajectoryFilterDebugMsg dbg;
        dbg.is_stable = trjf.lastFilterStatus();
        dbg.filter_mask = trjf.lastFilterMask();
        dbg.odom_queue_size = trjf.odomQueueSize();
        dbg.meas_queue_size = trjf.measurementQueueSize();
        dbg.trajectory_length = trjf.trajectoryQueueSize();
        dbg.filter_dt = trjf.lastFilterWindow();
        dbg.linear_error = trjf.lastLinearDelta();
        dbg.angular_error = trjf.lastAngularDelta();
        dbg.linear_deviation = trjf.lastLinearDeviation();
        dbg.angular_deviation = trjf.lastAngularDeviation();
        dbg.avg_linear_error = trjf.lastAvgLinearError();
        dbg.avg_angular_error = trjf.lastAvgAngularError();
        this->generic_pub.publish("metrics/trajectory_filter_stats", dbg);
#endif

        PROFILING_NOTIFY(odometry_debpub);
    }
}





#if LFD_ENABLED
void PerceptionNode::fiducial_callback_internal(FiducialResources& buff)
{
    PROFILING_NOTIFY(lfd_preprocess);

    this->transform_sync.beginMeasurementIteration(buff.iteration_count);

    thread_local pcl::PointCloud<FiducialPointType> reflector_points;
    pcl::fromROSMsg(*buff.raw_scan, reflector_points);

    util::pc_remove_selection(reflector_points, *buff.remove_indices);
    // signals to odom thread that these buffers can be reused
    buff.nan_indices.reset();
    buff.remove_indices.reset();

    double stddev, delta_r;
    const Eigen::Vector3d grav_vec =
        this->imu_samples.estimateGravity(0.5, &stddev, &delta_r);

    Eigen::Vector3f up_vec{ 0, 0, 1 };
    // if (stddev < 1. && delta_r < 0.01)
    {
        up_vec = grav_vec.template cast<float>();
    }
    up_vec = (buff.lidar_to_base.tf.inverse() * up_vec);

    util::geom::PoseTf3f fiducial_pose;
    PROFILING_NOTIFY2(lfd_preprocess, lfd_calculate);
    auto result = this->fiducial_detector.calculatePose(
        fiducial_pose.pose,
        reflector_points,
        up_vec );
    PROFILING_NOTIFY2(lfd_calculate, lfd_export);

    if (result)
    {
        util::geom::Pose3d fiducial_to_base, base_to_fiducial;
        fiducial_to_base
            << (buff.lidar_to_base.tf *
                (fiducial_pose.tf << fiducial_pose.pose));
        util::geom::inverse(base_to_fiducial, fiducial_to_base);

        this->transform_sync.endMeasurementIterationSuccess(
            base_to_fiducial,
            util::toFloatSeconds(buff.raw_scan->header.stamp));

        PoseStampedMsg p;
        p.pose << fiducial_to_base;
        p.header.stamp = buff.raw_scan->header.stamp;
        p.header.frame_id = this->base_frame;

        this->pose_pub.publish("fiducial_tag_pose", p);
    }
    else
    {
        this->transform_sync.endMeasurementIterationFailure();
    }

    PROFILING_NOTIFY2(lfd_export, lfd_debpub);

    #if PERCEPTION_PUBLISH_LFD_DEBUG > 0
    try
    {
        PoseStampedMsg _p;
        PointCloudMsg _pc;

        const auto& input_cloud = this->fiducial_detector.getInputPoints();
        const auto& redetect_cloud = this->fiducial_detector.getRedetectPoints();

        pcl::toROSMsg(input_cloud, _pc);
        _pc.header = buff.raw_scan->header;
        this->scan_pub.publish("fiducial_input_points", _pc);

        pcl::toROSMsg(redetect_cloud, _pc);
        _pc.header = buff.raw_scan->header;
        this->scan_pub.publish("fiducial_redetect_points", _pc);

        // if (result.has_point_num)
        // {
            const auto& seg_clouds = this->fiducial_detector.getSegClouds();
            const auto& seg_planes = this->fiducial_detector.getSegPlanes();
            const auto& seg_plane_centers =
                this->fiducial_detector.getPlaneCenters();
            const auto& remaining_points =
                this->fiducial_detector.getRemainingPoints();

            for (uint32_t i = 0; i < 3; i++)
            {
                _p.header = buff.raw_scan->header;
                _p.pose.position << seg_plane_centers[i];
                _p.pose.orientation << Eigen::Quaternionf::FromTwoVectors(
                    Eigen::Vector3f{1.f, 0.f, 0.f},
                    seg_planes[i].head<3>());

                std::string topic =
                    (std::ostringstream{} << "fiducial_plane_" << i << "/pose")
                        .str();
                this->pose_pub.publish(topic, _p);

                if(i < result.iterations)
                {
                    pcl::toROSMsg(seg_clouds[i], _pc);
                }
                else
                {
                    pcl::PointCloud<pcl::PointXYZ> empty_cloud;
                    pcl::toROSMsg(empty_cloud, _pc);
                }
                _pc.header = buff.raw_scan->header;

                topic = ((std::ostringstream{} << "fiducial_plane_" << i
                                               << "/points")
                             .str());
                this->scan_pub.publish(topic, _pc);
            }

            if (result.iterations == 3 && !remaining_points.empty())
            {
                pcl::toROSMsg(remaining_points, _pc);
                _pc.header = buff.raw_scan->header;

                this->scan_pub.publish("fiducial_unmodeled_points", _pc);
            }
        // }
    }
    catch (const std::exception& e)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "[FIDUCIAL DETECTION]: Failed to publish debug data -- what():\n\t%s",
            e.what());
    }
    #endif

    PROFILING_NOTIFY(lfd_debpub);
}
#endif





#if MAPPING_ENABLED
void PerceptionNode::mapping_callback_internal(MappingResources& buff)
{
    PROFILING_NOTIFY(mapping_preproc);

    // RCLCPP_INFO(this->get_logger(), "MAPPING CALLBACK INTERNAL");

    util::geom::PoseTf3f lidar_to_odom_tf;
    lidar_to_odom_tf.pose
        << (lidar_to_odom_tf.tf = buff.base_to_odom.tf * buff.lidar_to_base.tf);

    pcl::PointCloud<MappingPointType>* filtered_scan_t = nullptr;
    if constexpr (std::is_same<OdomPointType, MappingPointType>::value)
    {
        pcl::transformPointCloud(
            buff.lo_buff,
            buff.lo_buff,
            buff.base_to_odom.tf,
            true);
        filtered_scan_t = &buff.lo_buff;
    }
    else
    {
        thread_local pcl::PointCloud<MappingPointType> map_input_cloud;
        pcl::fromROSMsg(*buff.raw_scan, map_input_cloud);

        util::pc_remove_selection(map_input_cloud, *buff.remove_indices);
        pcl::transformPointCloud(
            map_input_cloud,
            map_input_cloud,
            lidar_to_odom_tf.tf,
            true);
        filtered_scan_t = &map_input_cloud;
    }

    #if PERCEPTION_USE_NULL_RAY_DELETION
    for (RayDirectionType& r : buff.null_vecs)
    {
        r.getNormalVector4fMap() =
            lidar_to_odom_tf.tf * r.getNormalVector4fMap();
    }

    PROFILING_NOTIFY2(mapping_preproc, mapping_kfc_update);

    auto results = this->environment_map.updateMap(
        lidar_to_odom_tf.pose.vec,
        *filtered_scan_t,
        buff.null_vecs);
    #else

    PROFILING_NOTIFY2(mapping_preproc, mapping_kfc_update);

    auto results = this->environment_map.updateMap(
        lidar_to_odom_tf.pose.vec,
        *filtered_scan_t);
    #endif

    PROFILING_NOTIFY2(mapping_kfc_update, mapping_search_local);

    pcl::Indices export_points;
    const Eigen::Vector3f search_range{
        static_cast<float>(this->param.map_export_horizontal_range),
        static_cast<float>(this->param.map_export_horizontal_range),
        static_cast<float>(this->param.map_export_vertical_range)},
        search_min{buff.base_to_odom.pose.vec - search_range},
        search_max{buff.base_to_odom.pose.vec + search_range};

    this->environment_map.getMap().boxSearch(
        search_min,
        search_max,
        export_points);

    PROFILING_NOTIFY2(mapping_search_local, mapping_export_trav);

    #if TRAVERSABILITY_ENABLED
    {
        auto& x = this->mt.traversibility_resources.lockInput();
        x.search_min = search_min;
        x.search_max = search_max;
        x.lidar_to_base = buff.lidar_to_base;
        x.base_to_odom = buff.base_to_odom;
        if (!x.points || x.points.use_count() > 1)
        {
            x.points = std::make_shared<pcl::PointCloud<MappingPointType>>();
        }
        util::pc_copy_selection(
            *this->environment_map.getPoints(),
            export_points,
            *x.points);
        x.stamp = util::toFloatSeconds(buff.raw_scan->header.stamp);
        this->mt.traversibility_resources.unlockInputAndNotify(x);
    }
    #else
    try
    {
        thread_local pcl::PointCloud<MappingPointType> trav_points;
        util::pc_copy_selection(
            *this->environment_map.getPoints(),
            export_points,
            trav_points);

        PointCloudMsg trav_points_output;
        pcl::toROSMsg(trav_points, trav_points_output);
        trav_points_output.header.stamp =
            util::toTimeStamp(buff.raw_scan->header.stamp);
        trav_points_output.header.frame_id = this->odom_frame;
        this->scan_pub.publish("traversability_points", trav_points_output);
    }
    catch (const std::exception& e)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "[MAPPING]: Failed to publish traversability subcloud -- what():\n\t%s",
            e.what());
    }
    #endif

    PROFILING_NOTIFY2(mapping_export_trav, mapping_debpub);

    try
    {
        PointCloudMsg output;
    #if PERCEPTION_PUBLISH_FULL_MAP > 0
        pcl::toROSMsg(*this->environment_map.getPoints(), output);
        output.header.stamp = buff.raw_scan->header.stamp;
        output.header.frame_id = this->odom_frame;
        this->scan_pub.publish("map_cloud", output);
    #endif

        pcl::toROSMsg(*filtered_scan_t, output);
        output.header.stamp = buff.raw_scan->header.stamp;
        output.header.frame_id = this->odom_frame;
        this->scan_pub.publish("filtered_scan", output);
    }
    catch (const std::exception& e)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "[MAPPING]: Failed to publish mapping debug scans -- what():\n\t%s",
            e.what());
    }

    this->metrics_pub.publish(
        "mapping/search_pointset",
        static_cast<double>(results.points_searched));
    this->metrics_pub.publish(
        "mapping/points_deleted",
        static_cast<double>(results.points_deleted));

    PROFILING_NOTIFY(mapping_debpub);
}
#endif





#if TRAVERSABILITY_ENABLED
void PerceptionNode::traversibility_callback_internal(
    TraversabilityResources& buff)
{
    PROFILING_NOTIFY(traversibility_preproc);

    thread_local Eigen::Vector3f env_grav_vec{0.f, 0.f, 1.f};
    if (this->imu_samples.hasSamples())
    {
        double stddev, delta_r;
        const Eigen::Vector3d grav_vec =
            this->imu_samples.estimateGravity(0.5, &stddev, &delta_r);
        if (stddev < 1. && delta_r < 0.01)
        {
            env_grav_vec =
                (buff.base_to_odom.tf * grav_vec.template cast<float>())
                    .normalized();
        }
    }

    PROFILING_NOTIFY2(traversibility_preproc, traversibility_gen_proc);

    this->trav_gen.processMapPoints(
        *buff.points,
        buff.search_min,
        buff.search_max,
        env_grav_vec,
        buff.base_to_odom.pose.vec);

    PROFILING_NOTIFY2(traversibility_gen_proc, traversibility_export);

    TraversibilityPointCloudType trav_points;
    TraversibilityMetaCloudType trav_meta;
    pcl::PointCloud<pcl::PointXYZINormal> trav_debug_cloud;

    this->trav_gen.swapPoints(trav_points);
    this->trav_gen.swapMetaDataList(trav_meta.points);

    trav_debug_cloud.points.resize(trav_points.size());
    for (size_t i = 0; i < trav_points.size(); i++)
    {
        auto& out = trav_debug_cloud.points[i];
        out.getVector3fMap() = trav_points.points[i].getVector3fMap();
        out.getNormalVector3fMap() = trav_meta.points[i].getNormalVector3fMap();
        out.intensity = trav_meta.points[i].trav_weight();
    }
    trav_debug_cloud.height = 1;
    trav_debug_cloud.width = trav_debug_cloud.points.size();
    trav_debug_cloud.is_dense = true;

    #if PATH_PLANNING_ENABLED
    {
        auto& x = this->mt.path_planning_resources.lockInput();
        x.stamp = buff.stamp;
        x.local_bound_min = buff.search_min;
        x.local_bound_max = buff.search_max;
        x.base_to_odom = buff.base_to_odom;
        x.trav_points.swap(trav_points);
        x.trav_meta.swap(trav_meta);
        this->mt.path_planning_resources.unlockInputAndNotify(x);
    }
    #endif

    PROFILING_NOTIFY2(traversibility_export, traversibility_debpub);

    try
    {
        PointCloudMsg output;
        pcl::toROSMsg(trav_debug_cloud, output);
        output.header.stamp = util::toTimeStamp(buff.stamp);
        output.header.frame_id = this->odom_frame;
        this->scan_pub.publish("traversability_points", output);
    }
    catch (const std::exception& e)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "[TRAVERSIBILITY]: Failed to publish debug cloud -- what():\n\t%s",
            e.what());
    }

    PROFILING_NOTIFY(traversibility_debpub);
}
#endif





#if PATH_PLANNING_ENABLED
void PerceptionNode::path_planning_callback_internal(
    PathPlanningResources& buff)
{
    using namespace util::geom::cvt::ops;

    if (buff.target.header.frame_id != this->odom_frame)
    {
        try
        {
            auto tf = this->tf_buffer.lookupTransform(
                this->odom_frame,
                buff.target.header.frame_id,
                util::toTf2TimePoint(buff.stamp));

            tf2::doTransform(buff.target, buff.target, tf);
        }
        catch (const std::exception& e)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "[PATH PLANNING CALLBACK]: Failed to transform target pose from '%s' to '%s'\n\twhat(): %s",
                buff.target.header.frame_id.c_str(),
                this->odom_frame.c_str(),
                e.what());
            return;
        }
    }

    Eigen::Vector3f odom_target;
    odom_target << buff.target.pose.position;

    std::vector<Eigen::Vector3f> path;

    if (!this->path_planner.solvePath(
            buff.base_to_odom.pose.vec,
            odom_target,
            buff.local_bound_min,
            buff.local_bound_max,
            buff.trav_points,
            buff.trav_meta,
            path))
    {
        RCLCPP_INFO(
            this->get_logger(),
            "[PATH PLANNING CALLBACK]: Failed to solve path");
        return;
    }

    PathMsg path_msg;
    path_msg.header.frame_id = this->odom_frame;
    path_msg.header.stamp = util::toTimeStamp(buff.stamp);

    path_msg.poses.reserve(path.size());
    for(const Eigen::Vector3f& kp : path)
    {
        PoseStampedMsg& pose = path_msg.poses.emplace_back();
        pose.pose.position << kp;
        pose.header.frame_id = this->odom_frame;
    }

    this->generic_pub.publish("planned_path", path_msg);

    RCLCPP_INFO(
        this->get_logger(),
        "[PATH PLANNING CALLBACK]: Published path with %lu keypoints.",
        path_msg.poses.size() );
}
#endif

};  // namespace perception
};  // namespace csm
