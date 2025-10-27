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

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <csm_metrics/profiling.hpp>

#include <cardinal_perception/msg/reflector_hint.hpp>
#include <cardinal_perception/msg/mining_eval_results.hpp>

#include <imu_transform.hpp>
#include <cloud_ops.hpp>


using namespace util::geom::cvt::ops;

using Vec3f = Eigen::Vector3f;
using Vec3d = Eigen::Vector3d;
using Mat4f = Eigen::Matrix4f;
using Iso3f = Eigen::Isometry3f;
using Quatf = Eigen::Quaternionf;
using Quatd = Eigen::Quaterniond;

using PathMsg = nav_msgs::msg::Path;
using TwistStampedMsg = geometry_msgs::msg::TwistStamped;

using ReflectorHintMsg = cardinal_perception::msg::ReflectorHint;
using MiningEvalResultsMsg = cardinal_perception::msg::MiningEvalResults;


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
    Vec3d grav_vec = this->imu_samples.estimateGravity(1., &stddev, &delta_r);

    PoseStampedMsg grav_pub;
    grav_pub.header.stamp = imu->header.stamp;
    grav_pub.header.frame_id = this->base_frame;
    grav_pub.pose.orientation
        << Quatd::FromTwoVectors(Vec3d{1., 0., 0.}, grav_vec);
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





// --- INTERNAL CALLBACK WORKERS ----------------------------------------------

void PerceptionNode::scan_callback_internal(
    const PointCloudMsg::ConstSharedPtr& scan)
{
    PROFILING_NOTIFY(odometry_init);

    OdomPointCloudType lo_cloud;
    util::geom::PoseTf3f lidar_to_base_tf, base_to_odom_tf;
    const auto scan_stamp = scan->header.stamp;

    PROFILING_NOTIFY2(odometry_init, odometry_preprocess);

    // 1. Preprocess Scan ------------------------------------------------------
    constexpr int Preproc_Config =
        (decltype(this->scan_preproc)::PREPROC_EXCLUDE_ZONES |
         (decltype(this->scan_preproc)::PREPROC_PERFORM_DESKEW *
          (PERCEPTION_USE_SCAN_DESKEW != 0)) |
         (decltype(this->scan_preproc)::PREPROC_EXPORT_NULL_RAYS *
          (PERCEPTION_USE_NULL_RAY_DELETION != 0)));
    this->scan_preproc.process<Preproc_Config>(
        *scan,
        this->base_frame,
        this->tf_buffer,
        this->imu_samples);

    this->scan_preproc.swapOutputPoints(lo_cloud);
    lidar_to_base_tf.pose
        << (lidar_to_base_tf.tf = this->scan_preproc.getTargetTf());

    PROFILING_NOTIFY2(odometry_preprocess, odometry_lfd_export);

    const uint32_t iteration_token =
        this->transform_sync.beginOdometryIteration();

#if LFD_ENABLED
    // 2. Export LFD Resources (if enabled) ------------------------------------
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
    this->scan_preproc.swapNullIndices(
        const_cast<pcl::Indices&>(*f.nan_indices));
    this->scan_preproc.swapRemovalIndices(
        const_cast<pcl::Indices&>(*f.remove_indices));
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
    Quatd imu_rot_q = Quatd::Identity();
    Mat4f imu_rot = Mat4f::Identity();
    if (this->imu_samples.hasSamples())
    {
        imu_rot_q = this->imu_samples.getDelta(
            this->lidar_odom.prevStamp(),
            new_odom_stamp);
        imu_rot.block<3, 3>(0, 0) =
            imu_rot_q.template cast<float>().toRotationMatrix();
    }

    // 3. Iterate LIO ----------------------------------------------------------
    auto lo_status = this->lidar_odom.processScan(
        lo_cloud,
        new_odom_stamp,
        base_to_odom_tf,
        imu_rot);

    PROFILING_NOTIFY(odometry_lio);

    if (!lo_status)
    {
        this->transform_sync.endOdometryIterationFailure();
        return;
    }

    PROFILING_NOTIFY(odometry_map_export);

#if MAPPING_ENABLED
    // 4. Export Mapping Resources (if enabled) --------------------------------
    MappingResources& m = this->mt.mapping_resources.lockInput();
    m.lidar_to_base = lidar_to_base_tf;
    m.base_to_odom = base_to_odom_tf;
    m.raw_scan = scan;
    m.lo_buff.swap(lo_cloud);
    #if PERCEPTION_USE_NULL_RAY_DELETION
    this->scan_preproc.swapNullRays(m.null_vecs);
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
    this->scan_preproc.swapNullIndices(
        const_cast<pcl::Indices&>(*m.nan_indices));
    this->scan_preproc.swapRemovalIndices(
        const_cast<pcl::Indices&>(*m.remove_indices));
    #endif
    this->mt.mapping_resources.unlockInputAndNotify(m);
#endif

    PROFILING_NOTIFY2(odometry_map_export, odometry_debpub);

    // 5. Update Transforms ----------------------------------------------------
    util::geom::PoseTf3d prev_odom_tf, new_odom_tf;
    const double prev_odom_stamp = this->transform_sync.getOdomTf(prev_odom_tf);

    // Update odom tf
    this->transform_sync.endOdometryIterationSuccess(
        base_to_odom_tf,
        new_odom_stamp);

    // 6. Publish Debug Data ---------------------------------------------------
    // Publish velocity
    const double t_diff =
        this->transform_sync.getOdomTf(new_odom_tf) - prev_odom_stamp;
    Quatd odom_rot_q = prev_odom_tf.pose.quat.inverse() * new_odom_tf.pose.quat;
    Vec3d l_vel = (new_odom_tf.pose.vec - prev_odom_tf.pose.vec) / t_diff;
    Vec3d r_vel = (odom_rot_q.toRotationMatrix().eulerAngles(0, 1, 2) / t_diff);

    TwistStampedMsg odom_vel;
    odom_vel.twist.linear << l_vel;
    odom_vel.twist.angular << r_vel;
    odom_vel.header.frame_id = this->odom_frame;
    odom_vel.header.stamp = scan_stamp;
    this->generic_pub.publish("odom_velocity", odom_vel);

    // this->generic_pub.publish(
    //     "odom_rot_error",
    //     Float64Msg{}.set__data(imu_rot_q.angularDistance(odom_rot_q)));

    // Publish LO debug
#if PERCEPTION_PUBLISH_LIO_DEBUG > 0
    this->lidar_odom.publishDebugScans(
        this->generic_pub,
        lo_status,
        this->odom_frame);
    this->lidar_odom.publishDebugMetrics(this->generic_pub);
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





#if LFD_ENABLED
void PerceptionNode::fiducial_callback_internal(FiducialResources& buff)
{
    PROFILING_NOTIFY(lfd_preprocess);

    this->transform_sync.beginMeasurementIteration(buff.iteration_count);

    pcl::PointCloud<FiducialPointType> reflector_points;
    pcl::fromROSMsg(*buff.raw_scan, reflector_points);

    util::pc_remove_selection(reflector_points, *buff.remove_indices);
    // signals to odom thread that these buffers can be reused
    buff.nan_indices.reset();
    buff.remove_indices.reset();

    double stddev, delta_r;
    const Vec3d grav_vec =
        this->imu_samples.estimateGravity(0.5, &stddev, &delta_r);

    Vec3f up_vec{0, 0, 1};
    // if (stddev < 1. && delta_r < 0.01)
    {
        up_vec = grav_vec.template cast<float>();
    }
    Iso3f base_to_lidar_tf = buff.lidar_to_base.tf.inverse();
    up_vec = (base_to_lidar_tf * up_vec);
    Vec3f base_pt = base_to_lidar_tf.translation();

    util::geom::PoseTf3f fiducial_pose;
    ReflectorHintMsg hint_msg;
    Vec3f centroid;
    PROFILING_NOTIFY2(lfd_preprocess, lfd_calculate);
    auto result = this->fiducial_detector.calculatePose(
        fiducial_pose.pose,
        reflector_points,
        &up_vec,
        &base_pt);
    hint_msg.samples = this->fiducial_detector.calculateReflectiveCentroid(
        centroid,
        hint_msg.variance);
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

    hint_msg.centroid.point << (buff.lidar_to_base.tf * centroid);
    hint_msg.centroid.header.stamp = buff.raw_scan->header.stamp;
    hint_msg.centroid.header.frame_id = this->base_frame;
    this->generic_pub.publish("reflector_hint", hint_msg);

    PROFILING_NOTIFY2(lfd_export, lfd_debpub);

    #if PERCEPTION_PUBLISH_LFD_DEBUG > 0
    try
    {
        PoseStampedMsg _p;
        PointCloudMsg _pc;

        const auto& input_cloud = this->fiducial_detector.getInputPoints();
        const auto& redetect_cloud =
            this->fiducial_detector.getRedetectPoints();

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
            _p.pose.orientation << Quatf::FromTwoVectors(
                Vec3f{1.f, 0.f, 0.f},
                seg_planes[i].head<3>());

            std::string topic =
                (std::ostringstream{} << "fiducial_plane_" << i << "/pose")
                    .str();
            this->pose_pub.publish(topic, _p);

            if (i < result.iterations)
            {
                pcl::toROSMsg(seg_clouds[i], _pc);
            }
            else
            {
                pcl::PointCloud<pcl::PointXYZ> empty_cloud;
                pcl::toROSMsg(empty_cloud, _pc);
            }
            _pc.header = buff.raw_scan->header;

            topic =
                ((std::ostringstream{} << "fiducial_plane_" << i << "/points")
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
        pcl_conversions::toPCL(buff.raw_scan->header, buff.lo_buff.header);
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

    if (this->param.map_crop_horizontal_range > 0. &&
        this->param.map_crop_vertical_range > 0.)
    {
        const Vec3f crop_range{
            static_cast<float>(this->param.map_crop_horizontal_range),
            static_cast<float>(this->param.map_crop_horizontal_range),
            static_cast<float>(this->param.map_crop_vertical_range)};
        this->sparse_map.setBounds(
            buff.base_to_odom.pose.vec - crop_range,
            buff.base_to_odom.pose.vec + crop_range);
    }

    #if PERCEPTION_USE_NULL_RAY_DELETION
    for (RayDirectionType& r : buff.null_vecs)
    {
        r.getNormalVector4fMap() =
            lidar_to_odom_tf.tf * r.getNormalVector4fMap();
    }

    PROFILING_NOTIFY2(mapping_preproc, mapping_kfc_update);

    auto results = this->sparse_map.updateMap(
        lidar_to_odom_tf.pose.vec,
        *filtered_scan_t,
        buff.null_vecs);
    #else

    PROFILING_NOTIFY2(mapping_preproc, mapping_kfc_update);

    auto results =
        this->sparse_map.updateMap(lidar_to_odom_tf.pose.vec, *filtered_scan_t);
    #endif

    if (this->param.map_export_horizontal_range > 0. &&
        this->param.map_export_vertical_range > 0.)
    {
        PROFILING_NOTIFY2(mapping_kfc_update, mapping_search_local);

        pcl::Indices export_points;
        const Vec3f search_range{
            static_cast<float>(this->param.map_export_horizontal_range),
            static_cast<float>(this->param.map_export_horizontal_range),
            static_cast<float>(this->param.map_export_vertical_range)};
        const Vec3f search_min{buff.base_to_odom.pose.vec - search_range};
        const Vec3f search_max{buff.base_to_odom.pose.vec + search_range};

        this->sparse_map.getMap().boxSearch(
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
            util::pc_copy_selection(
                *this->sparse_map.getPoints(),
                export_points,
                x.points);
            x.stamp = util::toFloatSeconds(buff.raw_scan->header.stamp);
            this->mt.traversibility_resources.unlockInputAndNotify(x);
        }
    #else
        try
        {
            thread_local pcl::PointCloud<MappingPointType> trav_points;
            util::pc_copy_selection(
                *this->sparse_map.getPoints(),
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
    }
    else
    {
        PROFILING_NOTIFY2(mapping_kfc_update, mapping_debpub);
    }

    try
    {
        pcl::PointCloud<pcl::PointNormal> output_buff;
        const auto& map_pts = *this->sparse_map.getPoints();
        const auto& map_norms = this->sparse_map.getMap().pointNormals();

        output_buff.points.resize(map_pts.size());
        for (size_t i = 0; i < output_buff.size(); i++)
        {
            output_buff.points[i].getVector3fMap() =
                map_pts.points[i].getVector3fMap();
            output_buff.points[i].getNormalVector3fMap() =
                map_norms[i].template head<3>();
            output_buff.points[i].curvature = map_norms[i][4];
        }
        output_buff.height = map_pts.height;
        output_buff.width = map_pts.width;
        output_buff.is_dense = map_pts.is_dense;

        PointCloudMsg output;
    #if PERCEPTION_PUBLISH_FULL_MAP > 0
        pcl::toROSMsg(output_buff, output);
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

    thread_local Vec3f env_grav_vec{0.f, 0.f, 1.f};
    if (this->imu_samples.hasSamples())
    {
        double stddev, delta_r;
        const Vec3d grav_vec =
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
        buff.points,
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
        out.curvature = trav_meta.points[i].curvature;
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
                // util::toTf2TimePoint(buff.stamp));
                tf2::timeFromSec(0));

            // ensure tf stamp is not wildly out of date

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

    Vec3f odom_target;
    odom_target << buff.target.pose.position;

    std::vector<Vec3f> path;

    if (!this->path_planner.solvePath(
            buff.base_to_odom.pose.vec,
            odom_target,
            buff.local_bound_min,
            buff.local_bound_max,
            buff.trav_points,
            buff.trav_meta,
            path))
    {
        return;
    }

    PathMsg path_msg;
    path_msg.header.frame_id = this->odom_frame;
    path_msg.header.stamp = util::toTimeStamp(buff.stamp);

    path_msg.poses.reserve(path.size());
    for (const Vec3f& kp : path)
    {
        PoseStampedMsg& pose = path_msg.poses.emplace_back();
        pose.pose.position << kp;
        pose.header.frame_id = this->odom_frame;
    }

    this->generic_pub.publish("planned_path", path_msg);

    // RCLCPP_INFO(
    //     this->get_logger(),
    //     "[PATH PLANNING CALLBACK]: Published path with %lu keypoints.",
    //     path_msg.poses.size());
}
#endif

};  // namespace perception
};  // namespace csm
