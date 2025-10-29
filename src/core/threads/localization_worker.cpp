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

#include "localization_worker.hpp"

#include <memory>
#include <sstream>

#include <Eigen/Core>

#include <pcl/point_cloud.h>

#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cardinal_perception/msg/reflector_hint.hpp>
#include <cardinal_perception/msg/trajectory_filter_debug.hpp>

#include <csm_metrics/profiling.hpp>

#include <util.hpp>
#include <geometry.hpp>
#include <cloud_ops.hpp>


using namespace util::geom::cvt::ops;

using Vec3f = Eigen::Vector3f;
using Vec3d = Eigen::Vector3d;
using Mat4f = Eigen::Matrix4f;
using Iso3f = Eigen::Isometry3f;
using Quatf = Eigen::Quaternionf;
using Quatd = Eigen::Quaterniond;

using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
using TwistStampedMsg = geometry_msgs::msg::TwistStamped;
using TransformStampedMsg = geometry_msgs::msg::TransformStamped;

using ReflectorHintMsg = cardinal_perception::msg::ReflectorHint;
using TrajectoryFilterDebugMsg = cardinal_perception::msg::TrajectoryFilterDebug;


namespace csm
{
namespace perception
{

LocalizationWorker::LocalizationWorker(
    RclNode& node,
    const Tf2Buffer& tf_buffer,
    const ImuIntegrator<>& imu_sampler) :
    node{node},
    tf_buffer{tf_buffer},
    imu_sampler{imu_sampler},
    tf_broadcaster{node},
    pub_map{&node, PERCEPTION_TOPIC(""), PERCEPTION_PUBSUB_QOS},
    lidar_odom{node},
    transform_sync{tf_broadcaster}
{
}

LocalizationWorker::~LocalizationWorker() { this->stopThreads(); }

void LocalizationWorker::configure(
    const std::string& map_frame,
    const std::string& odom_frame,
    const std::string& base_frame)
{
    this->map_frame = map_frame;
    this->odom_frame = odom_frame;
    this->base_frame = base_frame;
    this->transform_sync.setFrameIds(
        map_frame,
        odom_frame,
        base_frame);
}

void LocalizationWorker::accept(const PointCloudMsg::ConstSharedPtr& msg)
{
    this->odometry_resources.updateAndNotify(msg);
}

#if TAG_DETECTION_ENABLED
void LocalizationWorker::accept(const TagsTransformMsg::ConstSharedPtr& msg)
{
    PROFILING_SYNC();
    PROFILING_NOTIFY_ALWAYS(tags_detection);

    constexpr int tag_usage_mode =
        1;  // TODO: parameter (if we ever use this again)

    const TransformStampedMsg& tf = msg->estimated_tf;
    if (tf.header.frame_id == this->map_frame &&
        tf.child_frame_id == this->base_frame &&
        (tag_usage_mode > 0 || (tag_usage_mode < 0 && msg->filter_mask >= 31)))
    {
        TagDetection::Ptr td = std::make_shared<TagDetection>();
        td->pose << tf.transform;
        td->time_point = util::toFloatSeconds(tf.header.stamp);
        td->pix_area = msg->pix_area;
        td->avg_range = msg->avg_range;
        td->rms = msg->rms;
        td->num_tags = msg->num_tags;
        this->transform_sync.endMeasurementIterationSuccess(td, td->time_point);
    }

    PROFILING_NOTIFY_ALWAYS(tags_detection);
}
#endif

void LocalizationWorker::connectOutput(
    ResourcePipeline<MappingResources>& mapping_resources)
{
    this->mapping_resources = &mapping_resources;
}

void LocalizationWorker::startThreads()
{
    if (!this->threads_running)
    {
        this->threads_running = true;
        this->odometry_thread =
            std::thread{&LocalizationWorker::odom_thread_worker, this};
#if LFD_ENABLED
        this->fiducial_thread =
            std::thread{&LocalizationWorker::fiducial_thread_worker, this};
#endif
    }
}

void LocalizationWorker::stopThreads()
{
    this->threads_running = false;

    this->odometry_resources.notifyExit();
    if (this->odometry_thread.joinable())
    {
        this->odometry_thread.join();
    }

#if LFD_ENABLED
    this->fiducial_resources.notifyExit();
    if (this->fiducial_thread.joinable())
    {
        this->fiducial_thread.join();
    }
#endif
}


void LocalizationWorker::odom_thread_worker()
{
    RCLCPP_INFO(
        this->node.get_logger(),
        "[CORE]: Odometry thread started successfully.");

    do
    {
        auto& scan = this->odometry_resources.waitNewestResource();
        if (!this->threads_running.load())
        {
            return;
        }

        PROFILING_SYNC();
        PROFILING_NOTIFY_ALWAYS(odometry);
        this->scan_callback(scan);
        PROFILING_NOTIFY_ALWAYS(odometry);
    }  //
    while (this->threads_running.load());

    RCLCPP_INFO(
        this->node.get_logger(),
        "[CORE]: Odometry thread exited cleanly");
}

void LocalizationWorker::scan_callback(
    const PointCloudMsg::ConstSharedPtr& scan)
{
    PROFILING_NOTIFY(odometry_init);

    pcl::PointCloud<OdomPointType> lo_cloud;
    util::geom::PoseTf3f lidar_to_base_tf, base_to_odom_tf;

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
        this->imu_sampler);

    this->scan_preproc.swapOutputPoints(lo_cloud);
    lidar_to_base_tf.pose
        << (lidar_to_base_tf.tf = this->scan_preproc.getTargetTf());

    PROFILING_NOTIFY2(odometry_preprocess, odometry_lfd_export);

    const uint32_t iteration_token =
        this->transform_sync.beginOdometryIteration();

#if LFD_ENABLED
    // 2. Export LFD Resources (if enabled) ------------------------------------
    FiducialResources& f = this->fiducial_resources.lockInput();
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
    this->fiducial_resources.unlockInputAndNotify(f);
#else
    (void)iteration_token;
#endif

    PROFILING_NOTIFY2(odometry_lfd_export, odometry_lio);

    // apply sensor origin
    lo_cloud.sensor_origin_ << lidar_to_base_tf.pose.vec, 1.f;
    const double new_odom_stamp = util::toFloatSeconds(scan->header.stamp);

    // get imu estimated rotation
    Quatd imu_rot_q = Quatd::Identity();
    Mat4f imu_rot = Mat4f::Identity();
    if (this->imu_sampler.hasSamples())
    {
        imu_rot_q = this->imu_sampler.getDelta(
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

    // 4. Export Mapping Resources (if connected) ------------------------------
    if (this->mapping_resources)
    {
        MappingResources& m = this->mapping_resources->lockInput();
        m.lidar_to_base = lidar_to_base_tf;
        m.base_to_odom = base_to_odom_tf;
        m.raw_scan = scan;
        m.lidar_odom_buff.swap(lo_cloud);
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
        this->mapping_resources->unlockInputAndNotify(m);
    }

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
    odom_vel.header.stamp = scan->header.stamp;
    this->pub_map.publish("odom_velocity", odom_vel);

    // Publish LO debug
#if PERCEPTION_PUBLISH_LIO_DEBUG > 0
    this->lidar_odom.publishDebugScans(
        this->pub_map,
        lo_status,
        this->odom_frame);
    this->lidar_odom.publishDebugMetrics(this->pub_map);
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
    this->pub_map.publish("metrics/trajectory_filter_stats", dbg);
#endif

    PROFILING_NOTIFY(odometry_debpub);
}


#if LFD_ENABLED
void LocalizationWorker::fiducial_thread_worker()
{
    RCLCPP_INFO(
        this->node.get_logger(),
        "[CORE]: Lidar fiducial detection thread started successfully.");

    do
    {
        auto& buff = this->fiducial_resources.waitNewestResource();
        if (!this->threads_running.load())
        {
            return;
        }

        PROFILING_SYNC();
        PROFILING_NOTIFY_ALWAYS(lidar_fiducial);
        this->fiducial_callback(buff);
        PROFILING_NOTIFY_ALWAYS(lidar_fiducial);
    }  //
    while (this->threads_running.load());

    RCLCPP_INFO(
        this->node.get_logger(),
        "[CORE]: Lidar fiducial detection thread exited cleanly.");
}

void LocalizationWorker::fiducial_callback(FiducialResources& buff)
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
        this->imu_sampler.estimateGravity(0.5, &stddev, &delta_r);

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

        this->pub_map.publish("poses/fiducial_tag_pose", p);
    }
    else
    {
        this->transform_sync.endMeasurementIterationFailure();
    }

    hint_msg.centroid.point << (buff.lidar_to_base.tf * centroid);
    hint_msg.centroid.header.stamp = buff.raw_scan->header.stamp;
    hint_msg.centroid.header.frame_id = this->base_frame;
    this->pub_map.publish("reflector_hint", hint_msg);

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
        this->pub_map.publish("fiducial_input_points", _pc);

        pcl::toROSMsg(redetect_cloud, _pc);
        _pc.header = buff.raw_scan->header;
        this->pub_map.publish("fiducial_redetect_points", _pc);

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
            this->pub_map.publish(topic, _p);

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
            this->pub_map.publish(topic, _pc);
        }

        if (result.iterations == 3 && !remaining_points.empty())
        {
            pcl::toROSMsg(remaining_points, _pc);
            _pc.header = buff.raw_scan->header;

            this->pub_map.publish("fiducial_unmodeled_points", _pc);
        }
        // }
    }
    catch (const std::exception& e)
    {
        RCLCPP_INFO(
            this->node.get_logger(),
            "[FIDUCIAL DETECTION]: Failed to publish debug data -- what():\n\t%s",
            e.what());
    }
    #endif

    PROFILING_NOTIFY(lfd_debpub);
}
#endif

};  // namespace perception
};  // namespace csm
