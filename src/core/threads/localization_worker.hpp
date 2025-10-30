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

#include <config.hpp>

#include <atomic>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cardinal_perception/msg/tags_transform.hpp>

#include <modules/lidar_odom.hpp>
#include <modules/lf_detector.hpp>

#include <pub_map.hpp>
#include <imu_integrator.hpp>
#include <transform_sync.hpp>
#include <synchronization.hpp>
#include <scan_preprocessor.hpp>

#include "shared_resources.hpp"
#include "../perception_presets.hpp"


namespace csm
{
namespace perception
{

class LocalizationWorker
{
    friend class PerceptionNode;

    using RclNode = rclcpp::Node;
    using Tf2Buffer = tf2_ros::Buffer;
    using Tf2Broadcaster = tf2_ros::TransformBroadcaster;

    using PointCloudMsg = sensor_msgs::msg::PointCloud2;
    using TagsTransformMsg = cardinal_perception::msg::TagsTransform;

public:
    LocalizationWorker(
        RclNode& node,
        const Tf2Buffer& tf_buffer,
        const ImuIntegrator<>& imu_sampler);
    ~LocalizationWorker();

public:
    void configure(
        const std::string& map_frame,
        const std::string& odom_frame,
        const std::string& base_frame);

    void accept(const PointCloudMsg::ConstSharedPtr& msg);
#if TAG_DETECTION_ENABLED
    void accept(const TagsTransformMsg::ConstSharedPtr& msg);
#endif
    bool setGlobalAlignmentEnabled(bool enable);

    void connectOutput(ResourcePipeline<MappingResources>& mapping_resources);

    void startThreads();
    void stopThreads();

protected:
#if TAG_DETECTION_ENABLED
    struct TagDetection
    {
        using Ptr = std::shared_ptr<TagDetection>;
        using ConstPtr = std::shared_ptr<const TagDetection>;

        util::geom::Pose3d pose;

        double time_point, pix_area, avg_range, rms;
        size_t num_tags;

        inline operator util::geom::Pose3d&() { return this->pose; }
    };
#endif
#if LFD_ENABLED
    struct FiducialResources
    {
        util::geom::PoseTf3f lidar_to_base;
        PointCloudMsg::ConstSharedPtr raw_scan;
        std::shared_ptr<const pcl::Indices> nan_indices, remove_indices;
        uint32_t iteration_count;
    };
#endif

protected:
    void odom_thread_worker();
    void scan_callback(const PointCloudMsg::ConstSharedPtr& scan);
#if LFD_ENABLED
    void fiducial_thread_worker();
    void fiducial_callback(FiducialResources& buff);
#endif

protected:
    RclNode& node;
    const Tf2Buffer& tf_buffer;
    const ImuIntegrator<>& imu_sampler;
    Tf2Broadcaster tf_broadcaster;
    util::GenericPubMap pub_map;

    std::string map_frame;
    std::string odom_frame;
    std::string base_frame;

    std::atomic<bool> threads_running{false};
    std::atomic<bool> global_alignment_enabled{true};

    ScanPreprocessor<
        OdomPointType,
        RayDirectionType,
        SphericalDirectionPointType,
        TimestampPointType>
        scan_preproc;
    LidarOdometry<OdomPointType> lidar_odom;
#if LFD_ENABLED
    LidarFiducialDetector<FiducialPointType> fiducial_detector;
#endif
#if TAG_DETECTION_ENABLED
    TransformSynchronizer<TagDetection> transform_sync;
#else
    TransformSynchronizer<util::geom::Pose3d> transform_sync;
#endif

    ResourcePipeline<PointCloudMsg::ConstSharedPtr> odometry_resources;
    std::thread odometry_thread;
#if LFD_ENABLED
    ResourcePipeline<FiducialResources> fiducial_resources;
    std::thread fiducial_thread;
#endif
    ResourcePipeline<MappingResources>* mapping_resources{nullptr};
};

};  // namespace perception
};  // namespace csm
