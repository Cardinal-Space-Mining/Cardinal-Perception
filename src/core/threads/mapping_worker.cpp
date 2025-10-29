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

#include "mapping_worker.hpp"

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <csm_metrics/profiling.hpp>

#include <util.hpp>
#include <geometry.hpp>
#include <cloud_ops.hpp>


using namespace util::geom::cvt::ops;

using Vec3f = Eigen::Vector3f;

using PointCloudMsg = sensor_msgs::msg::PointCloud2;


namespace csm
{
namespace perception
{

MappingWorker::MappingWorker(RclNode& node) :
    node{node},
    pub_map{&node, PERCEPTION_TOPIC(""), PERCEPTION_PUBSUB_QOS}
{
}

MappingWorker::~MappingWorker() { this->stopThreads(); }

void MappingWorker::configure(
    const std::string& odom_frame,
    double map_crop_horizontal_range,
    double map_crop_vertical_range,
    double map_export_horizontal_range,
    double map_export_vertical_range)
{
    this->odom_frame = odom_frame;
    this->map_crop_horizontal_range = map_crop_horizontal_range;
    this->map_crop_vertical_range = map_crop_vertical_range;
    this->map_export_horizontal_range = map_export_horizontal_range;
    this->map_export_vertical_range = map_export_vertical_range;
}

ResourcePipeline<MappingResources>& MappingWorker::getInput()
{
    return this->mapping_resources;
}
void MappingWorker::connectOutput(
    ResourcePipeline<TraversibilityResources>& traversibility_resources)
{
    this->traversibility_resources = &traversibility_resources;
}

void MappingWorker::startThreads()
{
    if (!this->threads_running)
    {
        this->threads_running = true;
        this->mapping_thread =
            std::thread{&MappingWorker::mapping_thread_worker, this};
    }
}

void MappingWorker::stopThreads()
{
    this->threads_running = false;

    this->mapping_resources.notifyExit();
    if (this->mapping_thread.joinable())
    {
        this->mapping_thread.join();
    }
}

void MappingWorker::mapping_thread_worker()
{
    RCLCPP_INFO(
        this->node.get_logger(),
        "[CORE]: Mapping thread started successfully.");

    do
    {
        auto& buff = this->mapping_resources.waitNewestResource();
        if (!this->threads_running.load())
        {
            return;
        }

        PROFILING_SYNC();
        PROFILING_NOTIFY_ALWAYS(mapping);
        this->mapping_callback(buff);
        PROFILING_NOTIFY_ALWAYS(mapping);
    }  //
    while (this->threads_running.load());

    RCLCPP_INFO(
        this->node.get_logger(),
        "[CORE]: Mapping thread exited cleanly.");
}


void MappingWorker::mapping_callback(MappingResources& buff)
{
    PROFILING_NOTIFY(mapping_preproc);

    // RCLCPP_INFO(this->get_logger(), "MAPPING CALLBACK INTERNAL");

    util::geom::PoseTf3f lidar_to_odom_tf;
    lidar_to_odom_tf.pose
        << (lidar_to_odom_tf.tf = buff.base_to_odom.tf * buff.lidar_to_base.tf);

    pcl::PointCloud<MappingPointType>* filtered_scan_t = nullptr;
    if constexpr (std::is_same<OdomPointType, MappingPointType>::value)
    {
        pcl_conversions::toPCL(
            buff.raw_scan->header,
            buff.lidar_odom_buff.header);
        pcl::transformPointCloud(
            buff.lidar_odom_buff,
            buff.lidar_odom_buff,
            buff.base_to_odom.tf,
            true);
        filtered_scan_t = &buff.lidar_odom_buff;
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

    if (this->map_crop_horizontal_range > 0. &&
        this->map_crop_vertical_range > 0.)
    {
        const Vec3f crop_range{
            static_cast<float>(this->map_crop_horizontal_range),
            static_cast<float>(this->map_crop_horizontal_range),
            static_cast<float>(this->map_crop_vertical_range)};
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

    if (this->map_export_horizontal_range > 0. &&
        this->map_export_vertical_range > 0.)
    {
        PROFILING_NOTIFY2(mapping_kfc_update, mapping_search_local);

        pcl::Indices export_points;
        const Vec3f search_range{
            static_cast<float>(this->map_export_horizontal_range),
            static_cast<float>(this->map_export_horizontal_range),
            static_cast<float>(this->map_export_vertical_range)};
        const Vec3f search_min{buff.base_to_odom.pose.vec - search_range};
        const Vec3f search_max{buff.base_to_odom.pose.vec + search_range};

        this->sparse_map.getMap().boxSearch(
            search_min,
            search_max,
            export_points);

        PROFILING_NOTIFY2(mapping_search_local, mapping_export_trav);

        if (this->traversibility_resources)
        {
            auto& x = this->traversibility_resources->lockInput();
            x.bounds_min = search_min;
            x.bounds_max = search_max;
            x.lidar_to_base = buff.lidar_to_base;
            x.base_to_odom = buff.base_to_odom;
            util::pc_copy_selection(
                *this->sparse_map.getPoints(),
                export_points,
                x.points);
            x.stamp = util::toFloatSeconds(buff.raw_scan->header.stamp);
            this->traversibility_resources->unlockInputAndNotify(x);
        }
        else
        {
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
                this->pub_map.publish(
                    "traversibility_points",
                    trav_points_output);
            }
            catch (const std::exception& e)
            {
                RCLCPP_INFO(
                    this->node.get_logger(),
                    "[MAPPING]: Failed to publish traversibility subcloud -- what():\n\t%s",
                    e.what());
            }
        }

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
        this->pub_map.publish("map_cloud", output);
#endif

        pcl::toROSMsg(*filtered_scan_t, output);
        output.header.stamp = buff.raw_scan->header.stamp;
        output.header.frame_id = this->odom_frame;
        this->pub_map.publish("filtered_scan", output);
    }
    catch (const std::exception& e)
    {
        RCLCPP_INFO(
            this->node.get_logger(),
            "[MAPPING]: Failed to publish mapping debug scans -- what():\n\t%s",
            e.what());
    }

    this->pub_map.publish<std_msgs::msg::Float64>(
        "metrics/mapping/search_pointset",
        static_cast<double>(results.points_searched));
    this->pub_map.publish<std_msgs::msg::Float64>(
        "metrics/mapping/points_deleted",
        static_cast<double>(results.points_deleted));

    PROFILING_NOTIFY(mapping_debpub);
}

};  // namespace perception
};  // namespace csm
