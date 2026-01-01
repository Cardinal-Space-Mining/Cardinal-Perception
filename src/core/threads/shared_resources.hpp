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

#include <memory>
#include <vector>

#include <Eigen/Core>

#include <pcl/point_cloud.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <config.hpp>
#include <util/geometry.hpp>


namespace csm
{
namespace perception
{

struct MappingResources
{
    util::geom::PoseTf3f lidar_to_base;
    util::geom::PoseTf3f base_to_odom;
    sensor_msgs::msg::PointCloud2::ConstSharedPtr raw_scan;
    pcl::PointCloud<OdomPointType> lidar_odom_buff;
#if PERCEPTION_USE_NULL_RAY_DELETION
    std::vector<RayDirectionType> null_vecs;
#endif
    std::shared_ptr<const pcl::Indices> nan_indices;
    std::shared_ptr<const pcl::Indices> remove_indices;
};

struct TraversibilityResources
{
    double stamp;
    Eigen::Vector3f bounds_min;
    Eigen::Vector3f bounds_max;
    util::geom::PoseTf3f lidar_to_base;
    util::geom::PoseTf3f base_to_odom;
    pcl::PointCloud<MappingPointType> points;
};

struct PathPlanningResources
{
    double stamp;
    Eigen::Vector3f bounds_min;
    Eigen::Vector3f bounds_max;
    util::geom::PoseTf3f base_to_odom;
    geometry_msgs::msg::PoseStamped target;
    pcl::PointCloud<TraversibilityPointType> points;
};

struct MiningEvalResources
{
    double stamp;
    Eigen::Vector3f bounds_min;
    Eigen::Vector3f bounds_max;
    pcl::PointCloud<TraversibilityPointType> avoid_points;
    std::shared_ptr<void> query;
};

};  // namespace perception
};  // namespace csm
