/*******************************************************************************
*   Copyright (C) 2024-2025 Cardinal Space Mining Club                         *
*                                                                              *
*   Unless required by applicable law or agreed to in writing, software        *
*   distributed under the License is distributed on an "AS IS" BASIS,          *
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
*   See the License for the specific language governing permissions and        *
*   limitations under the License.                                             *
*                                                                              *
*                                ;xxxxxxx:                                     *
*                               ;$$$$$$$$$       ...::..                       *
*                               $$$$$$$$$$x   .:::::::::::..                   *
*                            x$$$$$$$$$$$$$$::::::::::::::::.                  *
*                        :$$$$$&X;      .xX:::::::::::::.::...                 *
*                .$$Xx++$$$$+  :::.     :;:   .::::::.  ....  :                *
*               :$$$$$$$$$  ;:      ;xXXXXXXXx  .::.  .::::. .:.               *
*              :$$$$$$$$: ;      ;xXXXXXXXXXXXXx: ..::::::  .::.               *
*             ;$$$$$$$$ ::   :;XXXXXXXXXXXXXXXXXX+ .::::.  .:::                *
*              X$$$$$X : +XXXXXXXXXXXXXXXXXXXXXXXX; .::  .::::.                *
*               .$$$$ :xXXXXXXXXXXXXXXXXXXXXXXXXXXX.   .:::::.                 *
*                X$$X XXXXXXXXXXXXXXXXXXXXXXXXXXXXx:  .::::.                   *
*                $$$:.XXXXXXXXXXXXXXXXXXXXXXXXXXX  ;; ..:.                     *
*                $$& :XXXXXXXXXXXXXXXXXXXXXXXX;  +XX; X$$;                     *
*                $$$: XXXXXXXXXXXXXXXXXXXXXX; :XXXXX; X$$;                     *
*                X$$X XXXXXXXXXXXXXXXXXXX; .+XXXXXXX; $$$                      *
*                $$$$ ;XXXXXXXXXXXXXXX+  +XXXXXXXXx+ X$$$+                     *
*              x$$$$$X ;XXXXXXXXXXX+ :xXXXXXXXX+   .;$$$$$$                    *
*             +$$$$$$$$ ;XXXXXXx;;+XXXXXXXXX+    : +$$$$$$$$                   *
*              +$$$$$$$$: xXXXXXXXXXXXXXX+      ; X$$$$$$$$                    *
*               :$$$$$$$$$. +XXXXXXXXX;      ;: x$$$$$$$$$                     *
*               ;x$$$$XX$$$$+ .;+X+      :;: :$$$$$xX$$$X                      *
*              ;;;;;;;;;;X$$$$$$$+      :X$$$$$$&.                             *
*              ;;;;;;;:;;;;;x$$$$$$$$$$$$$$$$x.                                *
*              :;;;;;;;;;;;;.  :$$$$$$$$$$X                                    *
*               .;;;;;;;;:;;    +$$$$$$$$$                                     *
*                 .;;;;;;.       X$$$$$$$:                                     *
*                                                                              *
*******************************************************************************/

#pragma once

#include <mutex>
#include <type_traits>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "point_def.hpp"
#include "map_octree.hpp"


namespace csm
{
namespace perception
{

/** KDTree Frustum Collision (KFC) mapping implementation */
template<
    typename PointT = csm::perception::MappingPointType,
    typename MapT = csm::perception::MapOctree<PointT> >
class KDFrustumCollisionMap
{
    static_assert(std::is_base_of<csm::perception::MapOctree<PointT>, MapT>::value);

    using CollisionPointType = csm::perception::CollisionPointType;

public:
    KDFrustumCollisionMap(double voxel_size = 1.) : map_octree{ voxel_size } {}
    ~KDFrustumCollisionMap() = default;

public:
    void applyParams(
        double frustum_search_radius,
        double radial_dist_thresh,
        double delete_delta_coeff,
        double delete_max_range,
        double add_max_range);

    void addPoints(
        Eigen::Vector3f origin,
        const pcl::PointCloud<PointT>& pts,
        const pcl::Indices* indices = nullptr);

protected:
    pcl::KdTreeFLANN<CollisionPointType> collision_kdtree;
    pcl::PointCloud<CollisionPointType>::Ptr submap_ranges;
    MapT map_octree;

    std::mutex mtx;

    double
        frustum_search_radius,
        radial_dist_thresh,
        delete_delta_coeff,
        delete_max_range,
        add_max_range;

};


};
};
