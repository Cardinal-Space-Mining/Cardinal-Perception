/*******************************************************************************
*   Copyright (C) 2024-2026 Cardinal Space Mining Club                         *
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

#include <type_traits>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <util/cloud_ops.hpp>
#include <traversibility_def.hpp>

#include "ue_octree.hpp"
#include "map_octree.hpp"


namespace csm
{
namespace perception
{

template<typename Point_T = pcl::PointXYZI>
class PathPlanMap
{
    static_assert(util::traits::supports_traversibility<Point_T>::value);

public:
    using PointT = Point_T;
    using PointCloudT = pcl::PointCloud<PointT>;

    using UEOctreeT = UEOctree<float>;
    using MapOctreeT = MapOctree<PointT>;

    using Vec3f = Eigen::Vector3f;
    using Arr3f = Eigen::Array3f;

public:
    PathPlanMap(float voxel_size = 0.1f);
    PathPlanMap(const PathPlanMap&) = delete;
    ~PathPlanMap() = default;

public:
    void clear();
    void reconfigure(
        float obstacle_merge_window,
        float voxel_size = -1.f);

    void append(
        const PointCloudT& pts,
        const Vec3f& bound_min,
        const Vec3f& bound_max);

    void crop(const Vec3f& min, const Vec3f& max);

    const PointCloudT& getPoints() const;
    const UEOctreeT& getUESpace() const;
    const MapOctreeT& getMap() const;

protected:
    UEOctreeT ue_octree;
    MapOctreeT map_octree;

    float obstacle_merge_window{0.5f};
};



// --- Implementation ----------------------------------------------------------

template<typename P>
PathPlanMap<P>::PathPlanMap(float vox_sz) :
    ue_octree{vox_sz},
    map_octree{vox_sz}
{
}

template<typename P>
void PathPlanMap<P>::clear()
{
    this->ue_octree.clear();
    this->map_octree.clear();
}

template<typename P>
void PathPlanMap<P>::reconfigure(
    float obstacle_merge_window,
    float voxel_size)
{
    this->obstacle_merge_window = obstacle_merge_window;

    if (voxel_size > 0.f)
    {
        this->ue_octree.reconfigure(voxel_size);
        this->map_octree.reconfigure(voxel_size);
    }
}

template<typename P>
void PathPlanMap<P>::append(
    const PointCloudT& pts,
    const Vec3f& bound_min,
    const Vec3f& bound_max)
{
    // TODO: frame-to-frame "bounds alignment" for more condistent map

    using namespace csm::perception::traversibility;

    const float res = static_cast<float>(this->map_octree.getResolution());

    pcl::Indices tmp_indices;
    std::vector<float> tmp_dists;
    typename PointCloudT::VectorType merge_pts;

    const auto& map_pts_vec = this->map_octree.points().points;

    // 1. If the update region is large enough to have space that doesn't
    // intersect the merge window, then remove it first.
    if (((bound_max.array() - bound_min.array()) >
         Arr3f::Constant(this->obstacle_merge_window * 2.f))
            .all())
    {
        Vec3f inner_min =
            bound_min + Vec3f::Constant(this->obstacle_merge_window);
        Vec3f inner_max =
            bound_max - Vec3f::Constant(this->obstacle_merge_window);

        this->map_octree.boxSearch(inner_min, inner_max, tmp_indices);
        this->map_octree.deletePoints(tmp_indices, false);
        tmp_indices.clear();
    }

    // 2. Copy out merge-window points, then remove from map
    this->map_octree.boxSearch(
        bound_min + Vec3f::Constant(res),
        bound_max - Vec3f::Constant(res),
        tmp_indices);
    merge_pts.reserve(tmp_indices.size());
    util::copySelection(map_pts_vec, tmp_indices, merge_pts);
    this->map_octree.deletePoints(tmp_indices, false);
    tmp_indices.clear();

    // 3. Insert new points
    this->map_octree.addPoints(pts);

    // TODO: fix weird artifacts when doing this:
    // 4. Update trav scores for old-new intersecting voxels (in window)
    // for (const auto& pt : merge_pts)
    // {
    //     if ((isWeighted(pt) || isObstacle(pt)) &&
    //         this->map_octree.radiusSearch(pt, res * 0.5f, tmp_indices, tmp_dists))
    //     {
    //         for(const pcl::index_t i : tmp_indices)
    //         {
    //             PointT& new_pt = this->map_octree.pointAt(i);
    //             if (weight(pt) > weight(new_pt))
    //             {
    //                 weight(new_pt) = weight(pt);
    //             }
    //         }
    //     }
    //     tmp_indices.clear();
    // }

    // 5. Update u-e octree, insert new frontier points
    this->ue_octree.addExploredSpace(bound_min, bound_max);

    // 6. Optimize point layout
    this->map_octree.optimizeStorage();
}

template<typename P>
void PathPlanMap<P>::crop(const Vec3f& min, const Vec3f& max)
{
    this->ue_octree.crop(min, max);
    this->map_octree.crop(min, max);
}

template<typename P>
const typename PathPlanMap<P>::PointCloudT& PathPlanMap<P>::getPoints() const
{
    return this->map_octree.points();
}

template<typename P>
const typename PathPlanMap<P>::UEOctreeT& PathPlanMap<P>::getUESpace() const
{
    return this->ue_octree;
}

template<typename P>
const typename PathPlanMap<P>::MapOctreeT& PathPlanMap<P>::getMap() const
{
    return this->map_octree;
}

};  // namespace perception
};  // namespace csm
