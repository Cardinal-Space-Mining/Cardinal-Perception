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
#include <vector>
#include <set>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "point_def.hpp"
#include "map_octree.hpp"

#ifndef KFC_MAP_STORE_INSTANCE_BUFFERS
#define KFC_MAP_STORE_INSTANCE_BUFFERS 0
#endif


namespace csm
{
namespace perception
{

#define KF_COLLISION_MODEL_CONE     0b001   // All points in a spherical coord range are used in computing collision.
#define KF_COLLISION_MODEL_RADIAL   0b010   // Only points a certain delta away from the lidar rays are used in computing collision. More physically accurate but more costly computation.
#define KF_COLLISION_MODEL_USE_DIFF 0b100   // Utilize a threshold distance for collision detection.


/** KDTree Frustum Collision (KFC) mapping implementation */
template<
    typename PointT,
    typename MapT = csm::perception::MapOctree<PointT>,
    typename CollisionPointT = pcl::PointXYZLNormal >
class KFCMap
{
    static_assert(
        std::is_base_of<csm::perception::MapOctree<PointT, void>, MapT>::value ||
        std::is_base_of<csm::perception::MapOctree<PointT, MapT>, MapT>::value );
    static_assert(
        pcl::traits::has_normal<CollisionPointT>::value &&
        pcl::traits::has_curvature<CollisionPointT>::value &&
        pcl::traits::has_label<CollisionPointT>::value );

public:
    KFCMap(double voxel_size = 0.1) : map_octree{ voxel_size } {}
    ~KFCMap() = default;

public:
    struct UpdateResult
    {
        union
        {
            uint64_t data;
            struct
            {
                uint32_t points_searched;
                uint32_t points_deleted;
            };
        };

        UpdateResult(uint64_t v = 0) : data{ v } {}
        uint64_t operator uint64_t() const { return this->data; }
    };

public:
    /* @param frustum_search_radius The radius which is searched in the KDTree for each lidar point when
     * computing collisions. This value should optimally be set to half of the angular distance (radians)
     * between consecutive points in the scan, and can be thought of as the radius of the "frustum cone"
     * computed for each ray.
     * @param radial_dist_thresh The radial distance from the lidar ray which points must be within to be
     * considered when computing collisions (for KF_COLLISION_MODEL_RADIAL). Note that the internal computation
     * is approximated to save on resources, meaning a larger parameter value will be needed to acheive a
     * given threshold. With small angluar deviations (as typical with most LiDARs), this affect is negligible.
     * @param delete_delta_coeff Scaled by the distance to each map point in question, for which the lidar
     * point range must be at least this value larger before deletion of the map point occurs. Useful in
     * negating accidental deletions when rays are nearly parallel with a surface (especially when using
     * KF_COLLISION_MODEL_CONE).
     * @param delete_max_range The maximum range for points that can be deleted from the map.
     * @param add_max_range The maximum range for points that can be added to the map.
     * @param voxel_res The octree resolution. Values <= 0 result in no change. */
    void applyParams(
        double frustum_search_radius,
        double radial_dist_thresh,
        double delete_delta_coeff,
        double delete_max_range,
        double add_max_range,
        double voxel_res = -1.);

    template<uint32_t CollisionModel = (KF_COLLISION_MODEL_RADIAL | KF_COLLISION_MODEL_USE_DIFF)>
    UpdateResult updateMap(
        Eigen::Vector3f origin,
        const pcl::PointCloud<PointT>& pts,
        const pcl::Indices* indices = nullptr);

    inline pcl::PointCloud<PointT>::ConstPtr getPoints() const
        { return this->map_octree.getInputCloud(); }

    inline const MapT& getMap() const
        { return this->map_octree; }

    inline size_t numPoints() const
        { return this->map_octree.getInputCloud().size(); }

protected:
    pcl::KdTreeFLANN<CollisionPointT> collision_kdtree;
    pcl::PointCloud<CollisionPointT>::Ptr submap_ranges{ nullptr };
    MapT map_octree;

    std::mutex mtx;

#if KFC_MAP_STORE_INSTANCE_BUFFERS
    struct
    {
        pcl::Indices search_indices, points_to_add;
        std::vector<float> dists;
        std::set<pcl::index_t> submap_remove_indices;
    }
    buff;
#endif

    double
        frustum_search_radius{ 0.01 },
        radial_dist_sqrd_thresh{ 0.01 * 0.01 },
        delete_delta_coeff{ 0.1 },
        delete_max_range{ 3. },
        add_max_range{ 5. };

};



template<typename PointT, typename MapT, typename CollisionPointT>
void KFCMap<PointT, MapT, CollisionPointT>::applyParams(
    double frustum_search_radius,
    double radial_dist_thresh,
    double delete_delta_coeff,
    double delete_max_range,
    double add_max_range,
    double voxel_res )
{
    this->frustum_search_radius = frustum_search_radius;
    this->radial_dist_sqrd_thresh = radial_dist_thresh * radial_dist_thresh;
    this->delete_delta_coeff = delete_delta_coeff;
    this->delete_max_range = delete_max_range;
    this->add_max_range = add_max_range;

    if(voxel_res > 0.)
    {
        this->map_octree.setResolution(voxel_res);
    }
}

template<typename PointT, typename MapT, typename CollisionPointT>
template<uint32_t CollisionModel>
KFCMap<PointT, MapT, CollisionPointT>::UpdateResult
KFCMap<PointT, MapT, CollisionPointT>::updateMap<CollisionModel>(
    Eigen::Vector3f origin,
    const pcl::PointCloud<PointT>& pts,
    const pcl::Indices* indices = nullptr )
{
    UpdateResult results{};
    if(indices && indices->size() <= 0) return results;

    std::unique_lock<std::mutex> lock{ this->mtx };

    auto map_cloud_ptr = this->map_octree.getInputCloud();
    if(map_cloud_ptr->empty())
    {
        this->map_octree.addPoints(pts, indices);
        return results;
    }

#if KFC_MAP_STORE_INSTANCE_BUFFERS <= 0
    thread_local struct
    {
        pcl::Indices search_indices, points_to_add;
        std::vector<float> dists;
        std::set<pcl::index_t> submap_remove_indices;
    }
    buff;
#endif

    buff.search_indices.clear();
    buff.dists.clear();

    PointT lp;
    lp.getVector3fMap() = origin;
    results.points_searched = this->map_octree.radiusSearch(
        lp,
        this->delete_max_range,
        buff.search_indices,
        buff.dists );

    if(!this->submap_ranges)
        this->submap_ranges = std::make_shared<pcl::PointCloud<CollisionPointT>>();
    this->submap_ranges.clear();
    this->submap_ranges.reserve(buff.search_indices.size());

    // IMPROVE: OMP parallelize
    for(size_t i = 0; i < buff.search_indices.size(); i++)
    {
        auto& v = this->submap_ranges->points.emplace_back();
        v.curvature = std::sqrt(buff.dists[i]);
        v.label = buff.search_indices[i];

        const auto& p = map_cloud_ptr->points[v.label];
        v.getNormalVector3fMap() = (p.getVector3fMap() - origin);
        v.getVector3fMap() = v.getNormalVector3fMap().normalized();
    }
    this->submap_ranges->width = this->submap_ranges->points.size();

    this->collision_kdtree.setInputCloud(this->submap_ranges);

    auto& scan_vec = pts.points;
    buff.submap_remove_indices.clear();
    buff.points_to_add.clear();
    buff.points_to_add.reserve(indices ? indices->size() : scan_vec.size());

    // IMPROVE: could possibly parallelize if the outputs are synchronized
    for(size_t idx = 0;; idx++)
    {
        size_t i = idx;
        if(indices)
        {
            if(idx >= indices->size()) break;
            i = static_cast<size_t>((*indices)[idx]);
        }
        else if(idx >= scan_vec.size()) break;

        CollisionPointT p;
        p.getNormalVector3fMap() = (scan_vec[i].getVector3fMap() - origin);
        p.curvature = p.getNormalVector3fMap().norm();
        p.getVector3fMap() = p.getNormalVector3fMap().normalized();

        this->collision_kdtree.radiusSearch(
            p,
            this->mapping_frustum_search_radius,
            buff.search_indices );

        for(size_t j = 0; j < buff.search_indices.size(); j++)
        {
            pcl::index_t k = buff.search_indices[j];
            const float v = this->submap_ranges->points[k].curvature;

            if constexpr(CollisionModel & KF_COLLISION_MODEL_RADIAL)
            {
                if(v * v * buff.dists[j] > this->radial_dist_sqrd_thresh) continue;
            }

            float comp = p.curvature - v;
            if constexpr(CollisionModel & KF_COLLISION_MODEL_USE_DIFF)
            {
                comp -= this->delete_delta_coeff * v;
            }
            if(comp > 0.f)
            {
                buff.submap_remove_indices.insert(this->submap_ranges->points[k].label);
                // IMPROVE: find a way to remove from the kdtree so subsequent searches are faster and we don't need to use a set
            }
        }

        if(p.curvature <= this->add_max_range)
        {
            buff.points_to_add.push_back(i);
        }
    }

    for(auto itr = buff.submap_remove_indices.begin(); itr != buff.submap_remove_indices.end(); itr++)
    {
        this->map_octree.deletePoint(*itr);
    }
    this->map_octree.addPoints(pts, buff.points_to_add);
    this->map_octree.normalizeCloud();

    results.points_deleted = static_cast<uint32_t>(buff.submap_remove_indices.size());
    return results;
}

};
};
