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

/** KFC Collision Modifiers
 1. KF_COLLISION_MODEL_USE_RADIAL - Instead of using the default conical 
        projection for computing collisions, only considers points a certain
        radial distance away from each lidar ray (producing a cylindrical 
        collision zone). This is more physically accurate and provides better
        mapping but comes with a slight performance hit.
 2. KF_COLLISION_MODEL_USE_DIFF - Enables the requirement for lidar points to be
        a certain delta farther than any given map point in order to trigger 
        deletion. See the delete_delta_coeff parameter for more details.
 3. KF_COLLISION_MODEL_USE_EXTENDED_PROJECTION - Enables the deletion of map
        points a dertain distance farther than any given lidar point. Used 
        primarily in specialty mapping applications where ghost points need to 
        be mitigated. Mutually exclusive with KF_COLLISION_MODEL_USE_DIFF, which
        takes precedence. */
#define KF_COLLISION_MODEL_USE_RADIAL            0b001
#define KF_COLLISION_MODEL_USE_DIFF              0b010
#define KF_COLLISION_MODEL_USE_EXTEND_PROJECTION 0b100

#define KF_COLLISION_DEFAULT_PARAMS                               \
    (KF_COLLISION_MODEL_USE_RADIAL | KF_COLLISION_MODEL_USE_DIFF)


/** KDTree Frustum Collision (KFC) mapping implementation */
template<
    typename PointT,
    typename MapT = csm::perception::MapOctree<PointT>,
    typename CollisionPointT = pcl::PointXYZLNormal>
class KFCMap
{
    // static_assert(
    //     std::is_base_of<csm::perception::MapOctree<PointT, void>, MapT>::
    //         value ||
    //     std::is_base_of<csm::perception::MapOctree<PointT, MapT>, MapT>::value);
    static_assert(
        pcl::traits::has_normal<CollisionPointT>::value &&
        pcl::traits::has_curvature<CollisionPointT>::value &&
        pcl::traits::has_label<CollisionPointT>::value);

public:
    KFCMap(double voxel_size = 0.1) : map_octree{voxel_size} {}
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

        inline UpdateResult(uint64_t v = 0) : data{v} {}
        inline operator uint64_t() const { return this->data; }
    };

public:
    /* @param frustum_search_radius The radius which is searched in the KDTree
     * for each lidar point when computing collisions. This value should 
     * optimally be set to half of the angular distance (radians) between 
     * consecutive points in the scan, and can be thought of as the radius of
     * the "frustum cone" computed for each ray.
     * @param radial_dist_thresh The radial distance from the lidar ray which
     * points must be within to be considered when computing collisions (for
     * KF_COLLISION_MODEL_RADIAL). Note that the internal computation is
     * approximated to save on resources, meaning a larger parameter value will
     * be needed to acheive a given threshold. With small angluar deviations (as
     * typical with most LiDARs), this affect is negligible.
     * @param delete_delta_coeff Scaled by the distance to each map point in
     * question, for which the lidar point range must be at least this value
     * larger before deletion of the map point occurs. Useful in negating
     * accidental deletions when rays are nearly parallel with a surface
     * (especially when not using KF_COLLISION_MODEL_RADIAL).
     * @param delete_max_range The maximum range for points that can be deleted
     * from the map.
     * @param add_max_range The maximum range for points that can be added to
     * the map.
     * @param voxel_res The octree resolution. Values <= 0 result in no change. 
     */
    void applyParams(
        double frustum_search_radius,
        double radial_dist_thresh,
        double delete_delta_coeff,
        double extended_delete_range,
        double delete_max_range,
        double add_max_range,
        double voxel_res = -1.);

    /* Updates the map */
    template<uint32_t CollisionModel = KF_COLLISION_DEFAULT_PARAMS>
    inline UpdateResult updateMap(
        Eigen::Vector3f origin,
        const pcl::PointCloud<PointT>& pts,
        const pcl::Indices* indices = nullptr)
    {
        return this->updateMap<CollisionModel, pcl::Axis>(
            origin,
            pts,
            nullptr,
            indices);
    }
    /* Updates the result */
    template<
        uint32_t CollisionModel = KF_COLLISION_DEFAULT_PARAMS,
        typename RayDirT = pcl::Axis>
    inline UpdateResult updateMap(
        Eigen::Vector3f origin,
        const pcl::PointCloud<PointT>& pts,
        const std::vector<RayDirT>& inf_rays,
        const pcl::Indices* pt_indices = nullptr)
    {
        static_assert(pcl::traits::has_normal<RayDirT>::value);

        return this->updateMap<CollisionModel, RayDirT>(
            origin,
            pts,
            &inf_rays,
            pt_indices);
    }

    inline typename pcl::PointCloud<PointT>::ConstPtr getPoints() const
    {
        return this->map_octree.getInputCloud();
    }

    inline const MapT& getMap() const { return this->map_octree; }

    inline size_t numPoints() const
    {
        return this->map_octree.getInputCloud()->size();
    }

protected:
    template<uint32_t CollisionModel, typename RayDirT>
    UpdateResult updateMap(
        Eigen::Vector3f origin,
        const pcl::PointCloud<PointT>& pts,
        const std::vector<RayDirT>* inf_rays,
        const pcl::Indices* pt_indices);

protected:
    pcl::KdTreeFLANN<CollisionPointT> collision_kdtree;
    typename pcl::PointCloud<CollisionPointT>::Ptr submap_ranges{nullptr};
    MapT map_octree;

    std::mutex mtx;

#if KFC_MAP_STORE_INSTANCE_BUFFERS
    struct
    {
        pcl::Indices search_indices, points_to_add;
        std::vector<float> dists;
        std::set<pcl::index_t> submap_remove_indices;
    } buff;
#endif

    double frustum_search_radius{0.01}, radial_dist_sqrd_thresh{0.01 * 0.01},
        delete_delta_coeff{0.1}, extended_delete_range{0.01},
        delete_max_range{3.}, add_max_range{5.};
};



template<typename PointT, typename MapT, typename CollisionPointT>
void KFCMap<PointT, MapT, CollisionPointT>::applyParams(
    double frustum_search_radius,
    double radial_dist_thresh,
    double delete_delta_coeff,
    double extended_delete_range,
    double delete_max_range,
    double add_max_range,
    double voxel_res)
{
    this->frustum_search_radius = frustum_search_radius;
    this->radial_dist_sqrd_thresh = radial_dist_thresh * radial_dist_thresh;
    this->delete_delta_coeff = delete_delta_coeff;
    this->extended_delete_range = extended_delete_range;
    this->delete_max_range = delete_max_range;
    this->add_max_range = add_max_range;

    if (voxel_res > 0.)
    {
        this->map_octree.setResolution(voxel_res);
    }
}

template<typename PointT, typename MapT, typename CollisionPointT>
template<uint32_t CollisionModel, typename RayDirT>
typename KFCMap<PointT, MapT, CollisionPointT>::UpdateResult
    KFCMap<PointT, MapT, CollisionPointT>::updateMap(
        Eigen::Vector3f origin,
        const pcl::PointCloud<PointT>& pts,
        const std::vector<RayDirT>* inf_rays,
        const pcl::Indices* pt_indices)
{
    UpdateResult results{};
    if (pt_indices && pt_indices->size() <= 0)
    {
        return results;
    }

    std::unique_lock<std::mutex> lock{this->mtx};

    auto map_cloud_ptr = this->map_octree.getInputCloud();
    if (map_cloud_ptr->empty())
    {
        this->map_octree.addPoints(pts, pt_indices);
        return results;
    }

#if KFC_MAP_STORE_INSTANCE_BUFFERS <= 0
    thread_local struct
    {
        pcl::Indices search_indices, points_to_add;
        std::vector<float> dists;
        std::set<pcl::index_t> submap_remove_indices;
    } buff;
#endif

    buff.search_indices.clear();
    buff.dists.clear();

    PointT lp;
    lp.getVector3fMap() = origin;
    results.points_searched = this->map_octree.radiusSearch(
        lp,
        this->delete_max_range,
        buff.search_indices,
        buff.dists);

    if (!this->submap_ranges)
    {
        this->submap_ranges =
            std::make_shared<pcl::PointCloud<CollisionPointT>>();
    }
    this->submap_ranges->clear();
    this->submap_ranges->reserve(buff.search_indices.size());

    // IMPROVE: OMP parallelize
    for (size_t i = 0; i < buff.search_indices.size(); i++)
    {
        auto& v = this->submap_ranges->points.emplace_back();
        v.curvature = std::sqrt(buff.dists[i]);
        v.label = buff.search_indices[i];

        const auto& p = map_cloud_ptr->points[v.label];
        v.getNormalVector3fMap() = (p.getVector3fMap() - origin);
        v.getVector3fMap() = v.getNormalVector3fMap().normalized();
    }
    this->submap_ranges->width = this->submap_ranges->points.size();
    this->submap_ranges->height = 1;

    this->collision_kdtree.setInputCloud(
        this->submap_ranges);  // TODO: handle no map points

    auto& scan_vec = pts.points;
    buff.submap_remove_indices.clear();
    buff.points_to_add.clear();
    buff.points_to_add.reserve(
        pt_indices ? pt_indices->size() : scan_vec.size());

    // IMPROVE: could possibly parallelize if the outputs are synchronized
    for (size_t idx = 0;; idx++)
    {
        size_t i = idx;
        if (pt_indices)
        {
            if (idx >= pt_indices->size())
            {
                break;
            }
            i = static_cast<size_t>((*pt_indices)[idx]);
        }
        else if (idx >= scan_vec.size())
        {
            break;
        }

        CollisionPointT p;
        p.getNormalVector3fMap() = (scan_vec[i].getVector3fMap() - origin);
        p.curvature = p.getNormalVector3fMap().norm();
        p.getVector3fMap() = p.getNormalVector3fMap().normalized();

        this->collision_kdtree.radiusSearch(
            p,
            this->frustum_search_radius,
            buff.search_indices,
            buff.dists);

        for (size_t j = 0; j < buff.search_indices.size(); j++)
        {
            pcl::index_t k = buff.search_indices[j];
            const float v = this->submap_ranges->points[k].curvature;

            if constexpr (CollisionModel & KF_COLLISION_MODEL_USE_RADIAL)
            {
                if (v * v * buff.dists[j] > this->radial_dist_sqrd_thresh)
                {
                    continue;
                }
            }

            float comp = p.curvature - v;
            if constexpr (CollisionModel & KF_COLLISION_MODEL_USE_DIFF)
            {
                comp -= this->delete_delta_coeff * v;
            }
            else if constexpr (
                CollisionModel & KF_COLLISION_MODEL_USE_EXTEND_PROJECTION)
            {
                comp += this->extended_delete_range;
            }
            if (comp > 0.f)
            {
                buff.submap_remove_indices.insert(
                    this->submap_ranges->points[k].label);
                // IMPROVE: find a way to remove from the kdtree so subsequent
                // searches are faster and we don't need to use a set
            }
        }

        if (p.curvature <= this->add_max_range)
        {
            buff.points_to_add.push_back(i);
        }
    }

    if (inf_rays)
    {
        for (const RayDirT& r : *inf_rays)
        {
            CollisionPointT p;
            p.getVector3fMap() = r.getNormalVector3fMap();

            this->collision_kdtree.radiusSearch(
                p,
                this->frustum_search_radius,
                buff.search_indices,
                buff.dists);

            for (size_t i = 0; i < buff.search_indices.size(); i++)
            {
                pcl::index_t k = buff.search_indices[i];
                const float v = this->submap_ranges->points[k].curvature;

                if constexpr (CollisionModel & KF_COLLISION_MODEL_USE_RADIAL)
                {
                    if (v * v * buff.dists[i] > this->radial_dist_sqrd_thresh)
                    {
                        continue;
                    }
                }

                buff.submap_remove_indices.insert(
                    this->submap_ranges->points[k].label);
            }
        }
    }

    for (auto itr = buff.submap_remove_indices.begin();
         itr != buff.submap_remove_indices.end();
         itr++)
    {
        this->map_octree.deletePoint(*itr);
    }
    this->map_octree.addPoints(pts, &buff.points_to_add);
    this->map_octree.normalizeCloud();

    results.points_deleted =
        static_cast<uint32_t>(buff.submap_remove_indices.size());
    return results;
}

};  // namespace perception
};  // namespace csm
