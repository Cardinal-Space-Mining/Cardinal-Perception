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

#include <cmath>

#include "../kfc_map.hpp"


namespace csm
{
namespace perception
{

template<typename PointT, typename MapT>
void KFCMap<PointT, MapT>::applyParams(
    double frustum_search_radius,
    double radial_dist_thresh,
    double surface_width,
    double delete_max_range,
    double add_max_range,
    double voxel_res)
{
    std::unique_lock<std::mutex> lock{this->mtx};

    this->frustum_search_radius = frustum_search_radius;
    this->radial_dist_thresh = radial_dist_thresh;
    this->half_surface_width = surface_width / 2.;
    this->delete_max_range = delete_max_range;
    this->add_max_range = add_max_range;

    if (voxel_res > 0.)
    {
        this->map_octree.setResolution(voxel_res);
    }
}

template<typename PointT, typename MapT>
void KFCMap<PointT, MapT>::setBounds(
    const Vec3f& min,
    const Vec3f& max)
{
    std::unique_lock<std::mutex> lock{this->mtx};

    this->bounds_min = min.array();
    this->bounds_max = max.array();

    this->map_octree.crop(min, max, true);
}

template<typename PointT, typename MapT>
template<int CollisionV, typename RayDirT>
typename KFCMap<PointT, MapT>::UpdateResult
    KFCMap<PointT, MapT>::updateMap(
        const Vec3f& origin,
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
        // start with raw scan if map is empty
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

    // reset search buffers
    buff.search_indices.clear();
    buff.dists.clear();

    // collect indices for points within range from scanner origin
    PointT lp;
    lp.getVector3fMap() = origin;
    results.points_searched = this->map_octree.radiusSearch(
        lp,
        this->delete_max_range,
        buff.search_indices,
        buff.dists);

    // prepare submap range buffer
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
        // store distance and original index for each submap point
        auto& v = this->submap_ranges->points.emplace_back();
        v.curvature = std::sqrt(buff.dists[i]);
        v.label = buff.search_indices[i];

        // store unit direction to each submap point from scanner
        const auto& p = map_cloud_ptr->points[v.label];
        v.getNormalVector3fMap() = (p.getVector3fMap() - origin);
        v.getVector3fMap() = v.getNormalVector3fMap().normalized();
    }
    this->submap_ranges->width = this->submap_ranges->points.size();
    this->submap_ranges->height = 1;

    // build kdtree using unit directions of submap points
    this->collision_kdtree.setInputCloud(
        this->submap_ranges);  // TODO: handle no map points

    auto& scan_vec = pts.points;
    buff.submap_remove_indices.clear();
    buff.points_to_add.clear();
    buff.points_to_add.reserve(
        pt_indices ? pt_indices->size() : scan_vec.size());

    // loop all input points
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

        // collision direction buffer
        const auto& src_p = scan_vec[i];
        CollisionPointT p;
        p.getNormalVector3fMap() = (src_p.getVector3fMap() - origin);
        p.curvature = p.getNormalVector3fMap().norm();
        p.getVector3fMap() = p.getNormalVector3fMap().normalized();

        // search for points with directions in angular range
        this->collision_kdtree.radiusSearch(
            p,
            this->frustum_search_radius,
            buff.search_indices,
            buff.dists);

        // loop points in angular range
        for (size_t j = 0; j < buff.search_indices.size(); j++)
        {
            pcl::index_t k = buff.search_indices[j];

            // only consider keeping the point if it has a valid normal
            if (!(CollisionV & KF_COLLISION_MODEL_REMOVE_INVALID_NORMALS) ||
                !std::isnan(this->map_octree.pointNormals()[k][4]))
            {
                if constexpr (CollisionV & KF_COLLISION_MODEL_USE_RADIAL)
                {
                    // keep the point if it's outside the radial zone
                    if ((this->submap_ranges->points[k].curvature *
                         std::sqrt(buff.dists[j])) > this->radial_dist_thresh)
                    {
                        continue;
                    }
                }

                // find local plane intersection, and keep the point if the new point is "in range" of this sample
                auto n = this->map_octree.pointNormals()[k].template head<3>();
                const float inv_a = 1.f / p.getVector3fMap().dot(n);
                const float b =
                    this->submap_ranges->points[k].getNormalVector3fMap().dot(
                        n);

                if (std::isinf(inv_a) ||
                    (std::abs(p.curvature - b * inv_a) <=
                     static_cast<float>(this->half_surface_width) /* * inv_a*/))
                {
                    continue;
                }
            }

            buff.submap_remove_indices.insert(
                this->submap_ranges->points[k].label);
        }

        // add point if it's in the currently configured bounds
        if ((src_p.getArray3fMap() > this->bounds_min).all() &&
            (src_p.getArray3fMap() < this->bounds_max).all() &&
            (p.curvature <= this->add_max_range))
        {
            buff.points_to_add.push_back(i);
        }
    }

    // if infinite range dirs provided, remove all points in respective collision zones
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

                if constexpr (CollisionV & KF_COLLISION_MODEL_USE_RADIAL)
                {
                    if ((this->submap_ranges->points[k].curvature *
                         std::sqrt(buff.dists[i])) > this->radial_dist_thresh)
                    {
                        continue;
                    }
                }

                buff.submap_remove_indices.insert(
                    this->submap_ranges->points[k].label);
            }
        }
    }

    // delete all points which were "collided with"
    for (auto itr = buff.submap_remove_indices.begin();
         itr != buff.submap_remove_indices.end();
         itr++)
    {
        this->map_octree.deletePoint(*itr);
    }
    // add source points
    this->map_octree.addPoints(pts, &buff.points_to_add);
    // ensure point buffers are dense
    this->map_octree.optimizeStorage();

    results.points_deleted =
        static_cast<uint32_t>(buff.submap_remove_indices.size());
    return results;
}

};  // namespace perception
};  // namespace csm
