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
        this->map_octree.reconfigure(voxel_res);
    }
}

template<typename PointT, typename MapT>
void KFCMap<PointT, MapT>::setBounds(const Vec3f& min, const Vec3f& max)
{
    std::unique_lock<std::mutex> lock{this->mtx};

    this->bounds_min = min.array();
    this->bounds_max = max.array();

    this->map_octree.crop(min, max, true);
}

template<typename PointT, typename MapT>
template<int CollisionV, typename RayDirT>
typename KFCMap<PointT, MapT>::UpdateResult KFCMap<PointT, MapT>::updateMap(
    const Vec3f& origin,
    const PointCloudT& pts,
    const std::vector<RayDirT>* inf_rays,
    const pcl::Indices* pt_indices)
{
    // 0. Init buffers ---------------------------------------------------------
    UpdateResult results{};
    if (pt_indices && pt_indices->size() <= 0)
    {
        return results;
    }

    pcl::Indices tmp_indices, points_to_add;
    std::vector<float> tmp_dists;
    std::set<pcl::index_t> submap_remove_indices;

    std::unique_lock<std::mutex> lock{this->mtx};

    // 1. Use raw if empty -----------------------------------------------------
    auto map_cloud_ptr = this->map_octree.getInputCloud();
    if (map_cloud_ptr->empty())
    {
        // start with raw scan if map is empty
        this->map_octree.addPoints(pts, pt_indices);
        return results;
    }

    // 2. Collect indices for points within range from scanner origin ----------
    PointT lp;
    lp.getVector3fMap() = origin;
    results.points_searched = this->map_octree.radiusSearch(
        lp,
        this->delete_max_range,
        tmp_indices,
        tmp_dists);

    // 3. Compute range, direction for each point in submap --------------------
    if (!this->submap_ranges)
    {
        this->submap_ranges = std::make_shared<CollisionPointCloudT>();
    }
    this->submap_ranges->clear();
    this->submap_ranges->reserve(tmp_indices.size());

    // IMPROVE: OMP parallelize
    for (size_t i = 0; i < tmp_indices.size(); i++)
    {
        // store distance and original index for each submap point
        auto& v = this->submap_ranges->points.emplace_back();
        v.curvature = std::sqrt(tmp_dists[i]);
        v.label = tmp_indices[i];

        // store unit direction to each submap point from scanner
        const auto& p = map_cloud_ptr->points[v.label];
        v.getNormalVector3fMap() = (p.getVector3fMap() - origin);
        v.getVector3fMap() = v.getNormalVector3fMap().normalized();
    }
    this->submap_ranges->width = this->submap_ranges->points.size();
    this->submap_ranges->height = 1;

    // 4. Build kdtree using unit directions of submap points ------------------
    // TODO: handle no map points
    this->collision_kdtree.setInputCloud(this->submap_ranges);

    auto& scan_vec = pts.points;
    points_to_add.reserve(pt_indices ? pt_indices->size() : scan_vec.size());

    // 5. Analyze collisions for all input points ------------------------------
    // IMPROVE: could possibly parallelize if the outputs are synchronized
    for (size_t idx = 0;; idx++)
    {
        // 5-A. Handle selection
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

        // 5-B. Compute direction and range for input point
        const auto& src_p = scan_vec[i];
        CollisionPointT p;
        p.getNormalVector3fMap() = (src_p.getVector3fMap() - origin);
        p.curvature = p.getNormalVector3fMap().norm();
        p.getVector3fMap() = p.getNormalVector3fMap().normalized();

        // 5-C. Search for points with directions in angular range using kdtree
        tmp_indices.clear();
        tmp_dists.clear();
        this->collision_kdtree.radiusSearch(
            p,
            this->frustum_search_radius,
            tmp_indices,
            tmp_dists);

        // 5-D. Compare input point to all points in search set to analyze collisions
        for (size_t j = 0; j < tmp_indices.size(); j++)
        {
            pcl::index_t k = tmp_indices[j];

            // only consider keeping the point if it has a valid normal
            if (!(CollisionV & KF_COLLISION_MODEL_REMOVE_INVALID_NORMALS) ||
                !std::isnan(this->map_octree.pointNormals()[k][4]))
            {
                if constexpr (CollisionV & KF_COLLISION_MODEL_USE_RADIAL)
                {
                    // keep the point if it's outside the radial zone
                    if ((this->submap_ranges->points[k].curvature *
                         std::sqrt(tmp_dists[j])) > this->radial_dist_thresh)
                    {
                        continue;
                    }
                }

                // find local plane intersection, and keep the point if the new
                // point is "in range" of this sample
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

            submap_remove_indices.insert(this->submap_ranges->points[k].label);
        }

        // 5-E. Add point if it's in the currently configured bounds
        if ((src_p.getArray3fMap() > this->bounds_min).all() &&
            (src_p.getArray3fMap() < this->bounds_max).all() &&
            (p.curvature <= this->add_max_range))
        {
            points_to_add.push_back(i);
        }
    }

    // 6. Repeat previous process with infinite ray directions, if provided ----
    if (inf_rays)
    {
        for (const RayDirT& r : *inf_rays)
        {
            CollisionPointT p;
            p.getVector3fMap() = r.getNormalVector3fMap();

            tmp_indices.clear();
            tmp_dists.clear();
            this->collision_kdtree.radiusSearch(
                p,
                this->frustum_search_radius,
                tmp_indices,
                tmp_dists);

            for (size_t i = 0; i < tmp_indices.size(); i++)
            {
                pcl::index_t k = tmp_indices[i];

                if constexpr (CollisionV & KF_COLLISION_MODEL_USE_RADIAL)
                {
                    if ((this->submap_ranges->points[k].curvature *
                         std::sqrt(tmp_dists[i])) > this->radial_dist_thresh)
                    {
                        continue;
                    }
                }

                submap_remove_indices.insert(
                    this->submap_ranges->points[k].label);
            }
        }
    }

    // 7. Delete all points which were "collided with" from the map ------------
    for (auto itr = submap_remove_indices.begin();
         itr != submap_remove_indices.end();
         itr++)
    {
        this->map_octree.deletePoint(*itr, true);
    }

    // 8. Add inputs points to map ---------------------------------------------
    this->map_octree.addPoints(pts, &points_to_add);

    // 9. Ensure map point buffer is dense -------------------------------------
    this->map_octree.optimizeStorage();

    results.points_deleted =
        static_cast<uint32_t>(submap_remove_indices.size());
    return results;
}

};  // namespace perception
};  // namespace csm
