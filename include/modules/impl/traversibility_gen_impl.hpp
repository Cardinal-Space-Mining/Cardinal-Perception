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

#include "../traversibility_gen.hpp"

#include <cmath>
#include <numbers>

#include <pcl/features/normal_3d.h>

#include <util/cloud_ops.hpp>
#include <util/std_utils.hpp>


namespace csm
{
namespace perception
{


template<typename M, typename T>
void TraversibilityGenerator<M, T>::configure(
    float normal_estimation_radius,
    float output_res,
    float grad_search_radius,
    float min_grad_diff,
    float non_traversible_grad_angle,
    float avoidance_radius,
    float trav_score_curvature_weight,
    float trav_score_grad_weight,
    int min_vox_cell_points,
    int interp_sample_count)
{
    this->normal_estimation_radius = normal_estimation_radius;
    this->output_res = output_res;
    this->grad_search_radius = grad_search_radius;
    this->min_grad_diff = min_grad_diff;
    this->non_trav_grad_thresh =
        1.f - std::cos(non_traversible_grad_angle * (std::numbers::pi / 180.));
    this->avoidance_radius = avoidance_radius;
    this->avoid_radius_sqrd = avoidance_radius * avoidance_radius;
    this->trav_score_curvature_weight = trav_score_curvature_weight;
    this->trav_score_grad_weight = trav_score_grad_weight;
    this->min_vox_cell_points = min_vox_cell_points;
    this->interp_sample_count = interp_sample_count;
}

template<typename M, typename T>
void TraversibilityGenerator<M, T>::processMapPoints(
    const MapPointCloud& map_points,
    const Vec3f& map_min_bound,
    const Vec3f& map_max_bound,
    const Vec3f& map_grav_vec,
    const Vec3f& source_pos)
{
    std::lock_guard lock_{this->mtx};
    this->process(
        map_points,
        map_min_bound,
        map_max_bound,
        map_grav_vec,
        source_pos);
}


template<typename M, typename T>
typename TraversibilityGenerator<M, T>::TravPointCloud&
    TraversibilityGenerator<M, T>::copyPoints(TravPointCloud& copy_cloud) const
{
    this->mtx.lock();
    copy_cloud = this->points;
    this->mtx.unlock();

    return copy_cloud;
}

template<typename M, typename T>
typename TraversibilityGenerator<M, T>::TravPointCloud&
    TraversibilityGenerator<M, T>::swapPoints(TravPointCloud& swap_cloud)
{
    this->mtx.lock();
    std::swap(this->points.points, swap_cloud.points);
    std::swap(this->points.height, swap_cloud.height);
    std::swap(this->points.width, swap_cloud.width);
    std::swap(this->points.is_dense, swap_cloud.is_dense);
    this->mtx.unlock();

    return swap_cloud;
}

template<typename M, typename T>
void TraversibilityGenerator<M, T>::extractTravPoints(
    TravPointCloud& trav_points) const
{
    trav_points.clear();

    this->mtx.lock();
    util::copySelection(this->points, this->trav_selection, trav_points);
    this->mtx.unlock();
}

template<typename M, typename T>
void TraversibilityGenerator<M, T>::extractExtTravPoints(
    TravPointCloud& ext_trav_points) const
{
    ext_trav_points.clear();

    this->mtx.lock();
    util::copySelection(
        this->points,
        this->ext_trav_selection,
        ext_trav_points);
    this->mtx.unlock();
}

template<typename M, typename T>
void TraversibilityGenerator<M, T>::extractNonTravPoints(
    TravPointCloud& non_trav_points) const
{
    non_trav_points.clear();

    this->mtx.lock();
    util::copySelection(this->points, this->avoid_selection, non_trav_points);
    this->mtx.unlock();
}

template<typename M, typename T>
pcl::Indices& TraversibilityGenerator<M, T>::swapTravIndices(
    pcl::Indices& swap_indices)
{
    this->mtx.lock();
    std::swap(this->trav_selection, swap_indices);
    this->mtx.unlock();

    return swap_indices;
}
template<typename M, typename T>
pcl::Indices& TraversibilityGenerator<M, T>::swapExtTravIndices(
    pcl::Indices& swap_indices)
{
    this->mtx.lock();
    std::swap(this->ext_trav_selection, swap_indices);
    this->mtx.unlock();

    return swap_indices;
}
template<typename M, typename T>
pcl::Indices& TraversibilityGenerator<M, T>::swapNonTravIndices(
    pcl::Indices& swap_indices)
{
    this->mtx.lock();
    std::swap(this->avoid_selection, swap_indices);
    this->mtx.unlock();

    return swap_indices;
}



template<typename M, typename T>
void TraversibilityGenerator<M, T>::process(
    const MapPointCloud& map_points,
    const Vec3f& map_min_bound,
    const Vec3f& map_max_bound,
    const Vec3f& map_grav_vec,
    const Vec3f& source_pos)
{
    using namespace csm::perception::traversibility;

    pcl::Indices cell_indices_buff, nearest_indices_buff;
    std::vector<float> dists_sqrd_buff;

    this->points.clear();
    this->trav_selection.clear();
    this->ext_trav_selection.clear();
    this->avoid_selection.clear();

    // 1. REGISTER GRID INDICES FOR EACH INPUT POINT
    this->vox_grid.reconfigure(map_min_bound, map_max_bound, this->output_res);
    this->cell_pt_indices.clear();
    this->cell_pt_indices.reserve(map_points.size());
    for (size_t i = 0; i < map_points.size(); i++)
    {
        const auto& pt3 = map_points.points[i];
        const auto idx =
            this->vox_grid.getBoundingCellIdx<3>(pt3.getVector3fMap());
        if (idx >= 0)
        {
            cell_pt_indices.emplace_back(
                static_cast<uint32_t>(idx),
                static_cast<pcl::index_t>(i));
        }
    }

    // 2. SORT INDICES BY VOXEL INDEX
    std::sort(
        this->cell_pt_indices.begin(),
        this->cell_pt_indices.end(),
        [](const IndicesPair& a, const IndicesPair& b)
        { return a.first < b.first; });

    // 3. COMPUTE CENTROID FOR EACH VOXEL AND EXPORT
    for (size_t i = 0; i < this->cell_pt_indices.size(); i++)
    {
        cell_indices_buff.clear();
        uint32_t curr_cell_idx = this->cell_pt_indices[i].first;

        for (; i < this->cell_pt_indices.size(); i++)
        {
            const auto& pair = this->cell_pt_indices[i];
            if (pair.first == curr_cell_idx)
            {
                cell_indices_buff.push_back(pair.second);
            }
            else
            {
                i--;
                break;
            }
        }

        Vec4f centroid;
        if (cell_indices_buff.size() >=
                static_cast<size_t>(this->min_vox_cell_points) &&
            pcl::compute3DCentroid(map_points, cell_indices_buff, centroid))
        {
            this->points.points.emplace_back().getVector3fMap() =
                centroid.template head<3>();
            weight(this->points.points.back()) = NOMINAL_MIN_WEIGHT<TravPointT>;
        }
    }

    this->points.height = this->points.points.size();
    this->points.width = 1;

    // 4. REBUILD KDTREE
    this->points_ptr = util::wrapUnmanaged(this->points);
    this->interp_search_tree.setInputCloud(this->points_ptr);

    // 5. FILTER NON-TRAV POINTS
    for (size_t i = 0; i < this->points.size(); i++)
    {
        auto& pt = this->points.points[i];

        this->interp_search_tree.radiusSearch(
            pt,
            this->grad_search_radius,
            nearest_indices_buff,
            dists_sqrd_buff);

        Vec3f min, max;
        min = max = pt.getVector3fMap();
        for (pcl::index_t idx : nearest_indices_buff)
        {
            const auto& pt_ = this->points[idx];
            if (pt_.z < min.z())
            {
                min = pt_.getVector3fMap();
            }
            if (pt_.z > max.z())
            {
                max = pt_.getVector3fMap();
            }
        }

        if (max.z() - min.z() >= this->min_grad_diff &&
            std::abs((max - min).normalized().dot(map_grav_vec)) >
                this->non_trav_grad_thresh)
        {
            weight(pt) = OBSTACLE_MARKER_VAL<TravPointT>;
            this->avoid_selection.push_back(i);
        }
        else
        {
            this->trav_selection.push_back(i);
        }
    }

    // 6. TRAVERSE 2D GRID AND FIND EMPTY CELLS
    this->interp_index_map.clear();
    this->vox_grid.initBuffer2(this->interp_index_map, 0);

    size_t n_interp_cells = this->vox_grid.size2();
    for (const auto& pt3 : this->points)
    {
        auto& v = this->vox_grid.getCell(
            this->interp_index_map,
            Vec2f{pt3.getVector3fMap().template head<2>()});

        if (!v)
        {
            n_interp_cells--;
            v--;
        }
    }

    // 7. ITERATE EMPTY CELLS AND FILL VIA INTERPOLATION
    const Vec2f origin_cell_center =
        this->vox_grid.getCellCenter2(Vec2i::Constant(0));
    size_t cell_i = 0;
    TravPointT center_pt{0.f, 0.f, source_pos.z()};
    for (center_pt.y = origin_cell_center.y();  //
         center_pt.y < map_max_bound.y();
         center_pt.y += this->output_res)
    {
        for (center_pt.x = origin_cell_center.x();  //
             center_pt.x < map_max_bound.x();
             center_pt.x += this->output_res)
        {
            if (!this->interp_index_map[cell_i] &&
                this->interp_search_tree.nearestKSearch(
                    center_pt,
                    this->interp_sample_count,
                    nearest_indices_buff,
                    dists_sqrd_buff))
            {
                float sum_z = 0.f;
                size_t samples = 0;
                for (pcl::index_t idx : nearest_indices_buff)
                {
                    if (isObstacle(this->points.points[idx]))
                    {
                        continue;
                    }

                    sum_z += this->points.points[idx].z;
                    samples++;
                }

                if (samples)
                {
                    this->trav_selection.push_back(this->points.points.size());
                    auto& interp_pt = this->points.points.emplace_back();

                    interp_pt.x = center_pt.x;
                    interp_pt.y = center_pt.y;
                    interp_pt.z = sum_z / samples;
                    weight(interp_pt) = NOMINAL_MIN_WEIGHT<TravPointT>;
                }
            }
            cell_i++;
        }
    }

    this->points.height = this->points.points.size();
    this->points.width = 1;

    // 8. REBUILD TRAV KDTREE
    this->trav_search_tree.setInputCloud(
        this->points_ptr,
        util::wrapUnmanaged(this->trav_selection));

    // 9. LOOP AVOID POINTS, APPLY SPREAD RADIUS
    for (pcl::index_t avoid_i : this->avoid_selection)
    {
        this->trav_search_tree.radiusSearch(
            this->points.points[avoid_i],
            this->avoidance_radius,
            nearest_indices_buff,
            dists_sqrd_buff);

        for (size_t i = 0; i < nearest_indices_buff.size(); i++)
        {
            const auto idx = nearest_indices_buff[i];
            const auto w = extendedWeight<TravPointT>(
                1.f - (dists_sqrd_buff[i] / this->avoid_radius_sqrd));
            if (w > weight(this->points.points[idx]))
            {
                weight(this->points.points[idx]) = w;
            }
        }
    }

    // 10. REINDEX TRAV SELECTION AND COMPUTE NORMALS
    for (size_t i = 0; i < this->trav_selection.size(); i++)
    {
        const auto& pt_idx = this->trav_selection[i];
        auto& pt = this->points.points[pt_idx];

        if (isExtended(pt))
        {
            this->ext_trav_selection.push_back(pt_idx);
            this->trav_selection[i] = this->trav_selection.back();
            this->trav_selection.pop_back();
            i--;
        }
        else
        {
            this->trav_search_tree.radiusSearch(
                pt,
                this->normal_estimation_radius,
                nearest_indices_buff,
                dists_sqrd_buff);

            Vec4f plane;
            float curvature;
            pcl::computePointNormal(
                this->points,
                nearest_indices_buff,
                plane,
                curvature);

            const float w = (curvature * this->trav_score_curvature_weight) +
                            (1.f - map_grav_vec.dot(plane.template head<3>())) *
                                this->trav_score_grad_weight;

            weight(pt) = nominalWeight<TravPointT>(std::min(1.f, w));
        }
    }
}

};  // namespace perception
};  // namespace csm
