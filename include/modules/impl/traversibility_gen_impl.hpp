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

#include <pcl/features/normal_3d.h>


namespace csm
{
namespace perception
{


template<typename P, typename M>
void TraversibilityGenerator<P, M>::configure(
    float normal_estimation_radius,
    float output_res,
    float grad_search_radius,
    float min_grad_diff,
    float non_traversible_grad_angle,
    float avoidance_radius,
    float trav_score_curvature_weight,
    float trav_score_grad_weight,
    int interp_sample_count)
{
    this->normal_estimation_radius = normal_estimation_radius;
    this->output_res = output_res;
    this->grad_search_radius = grad_search_radius;
    this->min_grad_diff = min_grad_diff;
    this->non_trav_grad_thresh =
        1.f - std::cos(non_traversible_grad_angle * (M_PI / 180.));
    this->avoidance_radius = avoidance_radius;
    this->avoid_radius_sqrd = avoidance_radius * avoidance_radius;
    this->trav_score_curvature_weight = trav_score_curvature_weight;
    this->trav_score_grad_weight = trav_score_grad_weight;
    this->interp_sample_count = interp_sample_count;
}

template<typename P, typename M>
void TraversibilityGenerator<P, M>::processMapPoints(
    const TravPointCloud& map_points,
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


template<typename P, typename M>
typename TraversibilityGenerator<P, M>::TravPointCloud&
    TraversibilityGenerator<P, M>::copyPoints(TravPointCloud& copy_cloud) const
{
    this->mtx.lock();
    copy_cloud = this->points;
    this->mtx.unlock();

    return copy_cloud;
}
template<typename P, typename M>
typename TraversibilityGenerator<P, M>::MetaDataList&
    TraversibilityGenerator<P, M>::copyMetaDataList(
        MetaDataList& copy_list) const
{
    this->mtx.lock();
    copy_list = this->points_meta.points;
    this->mtx.unlock();

    return copy_list;
}

template<typename P, typename M>
typename TraversibilityGenerator<P, M>::TravPointCloud&
    TraversibilityGenerator<P, M>::swapPoints(TravPointCloud& swap_cloud)
{
    this->mtx.lock();
    std::swap(this->points.points, swap_cloud.points);
    std::swap(this->points.height, swap_cloud.height);
    std::swap(this->points.width, swap_cloud.width);
    std::swap(this->points.is_dense, swap_cloud.is_dense);
    this->mtx.unlock();

    return swap_cloud;
}
template<typename P, typename M>
typename TraversibilityGenerator<P, M>::MetaDataList&
    TraversibilityGenerator<P, M>::swapMetaDataList(MetaDataList& swap_list)
{
    this->mtx.lock();
    std::swap(this->points_meta.points, swap_list);
    this->mtx.unlock();

    return swap_list;
}

template<typename P, typename M>
void TraversibilityGenerator<P, M>::extractTravElements(
    TravPointCloud& trav_points,
    MetaDataList& trav_meta_data) const
{
    trav_points.clear();
    trav_meta_data.clear();

    this->mtx.lock();
    util::pc_copy_selection(this->points, this->trav_selection, trav_points);
    util::pc_copy_selection(
        this->points_meta.points,
        this->trav_selection,
        trav_meta_data);
    this->mtx.unlock();
}
template<typename P, typename M>
void TraversibilityGenerator<P, M>::extractExtTravElements(
    TravPointCloud& ext_trav_points,
    MetaDataList& ext_trav_meta_data) const
{
    ext_trav_points.clear();
    ext_trav_meta_data.clear();

    this->mtx.lock();
    util::pc_copy_selection(
        this->points,
        this->ext_trav_selection,
        ext_trav_points);
    util::pc_copy_selection(
        this->points_meta.points,
        this->ext_trav_selection,
        ext_trav_meta_data);
    this->mtx.unlock();
}
template<typename P, typename M>
void TraversibilityGenerator<P, M>::extractNonTravElements(
    TravPointCloud& non_trav_points,
    MetaDataList& non_trav_meta_data) const
{
    non_trav_points.clear();
    non_trav_meta_data.clear();

    this->mtx.lock();
    util::pc_copy_selection(
        this->points,
        this->avoid_selection,
        non_trav_points);
    util::pc_copy_selection(
        this->points_meta.points,
        this->avoid_selection,
        non_trav_meta_data);
    this->mtx.unlock();
}

template<typename P, typename M>
pcl::Indices& TraversibilityGenerator<P, M>::swapTravIndices(
    pcl::Indices& swap_indices)
{
    this->mtx.lock();
    std::swap(this->trav_selection, swap_indices);
    this->mtx.unlock();

    return swap_indices;
}
template<typename P, typename M>
pcl::Indices& TraversibilityGenerator<P, M>::swapExtTravIndices(
    pcl::Indices& swap_indices)
{
    this->mtx.lock();
    std::swap(this->ext_trav_selection, swap_indices);
    this->mtx.unlock();

    return swap_indices;
}
template<typename P, typename M>
pcl::Indices& TraversibilityGenerator<P, M>::swapNonTravIndices(
    pcl::Indices& swap_indices)
{
    this->mtx.lock();
    std::swap(this->avoid_selection, swap_indices);
    this->mtx.unlock();

    return swap_indices;
}



template<typename P, typename M>
void TraversibilityGenerator<P, M>::process(
    const TravPointCloud& map_points,
    const Vec3f& map_min_bound,
    const Vec3f& map_max_bound,
    const Vec3f& map_grav_vec,
    const Vec3f& source_pos)
{
    pcl::Indices cell_indices_buff, nearest_indices_buff;
    std::vector<float> dists_sqrd_buff;

    this->points.clear();
    this->points_meta.clear();
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
        if (cell_indices_buff.size() >= 3 &&
            pcl::compute3DCentroid(map_points, cell_indices_buff, centroid))
        {
            this->points.points.emplace_back().getVector3fMap() =
                centroid.template head<3>();
        }
    }

    this->points.height = this->points.points.size();
    this->points.width = 1;
    this->points_meta.points.resize(
        this->points.points.size(),
        TRAV_MIN_WEIGHT);

    // 4. REBUILD KDTREE
    this->points_ptr = util::wrap_unmanaged(this->points);
    this->interp_search_tree.setInputCloud(this->points_ptr);

    // 5. FILTER NON-TRAV POINTS
    for (size_t i = 0; i < this->points.size(); i++)
    {
        auto& pt = this->points.points[i];
        auto& meta = this->points_meta.points[i];

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
            trav_weight(meta) = NON_TRAV_WEIGHT;
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
    PointT center_pt{0.f, 0.f, source_pos.z()};
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
                    if (trav_weight(this->points_meta.points[idx]) >
                        TRAV_MIN_WEIGHT)
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
                    auto& interp_meta = this->points_meta.points.emplace_back();

                    interp_pt.x = center_pt.x;
                    interp_pt.y = center_pt.y;
                    interp_pt.z = sum_z / samples;
                    trav_weight(interp_meta) = TRAV_MIN_WEIGHT;
                }
            }
            cell_i++;
        }
    }

    this->points.height = this->points.points.size();
    this->points.width = 1;
    this->points_meta.height = this->points_meta.points.size();
    this->points_meta.width = 1;

    // 8. REBUILD TRAV KDTREE
    this->trav_search_tree.setInputCloud(
        this->points_ptr,
        util::wrap_unmanaged(this->trav_selection));

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
            const float weight =
                TRAV_MAX_WEIGHT +
                (EXT_TRAV_MAX_WEIGHT - TRAV_MAX_WEIGHT) *
                    (1.f - (dists_sqrd_buff[i] / this->avoid_radius_sqrd));
            if (weight > trav_weight(this->points_meta.points[idx]))
            {
                trav_weight(this->points_meta.points[idx]) = weight;
            }
        }
    }

    // 10. REINDEX TRAV SELECTION AND COMPUTE NORMALS
    for (size_t i = 0; i < this->trav_selection.size(); i++)
    {
        const auto& pt_idx = this->trav_selection[i];
        const auto& pt = this->points.points[pt_idx];
        auto& pt_meta = this->points_meta.points[pt_idx];

        if (trav_weight(pt_meta) > TRAV_MAX_WEIGHT)
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
            pcl::computePointNormal(
                this->points,
                nearest_indices_buff,
                plane,
                pt_meta.curvature);

            pt_meta.getNormalVector3fMap() = plane.template head<3>();
            trav_weight(pt_meta) =
                (pt_meta.curvature * this->trav_score_curvature_weight) +
                (1.f - map_grav_vec.dot(pt_meta.getNormalVector3fMap())) *
                    this->trav_score_grad_weight;
        }
    }
}

};  // namespace perception
};  // namespace csm
