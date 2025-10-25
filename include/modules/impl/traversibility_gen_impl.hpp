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


template<typename Float_T = float, typename Index_T = int32_t>
class GridMeta
{
    static_assert(std::is_floating_point<Float_T>::value);

public:
    using FloatT = Float_T;
    using IndexT = Index_T;

    template<typename T>
    using Vec2 = Eigen::Vector<T, 2>;
    template<typename T>
    using Vec3 = Eigen::Vector<T, 3>;
    template<int D>
    using VecF = Eigen::Vector<FloatT, D>;

    using Vec2f = Vec2<FloatT>;
    using Vec3f = Vec3<FloatT>;
    using Vec2i = Vec2<IndexT>;
    using Vec3i = Vec3<IndexT>;
    using CellPos2 = Vec2i;
    using CellPos3 = Vec3i;

public:
    inline GridMeta() = default;
    inline ~GridMeta() = default;

public:
    template<int D>
    inline void
        reconfigure(const VecF<D>& min, const VecF<D>& max, FloatT cell_res)
    {
        static_assert(D == 2 || D == 3);

        this->reconfigure(min, max, VecF<D>::Constant(cell_res));
    }

    inline void
        reconfigure(const Vec2f& min, const Vec2f& max, const Vec2f& cell_res)
    {
        this->origin << min, 0.f;
        this->cell_res << cell_res, 0.f;
        this->dim.x() =
            static_cast<IndexT>(std::ceil((max.x() - min.x()) / cell_res.x()));
        this->dim.y() =
            static_cast<IndexT>(std::ceil((max.y() - min.y()) / cell_res.y()));
        this->dim.z() = 1;
    }
    inline void
        reconfigure(const Vec3f& min, const Vec3f& max, const Vec3f& cell_res)
    {
        this->origin = min;
        this->cell_res = cell_res;
        this->dim.x() =
            static_cast<IndexT>(std::ceil((max.x() - min.x()) / cell_res.x()));
        this->dim.y() =
            static_cast<IndexT>(std::ceil((max.y() - min.y()) / cell_res.y()));
        this->dim.z() =
            static_cast<IndexT>(std::ceil((max.z() - min.z()) / cell_res.z()));
    }

    template<typename T>
    inline void initBuffer2(std::vector<T>& buff) const
    {
        buff.resize(this->size2());
    }
    template<typename T>
    inline void initBuffer2(std::vector<T>& buff, const T& val) const
    {
        buff.resize(this->size2(), val);
    }
    template<typename T>
    inline void initBuffer3(std::vector<T>& buff) const
    {
        buff.resize(this->size3());
    }
    template<typename T>
    inline void initBuffer3(std::vector<T>& buff, const T& val) const
    {
        buff.resize(this->size3(), val);
    }

    inline IndexT flattenIdx(const CellPos2& pos) const
    {
        return (pos.y() * this->dim.x()) + pos.x();
    }
    inline IndexT flattenIdx(const CellPos3& pos) const
    {
        return (this->dim.x() * (this->dim.y() * pos.z() + pos.y())) + pos.x();
    }
    inline CellPos2 expandIdx2(IndexT idx) const
    {
        return this->expandIdx3().template head<2>();
    }
    inline CellPos3 expandIdx3(IndexT idx) const
    {
        IndexT xy_area = this->dim.x() * this->dim.y();
        IndexT idx_mod_xy = idx % xy_area;
        return CellPos3{
            (idx_mod_xy % this->dim.x()),
            (idx_mod_xy / this->dim.x()),
            (idx / xy_area)};
    }

    inline CellPos2 getBoundingCellPos(const Vec2f& pt) const
    {
        return CellPos2{
            static_cast<IndexT>(
                std::floor((pt.x() - this->origin.x()) / this->cell_res.x())),
            static_cast<IndexT>(
                std::floor((pt.y() - this->origin.y()) / this->cell_res.y())),
        };
    }
    inline CellPos3 getBoundingCellPos(const Vec3f& pt) const
    {
        return CellPos3{
            static_cast<IndexT>(
                std::floor((pt.x() - this->origin.x()) / this->cell_res.x())),
            static_cast<IndexT>(
                std::floor((pt.y() - this->origin.y()) / this->cell_res.y())),
            this->cell_res.z() == 0.
                ? 0
                : static_cast<IndexT>(std::floor(
                      (pt.z() - this->origin.z()) / this->cell_res.z())),
        };
    }
    template<int D>
    inline IndexT getBoundingCellIdx(const VecF<D>& pt) const
    {
        static_assert(D == 2 || D == 3);

        return this->flattenIdx(this->getBoundingCellPos(pt));
    }

    template<typename T, int D>
    inline typename std::vector<T>::reference getCell(
        std::vector<T>& data,
        const VecF<D>& pt) const
    {
        static_assert(D == 2 || D == 3);

        return data[this->getBoundingCellIdx(pt)];
    }
    template<typename T, int D>
    inline const typename std::vector<T>::reference getCell(
        const std::vector<T>& data,
        const VecF<D>& pt) const
    {
        static_assert(D == 2 || D == 3);

        return data[this->getBoundingCellIdx(pt)];
    }

    inline Vec2f getCellCenter2(const CellPos2& pos) const
    {
        return this->origin.template head<2>() +
               (pos.template cast<FloatT>() + Vec2f::Constant(0.5f))
                   .cwiseProduct(this->cell_res.template head<2>());
    }
    inline Vec3f getCellCenter3(const CellPos3& pos) const
    {
        return this->origin +
               (pos.template cast<FloatT>() + Vec3f::Constant(0.5f))
                   .cwiseProduct(this->cell_res);
    }

    inline bool isInBoundingBox(const Vec2f& pt) const
    {
        return (
            pt.x() >= this->origin.x() &&
            pt.x() < this->origin.x() + (this->dim.x() * this->cell_res.x()) &&
            pt.y() >= this->origin.y() &&
            pt.y() < this->origin.y() + (this->dim.y() * this->cell_res.y()));
    }
    inline bool isInBoundingBox(const Vec3f& pt) const
    {
        return (pt.x() >= this->origin.x() &&
                pt.x() <
                    this->origin.x() + (this->dim.x() * this->cell_res.x()) &&
                pt.y() >= this->origin.y() &&
                pt.y() <
                    this->origin.y() + (this->dim.y() * this->cell_res.y())) &&
               (this->cell_res.z() == 0. ||
                (pt.z() >= this->origin.z() &&
                 pt.z() <
                     this->origin.z() + (this->dim.z() * this->cell_res.z())));
    }

    inline const Vec2f cellRes2() const
    {
        return this->cell_res.template head<2>();
    }
    inline const Vec3f& cellRes3() const { return this->cell_res; }
    inline size_t size2() const { return static_cast<size_t>(this->maxIdx2()); }
    inline size_t size3() const { return static_cast<size_t>(this->maxIdx3()); }

protected:
    inline IndexT maxIdx2() const
    {
        return this->dim.template head<2>().prod();
    }
    inline IndexT maxIdx3() const { return this->dim.prod(); }

protected:
    Vec3f origin;
    Vec3f cell_res;
    Vec3i dim;
};



namespace csm
{
namespace perception
{

template<typename P, typename M>
TraversibilityGenerator<P, M>::TraversibilityGenerator(
    uint32_t norm_est_threads,
    int32_t norm_est_chunk_size) :
#if PCL_VERSION >= PCL_VERSION_CALC(1, 14, 0)
    normal_estimation{norm_est_threads, norm_est_chunk_size}
#else
    normal_estimation{norm_est_threads}
#endif
{
    this->normal_estimation.setSearchMethod(
        util::wrap_unmanaged(this->neo_search_tree));

    (void)norm_est_chunk_size;
}


template<typename P, typename M>
void TraversibilityGenerator<P, M>::configure(
    float normal_estimation_radius,
    float interp_grid_res,
    float grad_search_radius,
    float min_grad_diff,
    float non_traversible_grad_angle,
    float required_clearance,
    float avoidance_radius,
    int avoid_min_num_points,
    int interp_sample_count)
{
    this->interp_grid_res = interp_grid_res;
    this->grad_search_radius = grad_search_radius;
    this->min_grad_diff = min_grad_diff;
    this->non_trav_grad_thresh =
        1.f - std::cos(non_traversible_grad_angle * (M_PI / 180.));
    this->required_clearance = required_clearance;
    this->avoidance_radius = avoidance_radius;
    this->avoid_min_num_points = avoid_min_num_points;
    this->interp_sample_count = interp_sample_count;

    this->normal_estimation.setRadiusSearch(normal_estimation_radius);
}

template<typename P, typename M>
void TraversibilityGenerator<P, M>::processMapPoints(
    const TravPointCloud& map_points,
    const Vec3f& map_min_bound,
    const Vec3f& map_max_bound,
    const Vec3f& map_grav_vec,
    const Vec3f& source_pos,
    const MetaDataList* map_normals)
{
    std::lock_guard lock_{this->mtx};
    this->trav_points = map_points;
    if (map_normals && map_normals->size() == map_points.size())
    {
        this->trav_points_meta.points = *map_normals;
    }
    else
    {
        this->trav_points_meta.clear();
    }
    this->process(map_min_bound, map_max_bound, map_grav_vec, source_pos);
}

template<typename P, typename M>
void TraversibilityGenerator<P, M>::processMapPoints(
    TravPointCloud& map_points,
    const Vec3f& map_min_bound,
    const Vec3f& map_max_bound,
    const Vec3f& map_grav_vec,
    const Vec3f& source_pos,
    MetaDataList* map_normals)
{
    this->swapPoints(map_points);
    std::lock_guard lock_{this->mtx};
    if (map_normals && map_normals->size() == map_points.size())
    {
        std::swap(this->trav_points_meta.points, *map_normals);
    }
    else
    {
        this->trav_points_meta.clear();
    }
    this->process(map_min_bound, map_max_bound, map_grav_vec, source_pos);
}


template<typename P, typename M>
typename TraversibilityGenerator<P, M>::TravPointCloud&
    TraversibilityGenerator<P, M>::copyPoints(TravPointCloud& copy_cloud) const
{
    this->mtx.lock();
    copy_cloud = this->trav_points;
    this->mtx.unlock();

    return copy_cloud;
}
template<typename P, typename M>
typename TraversibilityGenerator<P, M>::MetaDataList&
    TraversibilityGenerator<P, M>::copyMetaDataList(
        MetaDataList& copy_list) const
{
    this->mtx.lock();
    copy_list = this->trav_points_meta.points;
    this->mtx.unlock();

    return copy_list;
}

template<typename P, typename M>
typename TraversibilityGenerator<P, M>::TravPointCloud&
    TraversibilityGenerator<P, M>::swapPoints(TravPointCloud& swap_cloud)
{
    this->mtx.lock();
    std::swap(this->trav_points.points, swap_cloud.points);
    std::swap(this->trav_points.height, swap_cloud.height);
    std::swap(this->trav_points.width, swap_cloud.width);
    std::swap(this->trav_points.is_dense, swap_cloud.is_dense);
    this->mtx.unlock();

    return swap_cloud;
}
template<typename P, typename M>
typename TraversibilityGenerator<P, M>::MetaDataList&
    TraversibilityGenerator<P, M>::swapMetaDataList(MetaDataList& swap_list)
{
    this->mtx.lock();
    std::swap(this->trav_points_meta.points, swap_list);
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
    util::pc_copy_selection(
        this->trav_points,
        this->trav_selection,
        trav_points);
    util::pc_copy_selection(
        this->trav_points_meta.points,
        this->trav_selection,
        trav_meta_data);
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
        this->trav_points,
        this->avoid_selection,
        non_trav_points);
    util::pc_copy_selection(
        this->trav_points_meta.points,
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
pcl::Indices& TraversibilityGenerator<P, M>::swapNonTravIndices(
    pcl::Indices& swap_indices)
{
    this->mtx.lock();
    std::swap(this->avoid_selection, swap_indices);
    this->mtx.unlock();

    return swap_indices;
}


// template<typename P, typename M>
// void TraversibilityGenerator<P, M>::process(
//     const Vec3f& map_min_bound,
//     const Vec3f& map_max_bound,
//     const Vec3f& map_grav_vec,
//     const Vec3f& source_pos)
// {
// #define CLEAR_AND_RESERVE(vec, size)
//     vec.clear();
//     vec.reserve(size);

//     pcl::Indices nearest_indices_buff;
//     std::vector<float> dists_sqrd_buff, cached_trav_weights;

//     typename TravPointCloud::Ptr trav_points_ptr =
//         util::wrap_unmanaged(this->trav_points);

//     // 1. ESTIMATE NORMALS FOR EACH INPUT POINT --------------------------------
//     this->trav_points_meta.clear();
//     this->neo_search_tree.setInputCloud(trav_points_ptr);
//     if (this->trav_points_meta.points.size() != this->trav_points.size())
//     {
//         this->normal_estimation.setInputCloud(trav_points_ptr);
//         this->normal_estimation.compute(this->trav_points_meta);
//     }

//     CLEAR_AND_RESERVE(this->trav_selection, this->trav_points_meta.size())
//     CLEAR_AND_RESERVE(this->avoid_selection, this->trav_points_meta.size())
//     CLEAR_AND_RESERVE(this->interp_selection, this->trav_points_meta.size())
//     // CLEAR_AND_RESERVE(this->unknown_selection, this->trav_points_meta.size())
//     cached_trav_weights.resize(this->trav_points_meta.size());


//     // 2. COMPUTE TRAV WEIGHT, CATEGORIZE POINTS -------------------------------
//     for (size_t i = 0; i < this->trav_points_meta.size(); i++)
//     {
//         MetaT& m = this->trav_points_meta.points[i];

//         // normal estimation sets normal and curvature to quiet_NaN() on error
//         if (std::isnan(m.curvature))
//         {
//             trav_weight(m) = UNKNOWN_WEIGHT;
//             // this->unknown_selection.push_back(i);
//             this->trav_selection.push_back(i);
//         }
//         else
//         {
//             // lower = more traversible
//             trav_weight(m) =
//                 (1.f - std::abs(m.getNormalVector3fMap().dot(map_grav_vec)));

//             if (trav_weight(m) >= this->non_trav_grad_thresh)
//             {
//                 trav_weight(m) = NON_TRAV_WEIGHT;
//                 this->avoid_selection.push_back(i);
//             }
//             else
//             {
//                 this->trav_selection.push_back(i);
//                 this->interp_selection.push_back(i);
//             }
//         }

//         cached_trav_weights[i] = trav_weight(m);
//     }


//     // 3. BIN ALL POINTS INTO 2D GRID ------------------------------------------
//     GridMeta<float> grid_meta;
//     grid_meta.reconfigure(map_min_bound, map_max_bound, this->interp_grid_res);
//     this->interp_index_map.clear();
//     grid_meta.initBuffer2(this->interp_index_map, 0);

//     size_t n_interp_cells = grid_meta.size();
//     for (const auto& pt3 : this->trav_points)
//     {
//         auto& v = grid_meta.getCell(
//             this->interp_index_map,
//             pt3.getVector3fMap().template head<2>());

//         if (!v)
//         {
//             n_interp_cells--;
//             v--;
//         }
//     }


//     // 4. PREALLOCATE SPACE FOR INTERPOLATED POINTS, BUILD INTERP KDTREE -------
//     this->trav_points.reserve(this->trav_points.size() + n_interp_cells);
//     this->trav_points_meta.reserve(
//         this->trav_points_meta.size() + n_interp_cells);
//     this->trav_selection.reserve(this->trav_selection.size() + n_interp_cells);

//     this->interp_search_tree.setInputCloud(
//         trav_points_ptr,
//         util::wrap_unmanaged(this->interp_selection));


//     // 5. TRAVERSE GRID, ADD INTERPOLATED POINTS AND METADATA (TRAV SELECTION) -
//     const Eigen::Vector2f origin_cell_center =
//         grid_meta.getCellCenter(Eigen::Vector2i{0, 0});

//     size_t i = 0;
//     // n_interp_cells = 0;
//     PointT center_pt{0.f, 0.f, source_pos.z()};
//     for (center_pt.y = origin_cell_center.y();  //
//          center_pt.y < map_max_bound.y();
//          center_pt.y += this->interp_grid_res)
//     {
//         for (center_pt.x = origin_cell_center.x();
//              center_pt.x < map_max_bound.x();
//              center_pt.x += this->interp_grid_res)
//         {
//             int32_t& mapped_idx = this->interp_index_map[i];
//             if (!mapped_idx &&  //
//                 this->interp_search_tree.nearestKSearch(
//                     center_pt,
//                     this->interp_sample_count,
//                     nearest_indices_buff,
//                     dists_sqrd_buff))
//             {
//                 this->trav_selection.push_back(this->trav_points.size());
//                 mapped_idx = static_cast<int32_t>(this->trav_selection.back());
//                 PointT& interp_pt = this->trav_points.points.emplace_back();
//                 MetaT& interp_pt_meta =
//                     this->trav_points_meta.points.emplace_back();

//                 interp_pt.x = center_pt.x;
//                 interp_pt.y = center_pt.y;
//                 interp_pt.z = 0.f;
//                 for (pcl::index_t nn_idx : nearest_indices_buff)
//                 {
//                     const auto& m = this->trav_points_meta.points[nn_idx];
//                     const auto n = m.getNormalVector3fMap();
//                     const auto b =
//                         this->trav_points.points[nn_idx].getVector3fMap();

//                     interp_pt.z +=
//                         ((n.dot(b) - (n.x() * interp_pt.x) -
//                           (n.y() * interp_pt.y)) /
//                          n.z());
//                     interp_pt_meta.getNormalVector3fMap() += n;
//                     interp_pt_meta.curvature += m.curvature;
//                     trav_weight(interp_pt_meta) += trav_weight(m);
//                 }
//                 interp_pt.z /= nearest_indices_buff.size();
//                 interp_pt_meta.getNormalVector3fMap() /=
//                     nearest_indices_buff.size();
//                 interp_pt_meta.curvature /= nearest_indices_buff.size();
//                 trav_weight(interp_pt_meta) /= nearest_indices_buff.size();

//                 // n_interp_cells++;
//             }
//             else
//             {
//                 mapped_idx = -1;
//             }
//             i++;
//         }
//     }


//     // // 6. APPLY COLLISION RADIUS TO TRAVERSAL POINTS, UPDATE SELECTIONS --------
//     // size_t last_trav_selection_i = 0;
//     // for (size_t i = 0; i < this->trav_selection.size(); i++)
//     // {
//     //     const auto& pt_idx = this->trav_selection[i];
//     //     auto& src_pt_meta = this->trav_points_meta.points[pt_idx];

//     //     // TODO : clearance handling

//     //     // search for all points within collision radius
//     //     this->neo_search_tree.radiusSearch(
//     //         this->trav_points.points[pt_idx],
//     //         this->avoidance_radius,
//     //         nearest_indices_buff,
//     //         dists_sqrd_buff);

//     //     // set trav weight to the max of all trav weights in range
//     //     bool remove = false;
//     //     for (pcl::index_t nn_idx : nearest_indices_buff)
//     //     {
//     //         const float wt = cached_trav_weights[nn_idx];
//     //         if (wt == NON_TRAV_WEIGHT)
//     //         {
//     //             // if in range of avoidance point, set for removal and break
//     //             trav_weight(src_pt_meta) = wt;
//     //             remove = true;
//     //             break;
//     //         }
//     //         else if (wt > trav_weight(src_pt_meta))
//     //         {
//     //             trav_weight(src_pt_meta) = wt;
//     //         }
//     //     }

//     //     if (remove)
//     //     {
//     //         // if need to remove, add to selection and keep
//     //         // last_trav_selection_i the same
//     //         this->avoid_selection.push_back(pt_idx);
//     //     }
//     //     else
//     //     {
//     //         // if keeping this point, copy into last_trav_selection_i if it has
//     //         // fallen behind and iterate
//     //         if (last_trav_selection_i < i)
//     //         {
//     //             this->trav_selection[last_trav_selection_i] = pt_idx;
//     //         }
//     //         last_trav_selection_i++;
//     //     }
//     // }
//     // this->trav_selection.resize(last_trav_selection_i);
//     // // sort avoid_selection indices
// }

// template<typename P, typename M>
// void TraversibilityGenerator<P, M>::process(
//     const Vec3f& map_min_bound,
//     const Vec3f& map_max_bound,
//     const Vec3f& map_grav_vec,
//     const Vec3f& source_pos)
// {
//     pcl::Indices nearest_indices_buff;
//     std::vector<float> dists_sqrd_buff, cached_trav_weights;

//     TravPointCloud out_vox_points, sample_vox_points;
//     MetaDataCloud out_vox_meta;

//     std::unordered_map<uint32_t, pcl::index_t> out_vox_indices;

//     typename TravPointCloud::Ptr out_vox_points_ptr =
//         util::wrap_unmanaged(out_vox_points);
//     typename TravPointCloud::Ptr sample_vox_points_ptr =
//         util::wrap_unmanaged(sample_vox_points);

//     pcl::search::KdTree<P> interp_search_tree;
//     pcl::search::KdTree<P> sample_search_tree;

//     // typename TravPointCloud::Ptr trav_points_ptr =
//     //     util::wrap_unmanaged(this->trav_points);
//     // this->neo_search_tree.setInputCloud(trav_points_ptr);

//     GridMeta<float> output_grid, sampling_grid;
//     output_grid.reconfigure(
//         map_min_bound,
//         map_max_bound,
//         this->interp_grid_res);
//     sampling_grid.reconfigure(
//         map_min_bound,
//         map_max_bound,
//         this->interp_grid_res / 2.f);

//     using IndicesPair = std::pair<pcl::index_t, uint32_t>;
//     // using IndicesTuple = std::tuple<pcl::index_t, uint32_t, uint32_t>;
//     std::vector<IndicesPair> out_pt_indices, sample_pt_indices;
//     out_pt_indices.reserve(this->trav_points.size());
//     sample_pt_indices.reserve(this->trav_points.size());
//     for (size_t i = 0; i < this->trav_points.size(); i++)
//     {
//         const auto& pt3 = this->trav_points.points[i];
//         const auto out_idx =
//             output_grid.getBoundingCellIdx<3>(pt3.getVector3fMap());
//         const auto sample_idx =
//             sampling_grid.getBoundingCellIdx<3>(pt3.getVector3fMap());
//         if (out_idx >= 0 && sample_idx >= 0)
//         {
//             out_pt_indices.emplace_back(
//                 static_cast<pcl::index_t>(i),
//                 static_cast<uint32_t>(out_idx));
//             sample_pt_indices.emplace_back(
//                 static_cast<pcl::index_t>(i),
//                 static_cast<uint32_t>(sample_idx));
//         }
//     }

//     auto sort_f = [](const IndicesPair& a, const IndicesPair& b)
//     { return a.second < b.second; };
//     std::sort(out_pt_indices.begin(), out_pt_indices.end(), sort_f);
//     std::sort(sample_pt_indices.begin(), sample_pt_indices.end(), sort_f);

//     pcl::Indices cell_indices;
//     for (size_t i = 0; i < out_pt_indices.size(); i++)
//     {
//         cell_indices.clear();
//         uint32_t curr_cell_idx = out_pt_indices[i].second;

//         for (; i < out_pt_indices.size(); i++)
//         {
//             const auto& pair = out_pt_indices[i];
//             if (pair.second == curr_cell_idx)
//             {
//                 cell_indices.push_back(pair.first);
//             }
//             else
//             {
//                 i--;
//                 break;
//             }
//         }

//         // EIGEN_ALIGN16 Eigen::Matrix3f cov;
//         Eigen::Vector4f centroid;
//         if (cell_indices.size() >= 4 &&
//             pcl::compute3DCentroid(this->trav_points, cell_indices, centroid))
//         {
//             // Eigen::Vector4f plane;
//             // float curvature;
//             // pcl::solvePlaneParameters(cov, centroid, plane, curvature);

//             out_vox_indices.emplace(curr_cell_idx, out_vox_points.size());
//             auto& pt = out_vox_points.points.emplace_back();
//             // auto& meta = out_vox_meta.points.emplace_back();

//             pt.getVector3fMap() = centroid.template head<3>();
//             // meta.getNormalVector3fMap() = plane.template head<3>();
//             // meta.curvature = curvature;

//             // trav_weight(meta) =
//             //     1.f - meta.getNormalVector3fMap().dot(map_grav_vec);

//             // this->neo_search_tree.radiusSearch(
//             //     pt,
//             //     this->grad_search_radius,
//             //     nearest_indices_buff,
//             //     dists_sqrd_buff);

//             // pcl::computeMeanAndCovarianceMatrix(
//             //     this->trav_points,
//             //     nearest_indices_buff,
//             //     cov,
//             //     centroid);

//             // trav_weight(meta) = abs((cov * map_grav_vec).dot(map_grav_vec));

//             // Vec3f min, max;
//             // min = max = pt.getVector3fMap();
//             // for (pcl::index_t idx : nearest_indices_buff)
//             // {
//             //     const auto& pt_ = this->trav_points[idx];
//             //     if (pt_.z < min.z())
//             //     {
//             //         min = pt_.getVector3fMap();
//             //     }
//             //     if (pt_.z > max.z())
//             //     {
//             //         max = pt_.getVector3fMap();
//             //     }
//             // }

//             // Vec3f diff_norm = (max - min).normalized();
//             // Vec3f pt_diff_norm = (pt.getVector3fMap() - min).normalized();
//             // if (max.z() - min.z() >= this->min_grad_diff &&
//             //     std::abs(diff_norm.dot(map_grav_vec)) >
//             //         this->non_trav_grad_thresh)
//             // {
//             //     trav_weight(meta) = NON_TRAV_WEIGHT;
//             // }
//             // else
//             // {
//             //     trav_weight(meta) = 0.f;
//             // }
//         }
//     }
//     for (size_t i = 0; i < sample_pt_indices.size(); i++)
//     {
//         cell_indices.clear();
//         uint32_t curr_cell_idx = sample_pt_indices[i].second;

//         for (; i < sample_pt_indices.size(); i++)
//         {
//             const auto& pair = sample_pt_indices[i];
//             if (pair.second == curr_cell_idx)
//             {
//                 cell_indices.push_back(pair.first);
//             }
//             else
//             {
//                 i--;
//                 break;
//             }
//         }

//         Eigen::Vector4f centroid;
//         if (cell_indices.size() >= 2 &&
//             pcl::compute3DCentroid(this->trav_points, cell_indices, centroid))
//         {
//             sample_vox_points.points.emplace_back().getVector3fMap() =
//                 centroid.template head<3>();
//         }
//     }

//     out_vox_points.height = out_vox_points.points.size();
//     out_vox_points.width = 1;
//     sample_vox_points.height = out_vox_points.points.size();
//     sample_vox_points.width = 1;

//     out_vox_meta.resize(out_vox_points.points.size());
//     for (auto& meta : out_vox_meta.points)
//     {
//         trav_weight(meta) = 0.f;
//     }

//     // typename TravPointCloud::Ptr vox_points_ptr =
//     //     util::wrap_unmanaged(vox_points);
//     // this->neo_search_tree.setInputCloud(vox_points_ptr);

//     interp_search_tree.setInputCloud(out_vox_points_ptr);
//     sample_search_tree.setInputCloud(sample_vox_points_ptr);

//     // pcl::Indices radius_indices;
//     // std::vector<float> dists;
//     for (size_t i = 0; i < out_vox_points.size(); i++)
//     {
//         auto& pt = out_vox_points.points[i];
//         auto& meta = out_vox_meta.points[i];

//         sample_search_tree.radiusSearch(
//             pt,
//             this->grad_search_radius,
//             nearest_indices_buff,
//             dists_sqrd_buff);

//         Vec3f min, max;
//         min = max = pt.getVector3fMap();
//         for (pcl::index_t idx : nearest_indices_buff)
//         {
//             if (trav_weight(out_vox_meta.points[idx]) == NON_TRAV_WEIGHT)
//             {
//                 continue;
//             }

//             const auto& pt_ = sample_vox_points[idx];
//             if (pt_.z < min.z())
//             {
//                 min = pt_.getVector3fMap();
//             }
//             if (pt_.z > max.z())
//             {
//                 max = pt_.getVector3fMap();
//             }
//         }

//         Vec3f diff_norm = (max - min).normalized();
//         // Vec3f pt_diff_norm = (pt.getVector3fMap() - min).normalized();
//         if (max.z() - min.z() >= this->min_grad_diff &&
//             std::abs(diff_norm.dot(map_grav_vec)) > this->non_trav_grad_thresh)
//         // (pt_diff_norm.dot(diff_norm) > 0.9f ||
//         //  pt_diff_norm == Vec3f::Zero()))
//         {
//             // trav_weight(meta) = NON_TRAV_WEIGHT;
//             uint32_t min_out_idx = output_grid.getBoundingCellIdx<3>(min);
//             uint32_t max_out_idx = output_grid.getBoundingCellIdx<3>(max);

//             if (out_vox_indices.count(min_out_idx))
//             {
//                 trav_weight(
//                     out_vox_meta.points[out_vox_indices.at(min_out_idx)]) =
//                     NON_TRAV_WEIGHT;
//             }
//             if (out_vox_indices.count(max_out_idx))
//             {
//                 trav_weight(
//                     out_vox_meta.points[out_vox_indices.at(max_out_idx)]) =
//                     NON_TRAV_WEIGHT;
//             }
//         }
//         // else
//         // {
//         //     trav_weight(meta) = 0.f;
//         // }
//     }

//     // std::vector<size_t> spread_indices;
//     // for (size_t i = 0; i < vox_points.size(); i++)
//     // {
//     //     auto& pt = vox_points.points[i];
//     //     auto& meta = vox_points_meta.points[i];

//     //     if (trav_weight(meta) == NON_TRAV_WEIGHT)
//     //     {
//     //         continue;
//     //     }

//     //     this->neo_search_tree
//     //         .radiusSearch(pt, this->avoidance_radius, radius_indices, dists);

//     //     int avoid_count = 0;
//     //     for (pcl::index_t idx : radius_indices)
//     //     {
//     //         if (trav_weight(vox_points_meta.points[idx]) == NON_TRAV_WEIGHT)
//     //         {
//     //             if (++avoid_count >= this->avoid_min_num_points)
//     //             {
//     //                 break;
//     //             }
//     //         }
//     //     }

//     //     if (avoid_count >= this->avoid_min_num_points)
//     //     {
//     //         spread_indices.push_back(i);
//     //     }
//     // }
//     // for (size_t i : spread_indices)
//     // {
//     //     trav_weight(vox_points_meta.points[i]) = NON_TRAV_WEIGHT;
//     // }

//     this->trav_points.swap(out_vox_points);
//     this->trav_points_meta.swap(out_vox_meta);
// }

// template<typename P, typename M>
// void TraversibilityGenerator<P, M>::process(
//     const Vec3f& map_min_bound,
//     const Vec3f& map_max_bound,
//     const Vec3f& map_grav_vec,
//     const Vec3f& source_pos)
// {
//     TravPointCloud out_vox_points, sample_vox_points;
//     MetaDataCloud out_vox_meta;

//     GridMeta<float> output_grid, sampling_grid;
//     output_grid.reconfigure(
//         map_min_bound,
//         map_max_bound,
//         this->interp_grid_res);
//     sampling_grid.reconfigure(
//         map_min_bound,
//         map_max_bound,
//         this->interp_grid_res / 2.f);

//     using IndicesPair = std::pair<pcl::index_t, uint32_t>;
//     std::vector<IndicesPair> out_pt_indices, sample_pt_indices;

//     pcl::octree::OctreePointCloudSearch<P> sampling_octree{
//         this->interp_grid_res / 2.f};

//     // voxelize at output resolution and sampling resolution
//     // --> minimum number of source points required for voxel creation

//     out_pt_indices.reserve(this->trav_points.size());
//     sample_pt_indices.reserve(this->trav_points.size());
//     for (size_t i = 0; i < this->trav_points.size(); i++)
//     {
//         const auto& pt3 = this->trav_points.points[i];
//         const auto out_idx =
//             output_grid.getBoundingCellIdx<3>(pt3.getVector3fMap());
//         const auto sample_idx =
//             sampling_grid.getBoundingCellIdx<3>(pt3.getVector3fMap());
//         if (out_idx >= 0 && sample_idx >= 0)
//         {
//             out_pt_indices.emplace_back(
//                 static_cast<pcl::index_t>(i),
//                 static_cast<uint32_t>(out_idx));
//             sample_pt_indices.emplace_back(
//                 static_cast<pcl::index_t>(i),
//                 static_cast<uint32_t>(sample_idx));
//         }
//     }

//     auto sort_f = [](const IndicesPair& a, const IndicesPair& b)
//     { return a.second < b.second; };
//     std::sort(out_pt_indices.begin(), out_pt_indices.end(), sort_f);
//     std::sort(sample_pt_indices.begin(), sample_pt_indices.end(), sort_f);

//     static constexpr int MIN_OUT_VOX_INPUT_POINTS = 4;
//     static constexpr int MIN_SAMPLE_VOX_INPUT_POINTS = 2;

//     pcl::Indices cell_indices;
//     for (size_t i = 0; i < out_pt_indices.size(); i++)
//     {
//         cell_indices.clear();
//         uint32_t curr_cell_idx = out_pt_indices[i].second;

//         for (; i < out_pt_indices.size(); i++)
//         {
//             const auto& pair = out_pt_indices[i];
//             if (pair.second == curr_cell_idx)
//             {
//                 cell_indices.push_back(pair.first);
//             }
//             else
//             {
//                 i--;
//                 break;
//             }
//         }

//         Eigen::Vector4f centroid;
//         if (cell_indices.size() >= MIN_OUT_VOX_INPUT_POINTS &&
//             pcl::compute3DCentroid(this->trav_points, cell_indices, centroid))
//         {
//             out_vox_points.points.emplace_back().getVector3fMap() =
//                 centroid.template head<3>();
//         }
//     }
//     for (size_t i = 0; i < sample_pt_indices.size(); i++)
//     {
//         cell_indices.clear();
//         uint32_t curr_cell_idx = sample_pt_indices[i].second;

//         for (; i < sample_pt_indices.size(); i++)
//         {
//             const auto& pair = sample_pt_indices[i];
//             if (pair.second == curr_cell_idx)
//             {
//                 cell_indices.push_back(pair.first);
//             }
//             else
//             {
//                 i--;
//                 break;
//             }
//         }

//         Eigen::Vector4f centroid;
//         if (cell_indices.size() >= MIN_SAMPLE_VOX_INPUT_POINTS &&
//             pcl::compute3DCentroid(this->trav_points, cell_indices, centroid))
//         {
//             sample_vox_points.points.emplace_back().getVector3fMap() =
//                 centroid.template head<3>();
//         }
//     }

//     out_vox_points.height = out_vox_points.points.size();
//     out_vox_points.width = 1;
//     sample_vox_points.height = out_vox_points.points.size();
//     sample_vox_points.width = 1;

//     out_vox_meta.resize(out_vox_points.points.size());
//     for (auto& meta : out_vox_meta.points)
//     {
//         trav_weight(meta) = 0.f;
//     }

//     // build coordinate axes aligned with gravity
//     // transform sampling set to be grav-aligned
//     // build search octree over sampling set
//     static const Vec3f X = Vec3f{1.f, 0.f, 0.f};
//     static const Vec3f Y = Vec3f{0.f, 1.f, 0.f};
//     using Mat4f = Eigen::Matrix4f;

//     Mat4f t = Mat4f::Identity();
//     Vec3f a, b, c = map_grav_vec.normalized();

//     if (std::abs(c.dot(X)) > 0.99f)
//     {
//         // use y-axis as reference
//         a = c.cross(Y).normalized();
//         b = a.cross(c).normalized();
//     }
//     else
//     {
//         // use x-axis as reference
//         b = X.cross(c).normalized();
//         a = c.cross(b).normalized();
//     }

//     t.block<1, 3>(0, 0) = a.transpose();
//     t.block<1, 3>(1, 0) = b.transpose();
//     t.block<1, 3>(2, 0) = c.transpose();

//     pcl::transformPointCloud(sample_vox_points, sample_vox_points, t);
//     sampling_octree.setInputCloud(util::wrap_unmanaged(sample_vox_points));
//     sampling_octree.addPointsFromInputCloud();

//     // for each output point:
//     // search in "local column" --> min, max points
//     // if delta > thresh --> non-trav
//     for (size_t i = 0; i < out_vox_points.size(); i++)
//     {
//         const auto& pt = out_vox_points.points[i];
//         auto& meta = out_vox_meta.points[i];

//         Vec3f tf_pt = (t * pt.getVector4fMap()).template head<3>();
//         Vec3f off = Vec3f{
//             this->interp_grid_res,
//             this->interp_grid_res,
//             this->grad_search_radius};

//         if (sampling_octree.boxSearch(tf_pt - off, tf_pt + off, cell_indices))
//         {
//             float min_z, max_z;
//             min_z = max_z = tf_pt.z();
//             for (pcl::index_t idx : cell_indices)
//             {
//                 const auto& pt_ = sample_vox_points[idx];
//                 if (pt_.z < min_z)
//                 {
//                     min_z = pt_.z;
//                 }
//                 if (pt_.z > max_z)
//                 {
//                     max_z = pt_.z;
//                 }
//             }

//             if (max_z - tf_pt.z() > this->min_grad_diff ||
//                 tf_pt.z() - min_z > this->min_grad_diff)
//             {
//                 trav_weight(meta) = NON_TRAV_WEIGHT;
//             }
//         }
//     }

//     this->trav_points.swap(out_vox_points);
//     this->trav_points_meta.swap(out_vox_meta);
// }

template<typename P, typename M>
void TraversibilityGenerator<P, M>::process(
    const Vec3f& map_min_bound,
    const Vec3f& map_max_bound,
    const Vec3f& map_grav_vec,
    const Vec3f& source_pos)
{
    TravPointCloud vox_points;
    MetaDataCloud vox_points_meta;

    typename TravPointCloud::Ptr vox_points_ptr =
        util::wrap_unmanaged(vox_points);

    GridMeta<float> grid;

    using IndicesPair = std::pair<uint32_t, pcl::index_t>;
    std::vector<IndicesPair> cell_pt_indices;

    pcl::Indices cell_indices, nearest_indices_buff;
    std::vector<float> dists_sqrd_buff, cached_trav_weights;

    // 1. REGISTER GRID INDICES FOR EACH INPUT POINT
    grid.reconfigure(map_min_bound, map_max_bound, this->interp_grid_res);
    cell_pt_indices.reserve(this->trav_points.size());
    for (size_t i = 0; i < this->trav_points.size(); i++)
    {
        const auto& pt3 = this->trav_points.points[i];
        const auto idx = grid.getBoundingCellIdx<3>(pt3.getVector3fMap());
        if (idx >= 0)
        {
            cell_pt_indices.emplace_back(
                static_cast<uint32_t>(idx),
                static_cast<pcl::index_t>(i));
        }
    }

    // 2. SORT INDICES BY VOXEL INDEX
    std::sort(
        cell_pt_indices.begin(),
        cell_pt_indices.end(),
        [](const IndicesPair& a, const IndicesPair& b)
        { return a.first < b.first; });

    // 3. COMPUTE CENTROID FOR EACH VOXEL AND EXPORT
    for (size_t i = 0; i < cell_pt_indices.size(); i++)
    {
        cell_indices.clear();
        uint32_t curr_cell_idx = cell_pt_indices[i].first;

        for (; i < cell_pt_indices.size(); i++)
        {
            const auto& pair = cell_pt_indices[i];
            if (pair.first == curr_cell_idx)
            {
                cell_indices.push_back(pair.second);
            }
            else
            {
                i--;
                break;
            }
        }

        Eigen::Vector4f centroid;
        if (cell_indices.size() >= 3 &&
            pcl::compute3DCentroid(this->trav_points, cell_indices, centroid))
        {
            vox_points.points.emplace_back().getVector3fMap() =
                centroid.template head<3>();
        }
    }

    vox_points.height = vox_points.points.size();
    vox_points.width = 1;
    vox_points_meta.points.resize(vox_points.points.size(), 0.f);

    // 4. REBUILD KDTREE
    this->interp_search_tree.setInputCloud(vox_points_ptr);

    // 5. FILTER NON-TRAV POINTS
    this->trav_selection.clear();
    this->avoid_selection.clear();
    for (size_t i = 0; i < vox_points.size(); i++)
    {
        auto& pt = vox_points.points[i];
        auto& meta = vox_points_meta.points[i];

        this->interp_search_tree.radiusSearch(
            pt,
            this->grad_search_radius,
            nearest_indices_buff,
            dists_sqrd_buff);

        Vec3f min, max;
        min = max = pt.getVector3fMap();
        for (pcl::index_t idx : nearest_indices_buff)
        {
            const auto& pt_ = vox_points[idx];
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
    grid.initBuffer2(this->interp_index_map, 0);

    size_t n_interp_cells = grid.size2();
    for (const auto& pt3 : vox_points)
    {
        auto& v = grid.getCell(
            this->interp_index_map,
            Eigen::Vector2f{pt3.getVector3fMap().template head<2>()});

        if (!v)
        {
            n_interp_cells--;
            v--;
        }
    }

    // 7. ITERATE EMPTY CELLS AND FILL VIA INTERPOLATION
    const Eigen::Vector2f origin_cell_center =
        grid.getCellCenter2(Eigen::Vector2i::Constant(0));
    size_t cell_i = 0;
    PointT center_pt{0.f, 0.f, source_pos.z()};
    for (center_pt.y = origin_cell_center.y();  //
         center_pt.y < map_max_bound.y();
         center_pt.y += this->interp_grid_res)
    {
        for (center_pt.x = origin_cell_center.x();  //
             center_pt.x < map_max_bound.x();
             center_pt.x += this->interp_grid_res)
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
                    if (trav_weight(vox_points_meta.points[idx]) > 0.f)
                    {
                        continue;
                    }

                    sum_z += vox_points.points[idx].z;
                    samples++;
                }

                if (samples)
                {
                    this->trav_selection.push_back(vox_points.points.size());
                    auto& interp_pt = vox_points.points.emplace_back();
                    auto& interp_meta = vox_points_meta.points.emplace_back();

                    interp_pt.x = center_pt.x;
                    interp_pt.y = center_pt.y;
                    interp_pt.z = sum_z / samples;
                    trav_weight(interp_meta) = 0.f;
                }
            }
            cell_i++;
        }
    }

    vox_points.height = vox_points.points.size();
    vox_points.width = 1;
    vox_points_meta.height = vox_points_meta.points.size();
    vox_points_meta.width = 1;

    // 8. REBUILD NEO KDTREE
    this->neo_search_tree.setInputCloud(
        vox_points_ptr,
        util::wrap_unmanaged(this->trav_selection));

    // 9. COMPUTE NORMALS FOR TRAV SUBSET (INCLUDE INTERPOLATED)
    // this->normal_estimation.setInputCloud(vox_points_ptr, )

    // 10. LOOP AVOID POINTS, APPLY SPREAD RADIUS
    for (pcl::index_t avoid_i : this->avoid_selection)
    {
        this->neo_search_tree.radiusSearch(
            vox_points.points[avoid_i],
            this->avoidance_radius,
            nearest_indices_buff,
            dists_sqrd_buff);

        for (pcl::index_t idx : nearest_indices_buff)
        {
            trav_weight(vox_points_meta.points[idx]) = NON_TRAV_WEIGHT;
        }
    }

    // 11. REINDEX TRAV SELECTION AND COMPUTE NORMALS
    const float norm_search_radius = this->interp_grid_res * 2.f;
    for (size_t i = 0; i < this->trav_selection.size(); i++)
    {
        const auto& pt_idx = this->trav_selection[i];
        const auto& pt = vox_points.points[pt_idx];
        auto& pt_meta = vox_points_meta.points[pt_idx];

        if (trav_weight(pt_meta) == NON_TRAV_WEIGHT)
        {
            this->avoid_selection.push_back(pt_idx);
            this->trav_selection[i] = this->trav_selection.back();
            this->trav_selection.pop_back();
            i--;
        }
        else
        {
            this->neo_search_tree.radiusSearch(
                pt,
                norm_search_radius,
                nearest_indices_buff,
                dists_sqrd_buff);

            Eigen::Vector4f plane;
            pcl::computePointNormal(
                vox_points,
                nearest_indices_buff,
                plane,
                pt_meta.curvature);

            pt_meta.getNormalVector3fMap() = plane.template head<3>();
            trav_weight(pt_meta) =
                pt_meta.curvature + 1.f -
                map_grav_vec.dot(pt_meta.getNormalVector3fMap());
        }
    }

    this->trav_points.swap(vox_points);
    this->trav_points_meta.swap(vox_points_meta);
}

};  // namespace perception
};  // namespace csm
