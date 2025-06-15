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
#include <limits>
#include <vector>
#include <type_traits>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/octree/octree_search.h>
#include <pcl/features/normal_3d_omp.h>

#include "util.hpp"
#include "point_def.hpp"


namespace util
{

template<
    Float_T = float,
    Index_T = int32_t>
class GridMeta2
{
    static_assert(std::is_floating_point<Float_T>::value);

public:
    using FloatT = Float_T;
    using IndexT = Index_T;

    template<typename T>
    using Vec2 = Eigen::Vector<T, 2>;
    using Vec2f = Vec2<FloatT>;
    using Index2 = Vec2<IndexT>;
    using CellPos = Index2;

public:
    inline GridMeta2() = default;
    inline ~GridMeta2() = default;

public:
    inline void reconfigure(
        const Vec2f& min,
        const Vec2f& max,
        FloatT cell_res )
    {
        this->reconfigure(min, max, Vec2f::Constant(cell_res));
    }
    inline void reconfigure(
        const Vec2f& min,
        const Vec2f& max,
        const Vec2f& cell_res )
    {
        this->origin = min;
        this->cell_res = cell_res;
        this->dim.x() = static_cast<IndexT>(
            std::ceil((max.x() - min.x()) / cell_res.x()) );
        this->dim.y() = static_cast<IndexT>(
            std::ceil((max.y() - min.y()) / cell_res.y()) );
    }

    template<typename T>
    inline void initBuffer(std::vector<T>& buff) const
    {
        buff.resize(this->size());
    }
    template<typename T>
    inline void initBuffer(std::vector<T>& buff, const T& val) const
    {
        buff.resize(this->size(), val);
    }

    inline IndexT flattenIdx(const CellPos& pos) const
    {
        return (pos.x() * this->dim.y()) + pos.y();
    }
    inline CellPos expandIdx(IndexT idx) const
    {
        return CellPos{ (idx % this->dim.y()), (idx / this->dim.y()) };
    }

    inline CellPos getBoundingCellPos(const Vec2f& pt) const
    {
        return
            CellPos
            {
                static_cast<IndexT>(
                    std::floor(
                        (pt.x() - this->origin.x()) / this->cell_res.x() ) ),
                static_cast<IndexT>(
                    std::floor(
                        (pt.y() - this->origin.y()) / this->cell_res.y() ) ),
            };
    }
    inline IndexT getBoundingCellIdx(const Vec2f& pt) const
    {
        return this->flattenIdx(this->getBoundingCellPos(pt));
    }

    template<typename T>
    inline T& getCell(std::vector<T>& data, const Vec2f& pt) const
    {
        return data[this->getBoundingCellIdx(pt)];
    }
    template<typename T>
    inline const T& getCell(
        const std::vector<T>& data,
        const Vec2f& pt ) const
    {
        return data[this->getBoundingCellIdx(pt)];
    }

    inline Vec2f getCellCenter(const CellPos& pos) const
    {
        return this->origin +
            pos.template cast<FloatT>().cwiseProduct(this->cell_res);
    }

    inline bool isInBoundingBox(const Vec2f& pt) const
    {
        return (
            pt.x() >= this->origin.x() &&
            pt.x() < this->origin.x() + (this->dim.x() * this->cell_res.x()) &&
            pt.y() >= this->origin.y() &&
            pt.y() < this->origin.y() + (this->dim.y() * this->cell_res.y()) );
    }

    inline size_t size() const
    {
        return static_cast<size_t>(this->maxIdx());
    }

protected:
    inline IndexT maxIdx() const
    {
        return this->dim.prod();
    }

protected:
    Vec2f origin;
    Vec2f cell_res;
    Index2 dim;

};

};



#ifndef TRAVERSIBILITY_NON_TRAV_WEIGHT
#define TRAVERSIBILITY_NON_TRAV_WEIGHT std::numeric_limits<float>::infinity()
#endif
#ifndef TRAVERSIBILITY_UNKNOWN_WEIGHT
#define TRAVERSIBILITY_UNKNOWN_WEIGHT -1.f
#endif

namespace csm
{
namespace perception
{

template<
    typename Point_T = pcl::PointXYZ,
    typename Meta_T = csm::perception::NormalTraversal>
class TraversibilityGenerator
{
    static_assert(pcl::traits::has_xyz<Point_T>::value);
    static_assert(pcl::traits::has_normal<Meta_T>::value);
    static_assert(
        util::traits::has_trav_weight<Meta_T>::value ||
        pcl::traits::has_curvature<Meta_T>::value );

public:
    using PointT = Point_T;
    using MetaT = Meta_T;
    using NormalT = MetaT;
    using Vec3f = Eigen::Vector3f;

    using TravPointCloud = pcl::PointCloud<PointT>;
    using MetaDataCloud = pcl::PointCloud<MetaT>;
    using MetaDataList = typename MetaDataCloud::VectorType;

public:
    inline TraversibilityGenerator(
        uint32_t norm_est_threads = 0,
        int32_t norm_est_chunk_size = 256
    ) :
        normal_estimation{ norm_est_threads, nor_est_chunk_size }
    {
        this->normal_estimation.setSearchMethod(
            util::wrap_unmanaged(this->neo_search_tree) );
    }
    inline ~TraversibilityGenerator() = default;

public:
    void configure(
        float normal_estimation_radius,
        float interp_grid_res,
        float non_traversible_grad_angle,
        float required_clearance,
        float avoidance_radius,
        int interp_sample_count );

    inline void processMapPoints(
        const TravPointCloud& map_points,
        const Vec3f& map_min_bound,
        const Vec3f& map_max_bound,
        const Vec3f& map_grav_vec,
        const Vec3f& source_pos )
    {
        std::lock_guard lock_{ this->mtx };
        this->trav_points = map_points;
        this->process(
            map_min_bound,
            map_max_bound,
            map_grav_vec,
            source_pos );
    }
    inline void processMapPoints(
        TravPointCloud& map_points,
        const Vec3f& map_min_bound,
        const Vec3f& map_max_bound,
        const Vec3f& map_grav_vec,
        const Vec3f& source_pos )
    {
        std::lock_guard lock_{ this->mtx };
        this->swapTravPoints(map_points);
        this->process(
            map_min_bound,
            map_max_bound,
            map_grav_vec,
            source_pos );
    }

    /* WARNING : no mutex locking occurs, so other threads can be writing to this buffer! */
    inline const TravPointCloud& getTravPoints() const
    {
        return this->trav_points;
    }
    /* WARNING : no mutex locking occurs, so other threads can be writing to this buffer! */
    inline const MetaDataCloud& getTravPointsMeta() const
    {
        return this->trav_points_meta;
    }

    inline TravPointCloud& swapTravPoints(TravPointCloud& swap_cloud)
    {
        this->mtx.lock();
        std::swap(this->trav_points.points, swap_cloud.points);
        std::swap(this->trav_points.height, swap_cloud.height);
        std::swap(this->trav_points.width, swap_cloud.width);
        std::swap(this->trav_points.is_dense, swap_cloud.is_dense);
        this->mtx.unlock();

        return swap_cloud;
    }
    inline MetaDataList& swapMetaDataList(MetaDataList& swap_list)
    {
        this->mtx.lock();
        std::swap(this->trav_points_meta.points, swap_list);
        this->mtx.unlock();

        return swap_list;
    }

protected:
    static inline float& trav_weight(MetaT& m)
    {
        if constexpr(util::traits::has_trav_weight<MetaT>::value)
        {
            return m.trav_weight();
        }
        else
        {
            return m.curvature;
        }
    }
    static inline float trav_weight(const MetaT& m)
    {
        if constexpr(util::traits::has_trav_weight<MetaT>::value)
        {
            return m.trav_weight();
        }
        else
        {
            return m.curvature;
        }
    }

    void process(
        const Vec3f& map_min_bound,
        const Vec3f& map_max_bound,
        const Vec3f& map_grav_vec,
        const Vec3f& source_pos );

protected:
    pcl::search::KdTree<PointT>
        neo_search_tree,
        interp_search_tree,
        avoid_search_tree;
    pcl::octree::OctreePointCloudSearch<PointT>
        cell_search_tree;
    pcl::NormalEstimationOMP<PointT, NormalT>
        normal_estimation;

    util::GridMeta2<> grid_meta;
    std::vector<bool> interp_validity;

    TravPointCloud trav_points;
    MetaDataCloud trav_points_meta;

    pcl::Indices
        trav_selection,     // the set of points that are traversible (after blur)
        avoid_selection,    // the set of points that are not traversible (after blur)
        interp_selection,   // the set of points that should be used for interpolation
        unknown_selection;  // the set of points which couldn't be estimated

    std::mutex mtx;

    float interp_grid_res{ 1. };
    float non_trav_grad_thresh{ 0.70710678118f };   // default is cos(45*)
    float required_clearance{ 0.5f };
    float avoidance_radius{ 0.5f };
    int interp_sample_count{ 7 };

};


template<typename P, typename M>
void TraversibilityGenerator<P, M>::configure(
    float normal_estimation_radius,
    float interp_grid_res,
    float non_traversible_grad_angle,
    float required_clearance,
    float avoidance_radius,
    int interp_sample_count )
{
    this->interp_grid_res = interp_grid_res;
    this->non_trav_grad_thresh =
        std::cos(non_traversible_grad_angle * (M_PI / 180.));   // dot product values LOWER than this value *equiv* gradient angles HIGHER than the source angle
    this->required_clearance = required_clearance;
    this->avoidance_radius = avoidance_radius;
    this->interp_sample_count = interp_sample_count;

    this->normal_estimation.setRadiusSearch(normal_estimation_radius);
}

template<typename P, typename M>
void TraversibilityGenerator<P, M>::process(
    const Vec3f& map_min_bound,
    const Vec3f& map_max_bound,
    const Vec3f& map_grav_vec,
    const Vec3f& source_pos )
{
    #define CLEAR_AND_RESERVE(vec, size) \
        vec.clear(); \
        vec.reserve(size);

    pcl::Indices
        nearest_indices_buff;
    std::vector<float>
        dists_sqrd_buff,
        cached_trav_weights;

// 1. ESTIMATE NORMALS FOR EACH INPUT POINT -----------------------------------
    this->trav_points_meta.clear();
    this->normal_estimation.setInputCloud(
        util::wrap_unmanaged(this->trav_points) );
    this->normal_estimation.compute(this->trav_points_meta);

    CLEAR_AND_RESERVE(this->trav_selection, this->trav_points_meta.size())
    CLEAR_AND_RESERVE(this->avoid_selection, this->trav_points_meta.size())
    CLEAR_AND_RESERVE(this->interp_selection, this->trav_points_meta.size())
    CLEAR_AND_RESERVE(this->unknown_selection, this->trav_points_meta.size())
    cached_trav_weights.resize(this->trav_points_meta.size());

// 2. COMPUTE TRAV WEIGHT, CATEGORIZE UNKNOWN/NON-TRAV POINTS -----------------
    for(size_t i = 0; i < this->trav_points_meta.size(); i++)
    {
        MetaT& m = this->trav_points_meta.points[i];

        if(isnan(m.curvature))  // normal estimation sets normal and curvature to quiet_NaN() on error
        {
            trav_weight(m) = TRAVERSIBILITY_UNKNOWN_WEIGHT;
            this->unknown_selection.push_back(i);
            this->trav_selection.push_back(i);
        }
        else
        {
            // lower = more traversible
            trav_weight(m) =
                (1.f - std::abs(m.getNormalVector3fMap().dot(map_grav_vec)));

            if(trav_weight(m) >= this->non_trav_grad_thresh)
            {
                trav_weight(m) = TRAVERSIBILITY_NON_TRAV_WEIGHT;
                this->avoid_selection.push_back(i);
            }
            else
            {
                this->trav_selection.push_back(i);
                this->interp_selection.push_back(i);
            }
        }

        cached_trav_weights[i] = trav_weight(m);
    }

// 3. APPLY COLLISION RADIUS TO TRAVERSAL POINTS, UPDATE SELECTIONS -----------
    size_t last_trav_selection_i = 0;
    for(size_t i = 0; i < this->trav_selection.size(); i++)
    {
        const auto& pt_idx = this->trav_selection[i];
        auto& src_pt_meta = this->trav_points_meta.points[pt_idx];

        // TODO : clearance handling

        // search for all points within collision radius
        this->neo_search_tree.radiusSearch(
            this->trav_points.points[pt_idx],
            this->avoidance_radius,
            nearest_indices_buff,
            dists_sqrd_buff );

        // set trav weight to the max of all trav weights in range
        bool remove = false;
        for(pcl::index_t nn_idx : nearest_indices_buff)
        {
            const float wt = cached_trav_weights[nn_idx];
            if(wt == TRAVERSIBILITY_NON_TRAV_WEIGHT)
            {
                // if in range of avoidance point, set for removal and break
                trav_weight(src_pt_meta) = wt;
                remove = true;
                break;
            }
            else
            if(wt > trav_weight(src_pt_meta))
            {
                trav_weight(src_pt_meta) = wt;
            }
        }

        if(remove)
        {
            // if need to remove, add to selection and keep last_trav_selection_i the same
            this->avoid_selection.push_back(pt_idx);
        }
        else
        {
            // if keeping this point, copy into last_trav_selection_i if it has fallen behind and iterate
            if(last_trav_selection_i < i)
            {
                this->trav_selection[last_trav_selection_i] = pt_idx;
            }
            last_trav_selection_i++;
        }
    }
    this->trav_selection.resize(last_trav_selection_i);
    // sort avoid_selection indices

// 4. TEST FOR OPEN CELLS AND CALCULATE INTERPOLATED POINTS -------------------
    this->grid_meta.reconfigure(
        map_min_bound.head<2>(),
        map_max_bound.head<2>(),
        this->interp_grid_res );
    this->grid_meta.initBuffer(this->interp_validity, true);

    for(const auto& pt3 : this->trav_points)
    {
        this->grid_meta.getCell(
            this->interp_validity,
            pt3.getVector3fMap().head<2>() ) = false;
    }

}

};
};
