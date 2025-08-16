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

#include <pcl/pcl_config.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>

#include "util.hpp"
#include "cloud_ops.hpp"
#include "point_def.hpp"


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
        pcl::traits::has_curvature<Meta_T>::value);

public:
    using PointT = Point_T;
    using MetaT = Meta_T;
    using NormalT = MetaT;
    using Vec3f = Eigen::Vector3f;

    using TravPointCloud = pcl::PointCloud<PointT>;
    using MetaDataCloud = pcl::PointCloud<MetaT>;
    using MetaDataList = typename MetaDataCloud::VectorType;

    static constexpr float UNKNOWN_WEIGHT = -1.f;
    static constexpr float NON_TRAV_WEIGHT =
        std::numeric_limits<float>::infinity();

public:
    inline TraversibilityGenerator(
        uint32_t norm_est_threads = 0,
        int32_t norm_est_chunk_size = 256) :
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
    inline ~TraversibilityGenerator() = default;

public:
    void configure(
        float normal_estimation_radius,
        float interp_grid_res,
        float non_traversible_grad_angle,
        float required_clearance,
        float avoidance_radius,
        int interp_sample_count);

    /* This overload copies the point buffer since it takes a const reference */
    inline void processMapPoints(
        const TravPointCloud& map_points,
        const Vec3f& map_min_bound,
        const Vec3f& map_max_bound,
        const Vec3f& map_grav_vec,
        const Vec3f& source_pos)
    {
        std::lock_guard lock_{this->mtx};
        this->trav_points = map_points;
        this->process(map_min_bound, map_max_bound, map_grav_vec, source_pos);
    }
    /* This overload swaps the point buffer since it takes a non-const
     * reference. Explicitly casting to const reference will apply the other
     * overload if this behavior is undesired. */
    inline void processMapPoints(
        TravPointCloud& map_points,
        const Vec3f& map_min_bound,
        const Vec3f& map_max_bound,
        const Vec3f& map_grav_vec,
        const Vec3f& source_pos)
    {
        this->swapPoints(map_points);
        std::lock_guard lock_{this->mtx};
        this->process(map_min_bound, map_max_bound, map_grav_vec, source_pos);
    }

public:
    inline TravPointCloud& copyPoints(TravPointCloud& copy_cloud) const
    {
        this->mtx.lock();
        copy_cloud = this->trav_points;
        this->mtx.unlock();

        return copy_cloud;
    }
    inline MetaDataList& copyMetaDataList(MetaDataList& copy_list) const
    {
        this->mtx.lock();
        copy_list = this->trav_points_meta.points;
        this->mtx.unlock();

        return copy_list;
    }

    /* WARNING : This can only be done once per iteration to extract the
     * processed result! */
    inline TravPointCloud& swapPoints(TravPointCloud& swap_cloud)
    {
        this->mtx.lock();
        std::swap(this->trav_points.points, swap_cloud.points);
        std::swap(this->trav_points.height, swap_cloud.height);
        std::swap(this->trav_points.width, swap_cloud.width);
        std::swap(this->trav_points.is_dense, swap_cloud.is_dense);
        this->mtx.unlock();

        return swap_cloud;
    }
    /* WARNING : This can only be done once per iteration to extract the
     * processed result! */
    inline MetaDataList& swapMetaDataList(MetaDataList& swap_list)
    {
        this->mtx.lock();
        std::swap(this->trav_points_meta.points, swap_list);
        this->mtx.unlock();

        return swap_list;
    }

    inline void extractTravElements(
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
    inline void extractNonTravElements(
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

    /* WARNING : This can only be done once per iteration to extract the
     * processed result! */
    inline pcl::Indices& swapTravIndices(pcl::Indices& swap_indices)
    {
        this->mtx.lock();
        std::swap(this->trav_selection, swap_indices);
        this->mtx.unlock();

        return swap_indices;
    }
    /* WARNING : This can only be done once per iteration to extract the
     * processed result! */
    inline pcl::Indices& swapNonTravIndices(pcl::Indices& swap_indices)
    {
        this->mtx.lock();
        std::swap(this->avoid_selection, swap_indices);
        this->mtx.unlock();

        return swap_indices;
    }

protected:
    static inline float& trav_weight(MetaT& m)
    {
        if constexpr (util::traits::has_trav_weight<MetaT>::value)
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
        if constexpr (util::traits::has_trav_weight<MetaT>::value)
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
        const Vec3f& source_pos);

protected:
    pcl::search::KdTree<PointT> neo_search_tree, interp_search_tree;
    pcl::NormalEstimationOMP<PointT, NormalT> normal_estimation;

    std::vector<int32_t> interp_index_map;

    TravPointCloud trav_points;
    MetaDataCloud trav_points_meta;

    pcl::Indices
        trav_selection,   // set of points that are traversible (after blur)
        avoid_selection,  // set of points that are not traversible (after blur)
        interp_selection;  // set of points that should be used for interpolation
    // unknown_selection;// set of points which couldn't be estimated

    mutable std::mutex mtx;

    float interp_grid_res{1.};
    float non_trav_grad_thresh{0.70710678118f};  // default is cos(45*)
    float required_clearance{0.5f};
    float avoidance_radius{0.5f};
    int interp_sample_count{7};
    //
};

};  // namespace perception
};  // namespace csm




#ifndef TRAVERSIBILITY_GEN_PRECOMPILED

    #include "impl/traversibility_gen_impl.hpp"

// clang-format off
#define TRAVERSIBILITY_GEN_INSTANTIATE_CLASS_TEMPLATE(POINT_TYPE, META_TYPE) \
    template class csm::perception::                    \
        TraversibilityGenerator<POINT_TYPE, META_TYPE>;

#define TRAVERSIBILITY_GEN_INSTANTIATE_PCL_DEPENDENCIES(POINT_TYPE, META_TYPE) \
    template class pcl::NormalEstimationOMP<POINT_TYPE, META_TYPE>;
// clang-format on

#endif
