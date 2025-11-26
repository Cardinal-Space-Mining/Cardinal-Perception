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
#include <utility>
#include <type_traits>

#include <Eigen/Core>

#include <pcl/pcl_config.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include "util.hpp"
#include "cloud_ops.hpp"
#include "meta_grid.hpp"
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

private:
    using Vec2f = Eigen::Vector2f;
    using Vec2i = Eigen::Vector2i;
    using Vec3f = Eigen::Vector3f;
    using Vec4f = Eigen::Vector4f;

    using IndicesPair = std::pair<uint32_t, pcl::index_t>;

public:
    using PointT = Point_T;
    using MetaT = Meta_T;
    using NormalT = MetaT;

    using TravPointCloud = pcl::PointCloud<PointT>;
    using MetaDataCloud = pcl::PointCloud<MetaT>;
    using MetaDataList = typename MetaDataCloud::VectorType;

    static constexpr float TRAV_MIN_WEIGHT = 0.f;
    static constexpr float TRAV_MAX_WEIGHT = 1.f;
    static constexpr float EXT_TRAV_MAX_WEIGHT = 10.f;
    static constexpr float NON_TRAV_MIN_WEIGHT = 10.f;
    static constexpr float UNKNOWN_WEIGHT = -1.f;
    static constexpr float FRONTIER_WEIGHT = -2.f;
    static constexpr float NON_TRAV_WEIGHT =
        std::numeric_limits<float>::infinity();

public:
    TraversibilityGenerator() = default;
    ~TraversibilityGenerator() = default;

public:
    void configure(
        float normal_estimation_radius,
        float output_res,
        float grad_search_radius,
        float min_grad_diff,
        float non_traversible_grad_angle,
        float avoidance_radius,
        float trav_score_curvature_weight,
        float trav_score_grad_weight,
        int interp_sample_count);

    void processMapPoints(
        const TravPointCloud& map_points,
        const Vec3f& map_min_bound,
        const Vec3f& map_max_bound,
        const Vec3f& map_grav_vec,
        const Vec3f& source_pos);

public:
    TravPointCloud& copyPoints(TravPointCloud& copy_cloud) const;
    MetaDataList& copyMetaDataList(MetaDataList& copy_list) const;

    /* WARNING : This can only be done once per iteration to extract the
     * processed result! */
    TravPointCloud& swapPoints(TravPointCloud& swap_cloud);
    /* WARNING : This can only be done once per iteration to extract the
     * processed result! */
    MetaDataList& swapMetaDataList(MetaDataList& swap_list);

    void extractTravPoints(TravPointCloud& trav_points) const;
    void extractTravElements(
        TravPointCloud& trav_points,
        MetaDataList& trav_meta_data) const;
    void extractExtTravPoints(TravPointCloud& ext_trav_points) const;
    void extractExtTravElements(
        TravPointCloud& ext_trav_points,
        MetaDataList& ext_trav_meta_data) const;
    void extractNonTravPoints(TravPointCloud& non_trav_points) const;
    void extractNonTravElements(
        TravPointCloud& non_trav_points,
        MetaDataList& non_trav_meta_data) const;

    /* WARNING : This can only be done once per iteration to extract the
     * processed result! */
    pcl::Indices& swapTravIndices(pcl::Indices& swap_indices);
    /* WARNING : This can only be done once per iteration to extract the
     * processed result! */
    pcl::Indices& swapExtTravIndices(pcl::Indices& swap_indices);
    /* WARNING : This can only be done once per iteration to extract the
     * processed result! */
    pcl::Indices& swapNonTravIndices(pcl::Indices& swap_indices);

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
        const TravPointCloud& map_points,
        const Vec3f& map_min_bound,
        const Vec3f& map_max_bound,
        const Vec3f& map_grav_vec,
        const Vec3f& source_pos);

protected:
    pcl::search::KdTree<PointT> interp_search_tree;
    pcl::search::KdTree<PointT> trav_search_tree;
    util::GridMeta<float> vox_grid;
    std::vector<IndicesPair> cell_pt_indices;
    std::vector<int32_t> interp_index_map;

    TravPointCloud points;
    MetaDataCloud points_meta;

    typename TravPointCloud::Ptr points_ptr;

    pcl::Indices
        trav_selection,  // set of points that are traversible (excluding spread)
        ext_trav_selection,  // extended set of points that may be traversible
        avoid_selection;     // set of points that are not traversible ever

    mutable std::mutex mtx;

    float normal_estimation_radius{0.2};
    float output_res{0.1};
    float grad_search_radius{0.25};
    float min_grad_diff{0.15};
    float non_trav_grad_thresh{0.70710678118f};  // default is cos(45*)
    float avoidance_radius{0.5f};
    float avoid_radius_sqrd{0.25f};
    float trav_score_curvature_weight{5.f};
    float trav_score_grad_weight{1.f};
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

#define TRAVERSIBILITY_GEN_INSTANTIATE_PCL_DEPENDENCIES(POINT_TYPE, META_TYPE)
    // template class pcl::NormalEstimationOMP<POINT_TYPE, META_TYPE>;
// clang-format on

#endif
