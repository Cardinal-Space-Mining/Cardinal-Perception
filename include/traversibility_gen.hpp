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
#include <vector>
#include <type_traits>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/octree/octree_search.h>
#include <pcl/features/normal_3d_omp.h>

#include "point_def.hpp"


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

    using PointCloudType = pcl::PointCloud<PointT>;
    using MetaCloudType = pcl::PointCloud<MetaT>;

public:
    inline TraversibilityGenerator() = default;
    inline ~TraversibilityGenerator() = default;

public:
    void configure(
        float interp_grid_res,
        float non_traversible_grad_angle,
        float avoidance_radius,
        int interp_sample_count );
    void processMapPoints(
        const PointCloudType& map_points,
        const Vec3f& map_min_bound,
        const Vec3f& map_max_bound,
        const Vec3f& map_grav_vec,
        const Vec3f& source_pos );

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

protected:
    pcl::search::KdTree<PointT>
        neo_search_tree,
        interp_search_tree,
        avoid_search_tree;
    pcl::octree::OctreePointCloudSearch<PointT>
        cell_search_tree;
    pcl::NormalEstimationOMP<PointT, NormalT>
        normal_estimation;

    float interp_grid_res{ 1. };
    float non_trav_grad_thresh{ 0.70710678118f };
    float avoidance_radius{ 0.5f };
    int interp_sample_count{ 7 };

};


template<typename P, typename M>
void TraversibilityGenerator<P, M>::configure(
    float interp_grid_res,
    float non_traversible_grad_angle,
    float avoidance_radius,
    int interp_sample_count )
{
    this->interp_grid_res = interp_grid_res;
    this->non_trav_grad_thresh =
        std::cos(non_traversible_grad_angle * (M_PI / 180.));   // dot product values LOWER than this value *equiv* gradient angles HIGHER than the source angle
    this->avoidance_radius = avoidance_radius;
    this->interp_sample_count = interp_sample_count;
}

template<typename P, typename M>
void TraversibilityGenerator<P, M>::processMapPoints(
    const PointCloudType& map_points,
    const Vec3f& map_min_bound,
    const Vec3f& map_max_bound,
    const Vec3f& map_grav_vec,
    const Vec3f& source_pos )
{
    
}
