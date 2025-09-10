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

#include <limits>
#include <vector>
#include <type_traits>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>

#include "util.hpp"

#ifndef PATH_PLANNING_PEDANTIC
    #define PATH_PLANNING_PEDANTIC 0
#endif
#ifndef PPLAN_PRINT_DEBUG
    #define PPLAN_PRINT_DEBUG 0
#endif

namespace csm
{
namespace perception
{

template<
    typename Float_T = float,
    typename PointT = pcl::PointXYZ,
    typename MetaPointT = pcl::Normal>
class PathPlanner
{
    static_assert(std::is_floating_point<Float_T>::value);
    static_assert(util::traits::has_trav_weight<MetaPointT>::value);

public:
    using FloatT = Float_T;
    using Point3 = Eigen::Vector<FloatT, 3>;
    using Box3 = Eigen::AlignedBox<FloatT, 3>;
    using LocationCloud = pcl::PointCloud<PointT>;
    using MetaCloud = pcl::PointCloud<MetaPointT>;

private:
    struct Node
    {
        const PointT& trav_point;
        FloatT cost;  // traversal cost of this node only
        FloatT g;     // cost from start to this node
        FloatT h;     // heuristic cost to goal
        Node* parent = nullptr;
        pcl::Indices neighbors;

        inline Node(
            const PointT& point,
            const MetaPointT& meta,
            FloatT h = 0.0f,
            Node* p = nullptr) :
            trav_point(point),
            cost(meta.trav_weight()),
            g(std::numeric_limits<FloatT>::infinity()),
            h(h),
            parent(p)
        {
        }

        inline FloatT f() const { return g + h; }  // total cost
        inline auto position() const { return trav_point.getVector3fMap(); }
    };

public:
    PathPlanner() = default;
    ~PathPlanner() = default;

    bool solvePath(
        const Point3& start,
        const Point3& goal,
        const Point3& local_bound_min,
        const Point3& local_bound_max,
        LocationCloud& loc_cloud,
        MetaCloud& meta_cloud,
        std::vector<Point3>& path);

    inline void setParameters(
        Float_T boundary_radius,
        Float_T goal_threshold,
        Float_T search_radius,
        Float_T lambda_dist,
        Float_T lambda_penalty,
        size_t max_neighbors = 10)
    {
        this->boundary_radius = boundary_radius;
        this->goal_threshold = goal_threshold;
        this->search_radius = search_radius;
        this->lambda_dist = lambda_dist;
        this->lambda_penalty = lambda_penalty;
        this->max_neighbors = max_neighbors;
    }

private:
    pcl::search::KdTree<PointT> kdtree;
    std::vector<Node> nodes;  // all nodes in the search space

    // Dist. from search space edge for boundary nodes
    Float_T boundary_radius = 0.15f;
    // threshold for considering goal reached
    Float_T goal_threshold = 0.1f;
    // radius for neighbor search
    Float_T search_radius = 1.0f;
    // maximum number of neighbors to consider
    size_t max_neighbors = 10;

    // weights for cost model: edge_cost = lambda_d * dist + lambda_p * penalty
    Float_T lambda_dist = static_cast<Float_T>(1.0);
    Float_T lambda_penalty = static_cast<Float_T>(1.0);
};

}  // namespace perception
}  // namespace csm





#ifndef PATH_PLANNER_PRECOMPILED

    #include "impl/path_planner_impl.hpp"

// clang-format off
#define PATH_PLANNER_INSTANTIATE_CLASS_TEMPLATE(        \
    FLOAT_TYPE,                                         \
    POINT_TYPE,                                         \
    META_TYPE)                                          \
    template class csm::perception::                    \
        PathPlanner<FLOAT_TYPE, POINT_TYPE, META_TYPE>;

#define PATH_PLANNER_INSTANTIATE_PCL_DEPENDENCIES(  \
    FLOAT_TYPE,                                     \
    POINT_TYPE,                                     \
    META_TYPE)                                      \
    template class pcl::search::KdTree<POINT_TYPE>;
// clang-format on

#endif
