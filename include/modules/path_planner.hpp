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

#include <vector>
#include <type_traits>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>

#include <point_def.hpp>


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
    typename Point_T = pcl::PointXYZ,
    typename MetaPoint_T = csm::perception::NormalTraversal>
class PathPlanner
{
    static_assert(util::traits::has_trav_weight<MetaPoint_T>::value);

public:
    using PointT = Point_T;
    using MetaPointT = MetaPoint_T;
    using PointCloudT = pcl::PointCloud<PointT>;
    using MetaCloudT = pcl::PointCloud<MetaPointT>;

    using Vec3f = Eigen::Vector3f;
    using Box3f = Eigen::AlignedBox3f;

private:
    struct Node
    {
        const PointT& trav_point;
        float cost;  // traversal cost of this node only
        float g;     // cost from start to this node
        float h;     // heuristic cost to goal
        Node* parent = nullptr;
        pcl::Indices neighbors;

        Node(
            const PointT& point,
            const MetaPointT& meta,
            float h = 0.0f,
            Node* p = nullptr);

        inline float f() const { return g + h; }  // total cost
        inline auto position() const { return trav_point.getVector3fMap(); }
    };

public:
    PathPlanner();
    ~PathPlanner() = default;

    void setParameters(
        float boundary_radius,
        float goal_threshold,
        float search_radius,
        float lambda_dist,
        float lambda_penalty,
        size_t max_neighbors = 10);

    bool solvePath(
        const Vec3f& start,
        const Vec3f& goal,
        const Vec3f& local_bound_min,
        const Vec3f& local_bound_max,
        const PointCloudT& loc_cloud,
        const MetaCloudT& meta_cloud,
        std::vector<Vec3f>& path);

private:
    pcl::search::KdTree<PointT> kdtree;
    std::vector<Node> nodes;  // all nodes in the search space

    // Dist. from search space edge for boundary nodes
    float boundary_radius = 0.15f;
    // threshold for considering goal reached
    float goal_threshold = 0.1f;
    // radius for neighbor search
    float search_radius = 1.0f;
    // weights for cost model: edge_cost = lambda_d * dist + lambda_p * penalty
    float lambda_dist = 1.f;
    float lambda_penalty = 1.f;
    // maximum number of neighbors to consider
    size_t max_neighbors = 10;
};

}  // namespace perception
}  // namespace csm





#ifndef PATH_PLANNER_PRECOMPILED

    #include "impl/path_planner_impl.hpp"

// clang-format off
#define PATH_PLANNER_INSTANTIATE_CLASS_TEMPLATE(        \
    POINT_TYPE,                                         \
    META_TYPE)                                          \
    template class csm::perception::                    \
        PathPlanner<POINT_TYPE, META_TYPE>;

#define PATH_PLANNER_INSTANTIATE_PCL_DEPENDENCIES(  \
    POINT_TYPE,                                     \
    META_TYPE)                                      \
    template class pcl::search::KdTree<POINT_TYPE>;
// clang-format on

#endif
