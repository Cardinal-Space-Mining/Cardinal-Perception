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

#include <vector>
#include <type_traits>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>

#include <traversibility_def.hpp>

#include "path_plan_map.hpp"


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

template<typename Point_T = pcl::PointXYZI>
class PathPlanner
{
    static_assert(
        pcl::traits::has_xyz<Point_T>::value &&
        util::traits::supports_traversibility<Point_T>::value);

public:
    using PointT = Point_T;
    using PointCloudT = pcl::PointCloud<PointT>;
    using PathPlanMapT = PathPlanMap<PointT>;

    using WeightT = traversibility::weight_t<PointT>;

    using Vec3f = Eigen::Vector3f;
    using Box3f = Eigen::AlignedBox3f;

public:
    PathPlanner();
    ~PathPlanner() = default;

    void setParameters(
        float boundary_radius,
        float goal_threshold,
        float search_radius,
        float distance_coeff,
        float straightness_coeff,
        float traversibility_coeff,
        size_t max_neighbors = 10);

    // bool solvePath(
    //     std::vector<Vec3f>& path,
    //     const Vec3f& start,
    //     const Vec3f& goal,
    //     const Vec3f& local_bound_min,
    //     const Vec3f& local_bound_max,
    //     const PointCloudT& trav_points,
    //     const WeightT max_weight =
    //         traversibility::NOMINAL_MAX_WEIGHT<PointT>);

    bool solvePath(
        std::vector<Vec3f>& path,
        const Vec3f& start,
        const Vec3f& goal,
        const PathPlanMapT& map,
        const WeightT max_weight = traversibility::NOMINAL_MAX_WEIGHT<PointT>);

private:
    struct Node
    {
        const PointT& point;
        Vec3f dir;  // previous direction vec
        float g;    // cost from start to this node
        float h;    // heuristic cost to goal
        Node* parent = nullptr;
        pcl::Indices neighbors;

        Node(const PointT& point, float h = 0.f, Node* p = nullptr);
        Node(
            const PointT& point,
            const Vec3f& dir,
            float g = 0.f,
            float h = 0.f,
            Node* p = nullptr);

        // total cost
        inline float f() const { return this->g + this->h; }
        // cost of this node
        inline WeightT cost() const
        {
            return traversibility::weight(this->point);
        }
        inline auto position() const { return this->point.getVector3fMap(); }
    };

private:
    PointCloudT points;
    pcl::search::KdTree<PointT> kdtree;
    std::vector<Node> nodes;  // all nodes in the search space

    // Dist. from search space edge for boundary nodes
    float boundary_radius = 0.15f;
    // threshold for considering goal reached
    float goal_threshold = 0.1f;
    // radius for neighbor search
    float search_radius = 0.5f;
    float verification_range = 1.5f;
    int verification_degree = 2;
    // cost model :
    // distance_coeff * (curr.pos - prev.pos).norm() +
    // straightness_coeff * (1 - prev.dir.dot(curr.pos - prev.pos).normalized()) +
    // traversibility_coeff * trav_weight
    float distance_coeff = 1.f;
    float straightness_coeff = 1.f;
    float traversibility_coeff = 1.f;
    // maximum number of neighbors to consider
    size_t max_neighbors = 10;
};

}  // namespace perception
}  // namespace csm





#ifndef PATH_PLANNER_PRECOMPILED

    #include "impl/path_planner_impl.hpp"

// clang-format off
#define PATH_PLANNER_INSTANTIATE_CLASS_TEMPLATE(POINT_TYPE) \
    template class csm::perception::PathPlanner<POINT_TYPE>;

#define PATH_PLANNER_INSTANTIATE_PCL_DEPENDENCIES(POINT_TYPE) \
    template class pcl::search::KdTree<POINT_TYPE>;
// clang-format on

#endif
