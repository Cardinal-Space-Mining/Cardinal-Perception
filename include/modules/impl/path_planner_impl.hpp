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

#include "../path_planner.hpp"

#include <queue>
#include <limits>
#include <iostream>
#include <unordered_set>

#include <Eigen/Geometry>

#include <util/std_utils.hpp>
#include <util/d_ary_heap.hpp>

#if PPLAN_PRINT_DEBUG
    #include <iostream>
    #define DEBUG_COUT(...) std::cout << __VA_ARGS__ << std::endl;
#else
    #define DEBUG_COUT(...)
#endif

using namespace csm::perception::traversibility;


namespace csm
{
namespace perception
{

template<typename P>
PathPlanner<P>::Node::Node(const P& point, float h, Node* p) :
    point{point},
    dir{},
    g{std::numeric_limits<float>::infinity()},
    h{h},
    parent{p}
{
}

template<typename P>
PathPlanner<P>::Node::Node(
    const PointT& point,
    const Vec3f& dir,
    float g,
    float h,
    Node* p) :
    point{point},
    dir{dir},
    g{g},
    h{h},
    parent{p}
{
}

template<typename P>
PathPlanner<P>::PathPlanner()
{
    this->kdtree.setSortedResults(true);
}

template<typename P>
void PathPlanner<P>::setParameters(
    float boundary_radius,
    float goal_threshold,
    float search_radius,
    float distance_coeff,
    float straightness_coeff,
    float traversibility_coeff,
    float commit_distance,
    float mid_horizon_distance,
    size_t max_plan_age,
    size_t verification_degree,
    size_t max_neighbors)
{
    this->boundary_radius = boundary_radius;
    this->goal_threshold = goal_threshold;
    this->search_radius = search_radius;
    this->distance_coeff = distance_coeff;
    this->straightness_coeff = straightness_coeff;
    this->traversibility_coeff = traversibility_coeff;
    this->commit_distance = commit_distance;
    this->mid_horizon_distance = mid_horizon_distance;
    this->max_plan_age = max_plan_age;
    this->verification_degree = verification_degree;
    this->max_neighbors = max_neighbors;
}

// template<typename P>
// bool PathPlanner<P>::solvePath(
//     std::vector<Vec3f>& path,
//     const Vec3f& start,
//     const Vec3f& goal,
//     const Vec3f& bound_min,
//     const Vec3f& bound_max,
//     const PointCloudT& points,
//     const WeightT max_weight)
// {
//     return false;
// }

template<typename P>
bool PathPlanner<P>::solvePath(
    std::vector<Vec3f>& path,
    const Vec3f& start,
    const Vec3f& goal,
    const PathPlanMapT& map,
    const WeightT max_weight)
{
    // 0. Init state
    const typename PathPlanMapT::UEOctreeT& ue_space = map.getUESpace();
    const PointCloudT& map_points = map.getPoints();

    pcl::Indices tmp_indices;
    std::vector<float> tmp_dists;
    std::vector<Vec3f> path_prefix;
    PointT extra_pt_buff;
    int start_idx = -1;

    this->nodes.clear();

    // 1. Filter out non-traversible points and build KDTree
    this->points.points.clear();
    this->points.points.reserve(map_points.size());
    for (const PointT& pt : map_points)
    {
        if (isWeighted(pt) && weight(pt) <= max_weight /*|| isFrontier(pt)*/)
        {
            this->points.points.emplace_back(pt);
        }
    }
    this->points.width = this->points.points.size();
    this->points.height = 1;
    this->points.is_dense = true;

    const auto points_ptr = util::wrapUnmanaged(this->points);
    this->kdtree.setInputCloud(points_ptr);

    // 2. Snap goal pt to pointset if inside explored region but outside of
    // discovery range. Since we removed all obstacle points, this changes
    // the goal to be the nearest non-obstacle point in the map as long as it
    // is in the explored region (in the case which the goal was
    // non-traversible).
    PointT goal_pt{goal.x(), goal.y(), goal.z()};
    const bool goal_is_explored = ue_space.isExplored(goal);
    if (goal_is_explored && !this->kdtree.radiusSearch(
                                goal_pt,
                                this->goal_threshold,
                                tmp_indices,
                                tmp_dists,
                                1))
    {
        if (this->kdtree.nearestKSearch(goal_pt, 1, tmp_indices, tmp_dists))
        {
            goal_pt = this->points[tmp_indices[0]];
            if (std::sqrt(tmp_dists[0]) > this->search_radius)
            {
                DEBUG_COUT(
                    "Warning - path planning goal node snapped to available"
                    " node with distance greater than search radius!");
            }
        }
        else
        {
            // no nearest point --> no points at all!
            return false;
        }
    }

    // 3. Attempt to reuse previous path if provided
    if (!path.empty())
    {
        // 3-A. Skip segments already behind the robot
        size_t start_i = 0;
        for (; start_i + 1 < path.size(); start_i++)
        {
            const auto& prev = path[start_i];
            const auto& curr = path[start_i + 1];
            Vec3f diff = curr - prev;
            float proj = (diff.dot(start - prev)) / diff.squaredNorm();
            if (proj < 1.f)
                break;
        }

        // 3-B. Walk all zones:
        //   Zone 0 [0, commit_distance)       : locked, no validation
        //   Zone 1 [commit_distance, mid_horizon_distance) : validated with KNN
        //   Zone 2 [mid_horizon_distance, end) : lazy, goal tracking only
        float path_len = 0.f;
        size_t commit_end_i = path.size();  // first index in Zone 1
        size_t mid_end_i = path.size();     // first index in Zone 2
        bool zone1_valid = true;
        float min_goal_dist = std::numeric_limits<float>::infinity();
        size_t goal_trim_i = start_i;
        pcl::Indices prev_nearest;
        size_t prev_i = start_i;

        for (size_t i = start_i; i < path.size(); i++)
        {
            const Vec3f& pt = path[i];

            if (i > start_i)
                path_len += (pt - path[prev_i]).norm();
            prev_i = i;

            const float d_to_goal = (goal_pt.getVector3fMap() - pt).norm();
            if (d_to_goal < min_goal_dist)
            {
                goal_trim_i = i;
                min_goal_dist = d_to_goal;
            }

            if (commit_end_i == path.size() && path_len >= this->commit_distance)
                commit_end_i = i;

            if (path_len < this->commit_distance)
                continue;

            if (path_len >= this->mid_horizon_distance)
            {
                if (mid_end_i == path.size())
                    mid_end_i = i;
                continue;
            }

            if (!zone1_valid)
                continue;

            tmp_indices.clear();
            if (!this->kdtree.nearestKSearch(
                    PointT{pt.x(), pt.y(), pt.z()},
                    static_cast<int>(this->verification_degree),
                    tmp_indices,
                    tmp_dists))
            {
                zone1_valid = false;
                mid_end_i = i;
                continue;
            }

            bool valid = true;
            for (size_t j = 0; j < tmp_indices.size() && valid; j++)
            {
                if (std::sqrt(tmp_dists[j]) > this->search_radius)
                {
                    valid = false;
                    break;
                }
                const auto& pt_a = this->points[tmp_indices[j]];
                for (const pcl::index_t k : prev_nearest)
                {
                    if ((pt_a.getVector3fMap() -
                         this->points[k].getVector3fMap()).norm() >
                        this->search_radius)
                    {
                        valid = false;
                        break;
                    }
                }
            }

            if (!valid)
            {
                zone1_valid = false;
                mid_end_i = i;
                continue;
            }

            prev_nearest.swap(tmp_indices);
        }

        // 3-C. Decide replan strategy
        const bool goal_reached =
            goal_is_explored
                ? min_goal_dist <= this->goal_threshold
                : ue_space.distToUnexplored(path.back(), this->boundary_radius) <=
                      this->boundary_radius;

        const bool timer_expired = (this->plan_age >= this->max_plan_age);

        if (!timer_expired && zone1_valid && goal_reached)
        {
            // Lazy path: everything looks good, increment staleness counter
            if (min_goal_dist <= this->goal_threshold)
                path.erase(path.begin() + goal_trim_i + 1, path.end());
            path.erase(path.begin(), path.begin() + start_i);
            this->plan_age++;
            return true;
        }

        // A* needed: determine where to replan from
        // Force/zone1-invalid → replan from Zone 0 boundary
        // Zone1 valid, zone2 stale/goal-not-reached → replan from Zone 1 boundary
        size_t astar_start_i =
            (timer_expired || !zone1_valid) ? commit_end_i : mid_end_i;

        if (astar_start_i >= path.size())
            astar_start_i = path.size() - 1;

        if (astar_start_i > start_i)
            path_prefix.assign(
                path.begin() + start_i, path.begin() + astar_start_i);

        const Vec3f astar_start_pos = path[astar_start_i];
        extra_pt_buff.getVector3fMap() = astar_start_pos;
        const Vec3f dir =
            astar_start_i > 0
                ? (astar_start_pos - path[astar_start_i - 1]).normalized()
                : Vec3f::Zero();
        this->nodes.emplace_back(
            extra_pt_buff,
            dir,
            0.f,
            this->distance_coeff *
                (goal_pt.getVector3fMap() - astar_start_pos).norm());
        start_idx = 0;
    }

    // 4. Construct nodes
    const size_t num_prefix_nodes = this->nodes.size();
    this->nodes.reserve(num_prefix_nodes + this->points.size());
    for (const auto& pt : this->points)
    {
        // compute h as proportional to straight-line goal distance
        this->nodes.emplace_back(
            pt,
            this->distance_coeff *
                (goal_pt.getVector3fMap() - pt.getVector3fMap()).norm());
    }

    // 5. Init start node if not already done (snap to pointset)
    if (start_idx < 0)
    {
        if (this->kdtree.nearestKSearch(
                PointT{start.x(), start.y(), start.z()},
                1,
                tmp_indices,
                tmp_dists))
        {
            start_idx = static_cast<int>(num_prefix_nodes) + tmp_indices[0];
            this->nodes[start_idx].g = 0.f;
            this->nodes[start_idx].dir.setZero();
        }
        else
        {
            DEBUG_COUT(
                "Error - could not snap start node to available points!");
            return false;
        }
    }

    // 6. Setup
    util::DAryHeap<float, int> open(static_cast<int>(this->nodes.size()));
    open.reserve_heap(this->nodes.size());
    open.push(start_idx, this->nodes[start_idx].f());

    std::vector<bool> closed(this->nodes.size(), false);

    int closest_idx = -1;
    float closest_dist = std::numeric_limits<float>::infinity();

    // 7. A* body
    while (!open.empty())
    {
        const int idx = open.pop();
        Node& current = this->nodes[idx];

        if (closed[idx])
        {
            continue;
        }
        else
        {
            closed[idx] = true;
        }

        // if ((current.position() - goal_pt.getVector3fMap()).norm() <
        //     this->goal_threshold)
        if (current.h < this->goal_threshold * this->distance_coeff)
        {
            closest_idx = idx;
            break;
        }

        if (current.neighbors.empty())
        {
            tmp_dists.clear();
            this->kdtree.radiusSearch(
                current.point,
                this->search_radius,
                current.neighbors,
                tmp_dists,
                this->max_neighbors);
        }

        for (const pcl::index_t nb_idx_ : current.neighbors)
        {
            const size_t nb_idx =
                num_prefix_nodes + static_cast<size_t>(nb_idx_);
            Node& nb = this->nodes[nb_idx];

            if (closed[nb_idx])
            {
                continue;
            }

            const Vec3f diff = (nb.position() - current.position());
            const float dist = diff.norm();
            const float inv_dot = (1.f - diff.dot(current.dir) / dist);
            const float edge_cost = (this->distance_coeff * dist) +
                                    (this->straightness_coeff * inv_dot) +
                                    (this->traversibility_coeff * nb.cost());
            const float tentative_g = current.g + edge_cost;

            if (tentative_g < nb.g)
            {
                nb.dir = (diff / dist);
                nb.g = tentative_g;
                nb.parent = &current;

                if (!open.contains(nb_idx))
                {
                    open.push(nb_idx, nb.f());
                }
                else
                {
                    open.decrease_key(nb_idx, nb.f());
                }

                const bool is_frontier =
                    ue_space.distToUnexplored(
                        nb.position(),
                        this->boundary_radius) <= this->boundary_radius;

                // We already snapped the goal to the nearest point in the set
                // as long as it was in explored range, thus if the goal is
                // reachable, it will trigger the exit condition above
                // (proximity). This implies that we should only track the
                // closest frontier node - either the goal was outside explored
                // range or there is an obstacle blocking it; either way we need
                // to keep exploring (ie. traverse to a frontier point). If we
                // can't reach any frontier nodes [and goal is not reached],
                // then the function should return false, meaning there is no
                // meaningful path to follow. Thus, the return state is only
                // correct if we limit closest_idx to being a frontier node here!
                if (is_frontier && nb.h < closest_dist)
                {
                    closest_idx = nb_idx;
                    closest_dist = nb.h;
                }
            }
        }
    }

    // 8. Export
    if (closest_idx >= 0)
    {
        path.clear();
        for (Node* n = &this->nodes[closest_idx]; n != nullptr; n = n->parent)
        {
            path.push_back(n->position());
        }
        if (!path_prefix.empty())
        {
            path.insert(path.end(), path_prefix.rbegin(), path_prefix.rend());
        }
        std::reverse(path.begin(), path.end());

        this->plan_age = 0;
        return true;
    }

    return false;
}

// template<typename P>
// bool PathPlanner<P>::solvePath(
//     const Vec3f& start,
//     const Vec3f& goal,
//     const Vec3f& local_bound_min,
//     const Vec3f& local_bound_max,
//     const PointCloudT& trav_points,
//     std::vector<Vec3f>& path)
// {
// #if PATH_PLANNING_PEDANTIC
//     if (start.x() < local_bound_min.x() || start.y() < local_bound_min.y() ||
//         start.x() > local_bound_max.x() || start.y() > local_bound_max.y())
//     {
//         throw std::out_of_range("Start point is out of bounds");
//     }
//     if (trav_points.points.empty())
//     {
//         throw std::invalid_argument("Input point cloud are empty");
//     }
// #endif

//     // temp buffers reused by radiusSearch
//     pcl::Indices tmp_indices;
//     std::vector<float> tmp_dists;

//     auto shared_trav_points = util::wrapUnmanaged(trav_points);
//     this->kdtree.setInputCloud(shared_trav_points);

//     PointT goal_pt;
//     goal_pt.getVector3fMap() = goal;
//     // handle goal point isn't reachable given goal thresh
//     if ((goal.array() >= local_bound_min.array()).all() &&
//         (goal.array() < local_bound_max.array()).all())
//     {
//         if (!this->kdtree.radiusSearch(
//                 goal_pt,
//                 this->goal_threshold,
//                 tmp_indices,
//                 tmp_dists,
//                 1))
//         {
//             if (this->kdtree.nearestKSearch(goal_pt, 1, tmp_indices, tmp_dists))
//             {
//                 goal_pt = trav_points.points[tmp_indices[0]];
//                 if (std::sqrt(tmp_dists[0]) > this->search_radius)
//                 {
//                     DEBUG_COUT(
//                         "Warning - path planning goal node snapped to available"
//                         " node with distance greater than search radius!");
//                 }
//             }
//             else
//             {
//                 return false;
//             }
//         }
//     }

//     // check for unreachable destination or move goal to closest traversible point
//     this->kdtree
//         .radiusSearch(goal_pt, this->search_radius, tmp_indices, tmp_dists);
//     bool all_invalid = true;
//     for (pcl::index_t i : tmp_indices)
//     {
//         if (isNominal(trav_points.points[i]))
//         {
//             goal_pt = trav_points.points[i];
//             all_invalid = false;
//             break;
//         }
//     }
//     if (all_invalid)
//     {
//         DEBUG_COUT("Error - goal is unreachable!");
//         return false;
//     }

//     this->nodes.clear();
//     this->nodes.reserve(trav_points.points.size());

//     // Heuristic = lambda_d * Euclidean distance
//     for (size_t i = 0; i < trav_points.points.size(); ++i)
//     {
//         const auto& point = trav_points.points[i];
//         float h = distance_coeff *
//                   (goal_pt.getVector3fMap() - point.getVector3fMap()).norm();
//         this->nodes.emplace_back(point, h);
//     }

//     // find start node
//     int start_i;
//     if (this->kdtree.nearestKSearch(
//             PointT{start.x(), start.y(), start.z()},
//             1,
//             tmp_indices,
//             tmp_dists))
//     {
//         start_i = tmp_indices[0];
//         this->nodes[start_i].g = 0.f;
//     }
//     else
//     {
//         DEBUG_COUT("Error - could not initialize starting location!");
//         return false;
//     }

//     // create open heap over f = g + h
//     util::DAryHeap<float, int> open(static_cast<int>(this->nodes.size()));
//     open.reserve_heap(this->nodes.size());
//     open.push(start_i, this->nodes[start_i].f());

//     // closed set
//     std::vector<bool> closed(this->nodes.size(), false);

//     // keep track of closest visited node to goal
//     int closest_idx = start_i;
//     // h proportional to distance to goal
//     float closest_dist = this->nodes[start_i].h;
//     bool found_boundary = false;

//     const Box3f outside_boundary{
//         local_bound_min + Vec3f::Constant(boundary_radius),
//         local_bound_max - Vec3f::Constant(boundary_radius)};

//     auto create_path = [&](Node& path_end)
//     {
//         path.clear();
//         for (Node* n = &path_end; n != nullptr; n = n->parent)
//         {
//             path.push_back(n->position());
//         }
//         std::reverse(path.begin(), path.end());
//     };

//     while (!open.empty())
//     {
//         const int idx = open.pop();
//         Node& current = this->nodes[idx];

//         if (closed[idx])
//         {
//             continue;  // skip if already closed
//         }
//         closed[idx] = true;

//         // Goal check (by geometry)
//         if ((current.position() - goal_pt.getVector3fMap()).norm() <
//             goal_threshold)
//         {
//             create_path(current);
//             return true;
//         }

//         // Lazily compute neighbors on first expansion
//         if (current.neighbors.empty())
//         {
//             current.neighbors.clear();
//             tmp_dists.clear();
//             this->kdtree.radiusSearch(
//                 current.point,
//                 search_radius,
//                 current.neighbors,
//                 tmp_dists,
//                 max_neighbors);
//         }

//         // Relax neighbors
//         for (const int nb_idx : current.neighbors)
//         {
//             if (closed[nb_idx])
//             {
//                 continue;
//             }
//             else if (this->nodes[nb_idx].cost() > 1.f)
//             {
//                 closed[nb_idx] = true;
//                 continue;
//             }
//             Node& nb = this->nodes[nb_idx];

//             // geometric edge length
//             const float geom = (nb.position() - current.position()).norm();

//             const float edge = distance_coeff * geom + traversibility_coeff * nb.cost();

//             const float tentative_g = current.g + edge;
//             if (tentative_g < nb.g)
//             {
//                 nb.g = tentative_g;
//                 nb.parent = &current;

//                 bool in_boundary = !outside_boundary.contains(nb.position());

//                 if (!found_boundary && in_boundary)
//                 {
//                     closest_dist = nb.h;
//                     closest_idx = nb_idx;
//                     found_boundary = true;
//                 }
//                 else if (nb.h < closest_dist)
//                 {
//                     if (!found_boundary || in_boundary)
//                     {
//                         closest_dist = nb.h;
//                         closest_idx = nb_idx;
//                     }
//                 }

//                 const float new_f = nb.f();
//                 if (!open.contains(nb_idx))
//                 {
//                     open.push(nb_idx, new_f);
//                 }
//                 else
//                 {
//                     open.decrease_key(nb_idx, new_f);  // strictly better
//                 }
//             }
//         }
//     }

//     // No path to goal: pick closest visited node to the goal
//     if (closest_idx >= 0)
//     {
//         create_path(this->nodes[closest_idx]);

//         if (!found_boundary)
//         {
//             DEBUG_COUT(
//                 "No boundary nodes reached, returning closest node to goal");
//         }
//         return found_boundary;
//     }

//     // Nothing reachable at all
//     DEBUG_COUT("No reachable nodes; start may be isolated");
//     return false;
// }

};  // namespace perception
};  // namespace csm

#undef DEBUG_COUT
