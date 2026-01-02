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
    trav_point(point),
    cost(weight(point)),
    g(std::numeric_limits<float>::infinity()),
    h(h),
    parent(p)
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
    float lambda_dist,
    float lambda_penalty,
    size_t max_neighbors)
{
    this->boundary_radius = boundary_radius;
    this->goal_threshold = goal_threshold;
    this->search_radius = search_radius;
    this->lambda_dist = lambda_dist;
    this->lambda_penalty = lambda_penalty;
    this->max_neighbors = max_neighbors;
}

template<typename P>
bool PathPlanner<P>::solvePath(
    const Vec3f& start,
    const Vec3f& goal,
    const Vec3f& local_bound_min,
    const Vec3f& local_bound_max,
    const PointCloudT& trav_points,
    std::vector<Vec3f>& path)
{
#if PATH_PLANNING_PEDANTIC
    if (start.x() < local_bound_min.x() || start.y() < local_bound_min.y() ||
        start.x() > local_bound_max.x() || start.y() > local_bound_max.y())
    {
        throw std::out_of_range("Start point is out of bounds");
    }
    if (trav_points.points.empty())
    {
        throw std::invalid_argument("Input point cloud are empty");
    }
#endif

    // temp buffers reused by radiusSearch
    pcl::Indices tmp_indices;
    std::vector<float> tmp_dists;

    auto shared_trav_points = util::wrapUnmanaged(trav_points);
    this->kdtree.setInputCloud(shared_trav_points);

    PointT goal_pt;
    goal_pt.getVector3fMap() = goal;
    // handle goal point isn't reachable given goal thresh
    if ((goal.array() >= local_bound_min.array()).all() &&
        (goal.array() < local_bound_max.array()).all())
    {
        if (!this->kdtree.radiusSearch(
                goal_pt,
                this->goal_threshold,
                tmp_indices,
                tmp_dists,
                1))
        {
            if (this->kdtree.nearestKSearch(goal_pt, 1, tmp_indices, tmp_dists))
            {
                goal_pt = trav_points.points[tmp_indices[0]];
                if (std::sqrt(tmp_dists[0]) > this->search_radius)
                {
                    DEBUG_COUT(
                        "Warning - path planning goal node snapped to available"
                        " node with distance greater than search radius!");
                }
            }
            else
            {
                return false;
            }
        }
    }

    // check for unreachable destination or move goal to closest traversible point
    this->kdtree.radiusSearch(goal_pt, this->search_radius, tmp_indices, tmp_dists);
    bool all_invalid = true;
    for(pcl::index_t i : tmp_indices)
    {
        if(isNominal(trav_points.points[i]))
        {
            goal_pt = trav_points.points[i];
            all_invalid = false;
            break;
        }
    }
    if(all_invalid)
    {
        DEBUG_COUT("Error - goal is unreachable!");
        return false;
    }

    this->nodes.clear();
    this->nodes.reserve(trav_points.points.size());

    // Heuristic = lambda_d * Euclidean distance
    for (size_t i = 0; i < trav_points.points.size(); ++i)
    {
        const auto& point = trav_points.points[i];
        float h = lambda_dist *
                  (goal_pt.getVector3fMap() - point.getVector3fMap()).norm();
        this->nodes.emplace_back(point, h);
    }

    // find start node
    int start_idx;
    if (this->kdtree.nearestKSearch(
            PointT{start.x(), start.y(), start.z()},
            1,
            tmp_indices,
            tmp_dists))
    {
        start_idx = tmp_indices[0];
        this->nodes[start_idx].g = 0.f;
    }
    else
    {
        DEBUG_COUT("Error - could not initialize starting location!");
        return false;
    }

    // create open heap over f = g + h
    util::DAryHeap<float, int> open(static_cast<int>(this->nodes.size()));
    open.reserve_heap(this->nodes.size());
    open.push(start_idx, this->nodes[start_idx].f());

    // closed set
    std::vector<bool> closed(this->nodes.size(), false);

    // keep track of closest visited node to goal
    int closest_idx = start_idx;
    // h proportional to distance to goal
    float closest_dist = this->nodes[start_idx].h;
    bool found_boundary = false;

    const Box3f outside_boundary{
        local_bound_min + Vec3f::Constant(boundary_radius),
        local_bound_max - Vec3f::Constant(boundary_radius)};

    auto create_path = [&](Node& path_end)
    {
        path.clear();
        for (Node* n = &path_end; n != nullptr; n = n->parent)
        {
            path.push_back(n->position());
        }
        std::reverse(path.begin(), path.end());
    };

    while (!open.empty())
    {
        const int idx = open.pop();
        Node& current = this->nodes[idx];

        if (closed[idx])
        {
            continue;  // skip if already closed
        }
        closed[idx] = true;

        // Goal check (by geometry)
        if ((current.position() - goal_pt.getVector3fMap()).norm() <
            goal_threshold)
        {
            create_path(current);
            return true;
        }

        // Lazily compute neighbors on first expansion
        if (current.neighbors.empty())
        {
            current.neighbors.clear();
            tmp_dists.clear();
            this->kdtree.radiusSearch(
                current.trav_point,
                search_radius,
                current.neighbors,
                tmp_dists,
                max_neighbors);
        }

        // Relax neighbors
        for (const int nb_idx : current.neighbors)
        {
            if (closed[nb_idx])
            {
                continue;
            }
            else if (this->nodes[nb_idx].cost > 1.f)
            {
                closed[nb_idx] = true;
                continue;
            }
            Node& nb = this->nodes[nb_idx];

            // geometric edge length
            const float geom = (nb.position() - current.position()).norm();

            const float edge = lambda_dist * geom + lambda_penalty * nb.cost;

            const float tentative_g = current.g + edge;
            if (tentative_g < nb.g)
            {
                nb.g = tentative_g;
                nb.parent = &current;

                bool in_boundary = !outside_boundary.contains(nb.position());

                if (!found_boundary && in_boundary)
                {
                    closest_dist = nb.h;
                    closest_idx = nb_idx;
                    found_boundary = true;
                }
                else if (nb.h < closest_dist)
                {
                    if (!found_boundary || in_boundary)
                    {
                        closest_dist = nb.h;
                        closest_idx = nb_idx;
                    }
                }

                const float new_f = nb.f();
                if (!open.contains(nb_idx))
                {
                    open.push(nb_idx, new_f);
                }
                else
                {
                    open.decrease_key(nb_idx, new_f);  // strictly better
                }
            }
        }
    }

    // No path to goal: pick closest visited node to the goal
    if (closest_idx >= 0)
    {
        create_path(this->nodes[closest_idx]);

        if (!found_boundary)
        {
            DEBUG_COUT(
                "No boundary nodes reached, returning closest node to goal");
        }
        return found_boundary;
    }

    // Nothing reachable at all
    DEBUG_COUT("No reachable nodes; start may be isolated");
    return false;
}

};  // namespace perception
};  // namespace csm

#undef DEBUG_COUT
