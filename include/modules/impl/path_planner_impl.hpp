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

#include "../path_planner.hpp"

#include <queue>
#include <iostream>
#include <unordered_set>

#include <Eigen/Geometry>

#include "d_ary_heap.hpp"

#if PPLAN_PRINT_DEBUG
    #include <iostream>
    #define DEBUG_COUT(...) std::cout << __VA_ARGS__ << std::endl;
#else
    #define DEBUG_COUT(...)
#endif

namespace csm
{
namespace perception
{

template<typename Float_T, typename PointT, typename MetaPointT>
bool PathPlanner<Float_T, PointT, MetaPointT>::solvePath(
    const Point3& start,
    const Point3& goal,
    const Point3& local_bound_min,
    const Point3& local_bound_max,
    LocationCloud& loc_cloud,
    MetaCloud& meta_cloud,
    std::vector<Point3>& path)
{
#if PATH_PLANNING_PEDANTIC
    if (start.x() < local_bound_min.x() || start.y() < local_bound_min.y() ||
        start.x() > local_bound_max.x() || start.y() > local_bound_max.y())
    {
        throw std::out_of_range("Start point is out of bounds");
    }
    if (loc_cloud.points.size() != meta_cloud.points.size())
    {
        throw std::invalid_argument(
            "Location and meta clouds must have the same size");
    }
    if (loc_cloud.points.empty() || meta_cloud.points.empty())
    {
        throw std::invalid_argument("Point clouds are empty");
    }
#endif

    auto shared_loc_cloud = util::wrap_unmanaged(loc_cloud);
    kdtree.setInputCloud(shared_loc_cloud);

    nodes.clear();
    nodes.reserve(loc_cloud.points.size());

    // Heuristic = lambda_d * Euclidean distance
    for (size_t i = 0; i < loc_cloud.points.size(); ++i)
    {
        const auto& point = loc_cloud.points[i];
        const auto& meta = meta_cloud.points[i];
        float_t h = lambda_dist * (goal - point.getVector3fMap()).norm();
        nodes.emplace_back(point, meta, h);
    }

    // find start node
    pcl::Indices kdtree_indices;
    std::vector<float_t> kdtree_distances;
    kdtree.nearestKSearch(
        PointT(start.x(), start.y(), start.z()),
        1,
        kdtree_indices,
        kdtree_distances);
    const int start_idx = kdtree_indices[0];
    Node& start_node = nodes[start_idx];
    start_node.g = static_cast<float_t>(0);

    // create open heap over f = g + h
    DaryHeap<float_t, int> open(static_cast<int>(nodes.size()));
    open.reserve_heap(nodes.size());
    open.push(start_idx, start_node.f());

    // closed set
    std::vector<bool> closed(nodes.size(), false);

    // keep track of closest visited node to goal
    int closest_idx = start_idx;
    float_t closest_dist = start_node.h;  // h proportional to distance to goal
    bool found_boundary = false;

    const Box3 outside_boundary(
        local_bound_min + Point3::Constant(boundary_radius),
        local_bound_max - Point3::Constant(boundary_radius));

    // temp buffers reused by radiusSearch
    pcl::Indices tmp_indices;
    std::vector<float_t> tmp_dists;

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
        Node& current = nodes[idx];

        if (closed[idx])
        {
            continue;  // skip if already closed
        }
        closed[idx] = true;

        // Goal check (by geometry)
        if ((current.position() - goal).norm() < goal_threshold)
        {
            create_path(current);
            return true;
        }

        // Lazily compute neighbors on first expansion
        if (current.neighbors.empty())
        {
            const Point3 pos = current.position();
            current.neighbors.clear();
            tmp_dists.clear();
            kdtree.radiusSearch(
                PointT(pos.x(), pos.y(), pos.z()),
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
            Node& nb = nodes[nb_idx];

            // geometric edge length
            const float_t geom = (nb.position() - current.position()).norm();

            const float_t edge = lambda_dist * geom + lambda_penalty * nb.cost;

            const float_t tentative_g = current.g + edge;
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

                const float_t new_f = nb.f();
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
        create_path(nodes[closest_idx]);

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
