#pragma once

#include <queue>
#include <limits>
#include <vector>
#include <type_traits>
#include <unordered_set>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>

#include "util.hpp"

#ifndef PATH_PLANNING_PEDANTIC
    #define PATH_PLANNING_PEDANTIC 0
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
        inline Point3 position() const { return trav_point.getVector3fMap(); }
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
        FloatT boundary_node_threshold,
        FloatT goal_threshold,
        FloatT search_radius,
        size_t max_neighbors = 10)
    {
        this->boundary_node_threshold = boundary_node_threshold;
        this->goal_threshold = goal_threshold;
        this->search_radius = search_radius;
        this->max_neighbors = max_neighbors;
    };

private:
    pcl::search::KdTree<PointT> kdtree;

    std::vector<Node> nodes;  // all nodes in the search space

    // threshold for considering a node as boundary
    FloatT boundary_node_threshold = 0.0f;
    // threshold for considering goal reached
    FloatT goal_threshold = 0.1f;
    // radius for neighbor search
    FloatT search_radius = 0.37f;
    // maximum number of neighbors to consider
    size_t max_neighbors = 10;
};





#ifndef PATH_PLANNER_PRECOMPILED
#include <iostream>

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
    #ifdef PATH_PLANNING_PEDANTIC
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
        throw std::invalid_argument("Location or meta cloud is empty");
    }
    #endif

    nodes.clear();
    nodes.reserve(loc_cloud.points.size());

    size_t end_i = loc_cloud.points.size();
    for (size_t i = 0; i < end_i;)
    {
        const auto& point = loc_cloud.points[i];
        const auto& meta = meta_cloud.points[i];

        if( meta.trav_weight() < 0 ||
            meta.trav_weight() == std::numeric_limits<float>::infinity())
        {
            end_i--;
            loc_cloud.points[i] = loc_cloud.points[end_i];
            meta_cloud.points[i] = meta_cloud.points[end_i];
        }
        else
        {
            FloatT h = (goal - point.getVector3fMap()).norm();
            nodes.emplace_back(point, meta, h);
            i++;
        }
    }
    loc_cloud.points.resize(end_i);
    meta_cloud.points.resize(end_i);

    auto shared_loc_cloud = util::wrap_unmanaged(loc_cloud);
    kdtree.setInputCloud(shared_loc_cloud);

    std::cout << "[PATH PLANNING]: created " << nodes.size() << " total nodes" << std::endl;

    // find start node
    pcl::Indices kdtree_indices;
    std::vector<FloatT> kdtree_distances;
    kdtree.nearestKSearch(
        PointT(start.x(), start.y(), start.z()),
        1,
        kdtree_indices,
        kdtree_distances);
    int start_idx = kdtree_indices[0];
    Node& start_node = nodes[start_idx];
    start_node.g = 0.0f;
    {
        const Point3& pos = start_node.position();
        kdtree.radiusSearch(
            PointT(pos.x(), pos.y(), pos.z()),
            search_radius,
            start_node.neighbors,
            kdtree_distances,
            max_neighbors);
    }

    auto cmp = [this](const int a_idx, const int b_idx)
    {
        // min-heap based on f value
        return nodes[a_idx].f() > nodes[b_idx].f();
    };
    std::priority_queue<int, std::vector<int>, decltype(cmp)> q(cmp);
    std::unordered_set<int> in_q;

    // std::cout << "[PATH PLANNING]: begin iteration --------------" << std::endl;

    q.emplace(start_idx);
    in_q.insert(start_idx);
    while (!q.empty())
    {
        int idx = q.top();
        Node& current = nodes[idx];
        in_q.erase(idx);
        q.pop();

        // std::cout << "\t" << idx << std::endl;

        // Check if we reached the goal
        if ((current.position() - goal).norm() < goal_threshold)
        {
            std::cout << "[PATH PLANNING]: reached goal" << std::endl;

            // Reconstruct path
            path.clear();
            for (Node* n = &current; n != nullptr; n = n->parent)
            {
                path.push_back(n->position());
            }
            std::reverse(path.begin(), path.end());
            return true;
        }

        // Get neighbors if node is visited for the first time. We know if a
        // node is visited for the first time if its g value is infinity
        if (current.neighbors.empty())
        {
            const Point3& pos = current.position();
            kdtree.radiusSearch(
                PointT(pos.x(), pos.y(), pos.z()),
                search_radius,
                current.neighbors,
                kdtree_distances,
                max_neighbors);
        }

        for (const auto& neighbor_index : current.neighbors)
        {
            Node& neighbor = nodes[neighbor_index];

            FloatT tentative_g = current.g + neighbor.cost;

            if (tentative_g < neighbor.g)
            {
                neighbor.g = tentative_g;
                neighbor.parent = &current;

                if (in_q.find(neighbor_index) == in_q.end())
                {
                    q.emplace(neighbor_index);
                    in_q.insert(neighbor_index);
                }
            }
        }
    }

    // std::cout << "[PATH PLANNING]: end iteration --------------" << std::endl;

    // If we reach here, no path was found, instead find the closest point to
    // goal on the boundary
    pcl::IndicesPtr boundary_indices = std::make_shared<pcl::Indices>();
    for (size_t i = 0; i < nodes.size(); ++i)
    {
        const Node& node = nodes[i];
        if ((node.position() - start).squaredNorm() >
            boundary_node_threshold * boundary_node_threshold)
        {
            boundary_indices->push_back(i);
        }
    }
    if (boundary_indices->empty())
    {
        std::cout << "[PATH PLANNING]: no boundary node found" << std::endl;
        return false;
    }

    kdtree.setInputCloud(shared_loc_cloud, boundary_indices);
    kdtree.nearestKSearch(
        PointT(goal.x(), goal.y(), goal.z()),
        1,
        kdtree_indices,
        kdtree_distances);
    Node frontier_node = nodes[kdtree_indices[0]];
    // Reconstruct path
    path.clear();
    for (Node* n = &frontier_node; n != nullptr; n = n->parent)
    {
        path.push_back(n->position());
    }
    std::reverse(path.begin(), path.end());
    return true;
}



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

}  // namespace perception
}  // namespace csm
