#pragma once

#include <vector>
#include <limits>
#include <type_traits>
#include <queue>
#include <unordered_set>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>

#include "../config.hpp"
#include "util.hpp"

#ifndef PATH_PLANNING_PEDANTIC
#define PATH_PLANNING_PEDANTIC 0
#endif


namespace csm
{
namespace perception
{

template <typename float_t = float>
class PathPlanner {
    static_assert(std::is_floating_point<float_t>::value);

public:
    using Point3 = Eigen::Vector<float_t, 3>;
    using LocationCloud = pcl::PointCloud<TraversibilityPointType>;
    using MetaCloud = pcl::PointCloud<TraversibilityMetaType>;

private:
    struct Node {
        const TraversibilityPointType &trav_point;
        float_t cost; // traversal cost of this node only
        float_t g; // cost from start to this node
        float_t h; // heuristic cost to goal
        Node* parent = nullptr;
        pcl::Indices neighbors;
        
        Node(
            const TraversibilityPointType& point, 
            const TraversibilityMetaType& meta, 
            float_t h = 0.0f, 
            Node* p = nullptr
        ) :
            trav_point(point),
            cost(meta.data_n[3]),
            g(std::numeric_limits<float_t>::infinity()),
            h(h),
            parent(p)
        {}

        inline float_t f() const { return g + h; } // total cost
        inline Point3 position() const { return trav_point.getVector3fMap(); }
    };

public:
    PathPlanner() = default;
    ~PathPlanner() = default;

    bool solvePath(
        const Point3& start, const Point3& goal,
        const Point3& local_bound_min, const Point3& local_bound_max,
        const LocationCloud& loc_cloud,
        const MetaCloud& meta_cloud,
        std::vector<Point3>& path);

    inline void setParameters(
        float_t boundary_node_threshold,
        float_t goal_threshold,
        float_t search_radius,
        size_t max_neighbors = 10)
    {
        this->boundary_node_threshold = boundary_node_threshold;
        this->goal_threshold = goal_threshold;
        this->search_radius = search_radius;
        this->max_neighbors = max_neighbors;
    };

private:
    pcl::search::KdTree<TraversibilityPointType> kdtree;

    std::vector<Node> nodes; // all nodes in the search space

    float_t boundary_node_threshold = 0.0f; // threshold for considering a node as boundary
    float_t goal_threshold = 0.1f; // threshold for considering goal reached
    float_t search_radius = 1.0f; // radius for neighbor search
    size_t max_neighbors = 10; // maximum number of neighbors to consider
};

template <typename float_t>
bool PathPlanner<float_t>::solvePath(
    const Point3& start, const Point3& goal,
    const Point3& local_bound_min, const Point3& local_bound_max,
    const LocationCloud& loc_cloud,
    const MetaCloud& meta_cloud,
    std::vector<Point3>& path)
{
#ifdef PATH_PLANNING_PEDANTIC
    if (start.x() < local_bound_min.x() || start.y() < local_bound_min.y() ||
        start.x() > local_bound_max.x() || start.y() > local_bound_max.y()) {
        throw std::out_of_range("Start point is out of bounds");
    }
    if (loc_cloud.points.size() != meta_cloud.points.size()) {
        throw std::invalid_argument("Location and meta clouds must have the same size");
    }
    if (loc_cloud.points.empty() || meta_cloud.points.empty()) {
        throw std::invalid_argument("Location or meta cloud is empty");
    }
#endif

    auto shared_loc_cloud = util::wrap_unmanaged(loc_cloud);

    kdtree.setInputCloud(shared_loc_cloud);

    nodes.clear();
    nodes.reserve(loc_cloud.points.size());

    for (size_t i = 0; i < loc_cloud.points.size(); ++i) {
        const auto& point = loc_cloud.points[i];
        const auto& meta = meta_cloud.points[i];
        float_t h = (goal - point.getVector3fMap()).norm();

        nodes.emplace_back(point, meta, h);
    }

    // find start node
    pcl::Indices kdtree_indices;
    std::vector<float_t> kdtree_distances;
    kdtree.nearestKSearch(
        TraversibilityPointType(start.x(), start.y(), start.z()),
        1,
        kdtree_indices,
        kdtree_distances
    );
    int start_idx = kdtree_indices[0];
    Node &start_node = nodes[start_idx];
    start_node.g = 0.0f;

    auto cmp = [this](const int a_idx, const int b_idx) {
        return nodes[a_idx].f() > nodes[b_idx].f(); // min-heap based on f value
    };
    std::priority_queue<int, std::vector<int>, decltype(cmp)> q(cmp);
    std::unordered_set<int> in_q;

    q.emplace(start_idx);
    in_q.insert(start_idx);
    while (!q.empty()) {
        int idx = q.top();
        Node &current = nodes[idx];
        in_q.erase(idx);
        q.pop();

        // Check if we reached the goal
        if ((current.position() - goal).norm() < goal_threshold) {
            // Reconstruct path
            path.clear();
            for (Node* n = &current; n != nullptr; n = n->parent) {
                path.push_back(n->position());
            }
            std::reverse(path.begin(), path.end());
            return true;
        }

        // Get neighbors if node is visited for the first time
        // We know if a node is visited for the first time if its g value is infinity
        if (current.g == std::numeric_limits<float_t>::infinity()) {
            const Point3 &pos = current.position();
            kdtree.radiusSearch(
                TraversibilityPointType(pos.x(), pos.y(), pos.z()),
                search_radius,
                current.neighbors,
                kdtree_distances,
                max_neighbors
            );
        }

        for (const auto& neighbor_index : current.neighbors) {
            Node& neighbor = nodes[neighbor_index];

            float_t tentative_g = current.g + neighbor.cost;

            if (tentative_g < neighbor.g) {
                neighbor.g = tentative_g;
                neighbor.parent = &current;

                if (in_q.find(neighbor_index) == in_q.end()) {
                    q.emplace(neighbor_index);
                    in_q.insert(neighbor_index);
                }
            }
        }
    }
    // If we reach here, no path was found, instead find the closest point to goal on the boundary
    pcl::IndicesPtr boundary_indices = std::make_shared<pcl::Indices>();
    for (size_t i = 0; i < nodes.size(); ++i) {
        const Node& node = nodes[i];
        if ((node.position() - start).squaredNorm() > boundary_node_threshold*boundary_node_threshold) {
            boundary_indices->push_back(i);
        }
    }
    if (boundary_indices->empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("PathPlanner"), "No boundary nodes found");
        return false;
    }

    kdtree.setInputCloud(shared_loc_cloud, boundary_indices);
    kdtree.nearestKSearch(
        TraversibilityPointType(goal.x(), goal.y(), goal.z()),
        1,
        kdtree_indices,
        kdtree_distances
    );
    Node frontier_node = nodes[kdtree_indices[0]];
    // Reconstruct path
    path.clear();
    for (Node* n = &frontier_node; n != nullptr; n = n->parent) {
        path.push_back(n->position());
    }
    std::reverse(path.begin(), path.end());
    return true;
}
    
} // namespace perception
} // namespace csm
