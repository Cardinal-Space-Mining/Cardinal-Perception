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
