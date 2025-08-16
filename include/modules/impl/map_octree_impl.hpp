#include "../map_octree.hpp"

#include <limits>
// #include <iostream>

// these were at the top -- unsure if they are needed for non-pcl template instantiations?
// #include <pcl/octree/impl/octree_base.hpp>
// #include <pcl/octree/impl/octree_pointcloud.hpp>
// #include <pcl/octree/impl/octree_search.hpp>



namespace csm
{
namespace perception
{

template<typename PointT, typename ChildT>
void MapOctree<PointT, ChildT>::addPoint(const PointT& pt)
{
    pcl::octree::OctreeKey key;
    auto* pt_idx = this->getOrCreateOctreePoint(pt, key);
    if (!pt_idx)
    {
        return;
    }

    if (pt_idx->getSize() <= 0)
    {
        while (this->hole_indices.size() > 0)
        {
            const size_t hole_idx = this->hole_indices.back();
            this->hole_indices.pop_back();
            // this->holes_removed++;
            if (hole_idx < this->cloud_buff->size())
            {
                (*this->cloud_buff)[hole_idx] = pt;
                pt_idx->addPointIndex(hole_idx);
                return;
            }
        }
        this->cloud_buff->push_back(pt);
        pt_idx->addPointIndex(this->cloud_buff->size() - 1);
    }
    else
    {
        auto& map_point = (*this->cloud_buff)[pt_idx->getPointIndex()];

        if (Derived_T::mergePointFields(map_point, pt))
        {
            this->hole_indices.push_back(pt_idx->getPointIndex());
            pt_idx->reset();
        }
    }
}

template<typename PointT, typename ChildT>
void MapOctree<PointT, ChildT>::addPoints(
    const pcl::PointCloud<PointT>& pts,
    const pcl::Indices* indices)
{
    if (!indices)
    {
        for (const PointT& pt : pts.points)
        {
            this->addPoint(pt);
        }
    }
    else
    {
        for (const pcl::index_t i : *indices)
        {
            this->addPoint(pts.points[i]);
        }
    }
}

template<typename PointT, typename ChildT>
void MapOctree<PointT, ChildT>::deletePoint(
    const pcl::index_t pt_idx,
    bool trim_nodes)
{
    const size_t _pt_idx = static_cast<size_t>(pt_idx);
    assert(_pt_idx < this->cloud_buff->size());

    PointT& pt = (*this->cloud_buff)[_pt_idx];

    pcl::octree::OctreeKey key;
    auto* idx = this->getOctreePoint(pt, key);
    if (!idx)
    {
        return;
    }

    if (idx->getSize() > 0 && idx->getPointIndex() == pt_idx)
    {
        if constexpr (std::numeric_limits<decltype(pt.x)>::has_quiet_NaN)
        {
            pt.x = pt.y = pt.z =
                std::numeric_limits<decltype(pt.x)>::quiet_NaN();
        }
        this->hole_indices.push_back(_pt_idx);
        idx->reset();
    }

    if (idx->getSize() <= 0 && trim_nodes)
    {
        this->removeLeaf(key);
    }
}

template<typename PointT, typename ChildT>
void MapOctree<PointT, ChildT>::deletePoints(
    const pcl::Indices& indices,
    bool trim_nodes)
{
    for (pcl::index_t i : indices)
    {
        this->deletePoint(i, trim_nodes);
    }
}


template<typename PointT, typename ChildT>
void MapOctree<PointT, ChildT>::normalizeCloud()
{
    // std::cout << "exhibit a" << std::endl;

    if (this->hole_indices.size() >= this->cloud_buff->size())
    {
        this->cloud_buff->clear();
        this->hole_indices.clear();
        return;
    }

    // std::cout << "exhibit b" << std::endl;

    const size_t target_len =
        this->cloud_buff->size() - this->hole_indices.size();
    int64_t end_idx = static_cast<int64_t>(this->cloud_buff->size()) - 1;

    // std::cout << "exhibit c" << std::endl;

    pcl::octree::OctreeKey key;
    for (const size_t idx : this->hole_indices)
    {
        // std::cout << "exhibit d" << std::endl;

        LeafContainer_T* pt_idx = nullptr;
        while (end_idx >= 0 &&
               (!pcl::isFinite((*this->cloud_buff)[end_idx]) ||
                !(pt_idx =
                      this->getOctreePoint((*this->cloud_buff)[end_idx], key))))
        {
            end_idx--;
        }

        // std::cout << "exhibit e" << std::endl;

        if (!pt_idx)
        {
            break;  // logically equivalent to end_idx < 0
        }
        if (idx >= static_cast<size_t>(end_idx))
        {
            continue;
        }

        // std::cout << "exhibit f" << std::endl;

        if (pt_idx->getSize() > 0 && pt_idx->getPointIndex() == end_idx)
        {
            {
                (*this->cloud_buff)[idx] = (*this->cloud_buff)[end_idx];
                pt_idx->addPointIndex(idx);
                end_idx--;
            }
        }

        // std::cout << "exhibit g" << std::endl;

        if (end_idx < (int64_t)target_len)
        {
            break;
        }
    }

    // std::cout << "exhibit h" << std::endl;

    this->cloud_buff->resize(end_idx + 1);
    this->hole_indices.clear();

    // std::cout << "exhibit i" << std::endl;
}



template<typename PointT, typename ChildT>
bool MapOctree<PointT, ChildT>::mergePointFields(
    PointT& map_point,
    const PointT& new_point)
{
    static constexpr float INV_LPF_FACTOR = (1.f - POINT_MERGE_LPF_FACTOR);

    (map_point.getVector3fMap() *= POINT_MERGE_LPF_FACTOR) +=
        (new_point.getVector3fMap() * INV_LPF_FACTOR);

    if constexpr (util::traits::has_intensity<PointT>::value)
    {
        (map_point.intensity *= POINT_MERGE_LPF_FACTOR) +=
            (new_point.intensity * INV_LPF_FACTOR);
    }
    if constexpr (util::traits::has_reflective<PointT>::value)
    {
        (map_point.reflective *= POINT_MERGE_LPF_FACTOR) +=
            (new_point.reflective * INV_LPF_FACTOR);
    }
    if constexpr (pcl::traits::has_normal<PointT>::value)
    {
        (map_point.getNormalVector3fMap() *= POINT_MERGE_LPF_FACTOR) +=
            (new_point.getNormalVector3fMap() * INV_LPF_FACTOR);
    }
    if constexpr (pcl::traits::has_curvature<PointT>::value)
    {
        (map_point.curvature *= POINT_MERGE_LPF_FACTOR) +=
            (new_point.curvature * INV_LPF_FACTOR);
    }
    if constexpr (pcl::traits::has_label<PointT>::value)
    {
        map_point.label = new_point.label;
    }
    // TODO?: handle color fields

    return false;
}

template<typename PointT, typename ChildT>
typename MapOctree<PointT, ChildT>::LeafContainer_T*
    MapOctree<PointT, ChildT>::getOctreePoint(
        const PointT& pt,
        pcl::octree::OctreeKey& key)
{
    this->genOctreeKeyforPoint(pt, key);
    return this->findLeaf(key);
}

template<typename PointT, typename ChildT>
typename MapOctree<PointT, ChildT>::LeafContainer_T*
    MapOctree<PointT, ChildT>::getOrCreateOctreePoint(
        const PointT& pt,
        pcl::octree::OctreeKey& key)
{
    // make sure bounding box is big enough
    this->adoptBoundingBoxToPoint(pt);

    // generate key
    this->genOctreeKeyforPoint(pt, key);

    typename Super_T::LeafNode* leaf_node;
    typename Super_T::BranchNode* parent_branch_of_leaf_node;
    auto depth_mask = this->createLeafRecursive(
        key,
        this->depth_mask_,
        this->root_node_,
        leaf_node,
        parent_branch_of_leaf_node);

    if (this->dynamic_depth_enabled_ && depth_mask)
    {
        // get amount of objects in leaf container
        std::size_t leaf_obj_count = (*leaf_node)->getSize();

        while (leaf_obj_count >= this->max_objs_per_leaf_ && depth_mask)
        {
            // index to branch child
            unsigned char child_idx =
                key.getChildIdxWithDepthMask(depth_mask * 2);

            this->expandLeafNode(
                leaf_node,
                parent_branch_of_leaf_node,
                child_idx,
                depth_mask);

            depth_mask = this->createLeafRecursive(
                key,
                this->depth_mask_,
                this->root_node_,
                leaf_node,
                parent_branch_of_leaf_node);
            leaf_obj_count = (*leaf_node)->getSize();
        }
    }

    return (leaf_node ? leaf_node->getContainerPtr() : nullptr);
}

};
};
