#pragma once

// #include <pcl/octree/octree_pointcloud.h>
// #include <pcl/octree/impl/octree_pointcloud.hpp>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/impl/octree_search.hpp>

#include <cassert>


namespace util
{
using namespace pcl::octree;

template<typename PointT>
class MapOctree : public OctreePointCloudSearch<PointT>
{
    using Super_T = OctreePointCloudSearch<PointT>;
public:
    MapOctree(const double res) : OctreePointCloudSearch<PointT>(res) {}

    // void deletePointFromCloud(const PointT& point, Super_T::PointCloudPtr cloud);
    void deletePointFromCloud(const pcl::uindex_t pt_idx, Super_T::PointCloudPtr cloud);

};


// template<typename PointT>
// void MapOctree<PointT>::deletePointFromCloud(const PointT& point, Super_T::PointCloudPtr cloud)
// {

// }

template<typename PointT>
void MapOctree<PointT>::deletePointFromCloud(const pcl::uindex_t pt_idx, Super_T::PointCloudPtr cloud)
{
    assert(this->input_ == cloud);
    assert(pt_idx < this->input_->size());

    OctreeKey key;
    const PointT& pt = (*this->input_)[pt_idx];

    this->genOctreeKeyforPoint(pt, key);

    Super_T::LeafNode* leaf_node;
    Super_T::BranchNode* parent_branch_of_leaf_node;
    auto depth_mask = this->createLeafRecursive(
        key, this->depth_mask_, this->root_node_, leaf_node, parent_branch_of_leaf_node);

    if (this->dynamic_depth_enabled_ && depth_mask)
    {
        // get amount of objects in leaf container
        std::size_t leaf_obj_count = (*leaf_node)->getSize();

        while (leaf_obj_count >= max_objs_per_leaf_ && depth_mask)
        {
            // index to branch child
            unsigned char child_idx = key.getChildIdxWithDepthMask(depth_mask * 2);

            expandLeafNode(leaf_node, parent_branch_of_leaf_node, child_idx, depth_mask);

            depth_mask = this->createLeafRecursive(key,
                                                    this->depth_mask_,
                                                    this->root_node_,
                                                    leaf_node,
                                                    parent_branch_of_leaf_node);
            leaf_obj_count = (*leaf_node)->getSize();
        }
    }

    // (*leaf_node)->addPointIndex(point_idx_arg);
    auto& pts = (*leaf_node)->getPointIndicesVector();
    for(size_t i = 0; i < pts.size(); i++)
    {
        if(pts[i] == pt_idx)
        {
            pts[i] = pts[pts.size() - 1];
            pts.resize(pts.size() - 1);
        }
    }

    if(pts.size() <= 0)
    {
        this->removeLeaf(key);
    }

    // remove from source cloud and reshuffle indices
}

};
