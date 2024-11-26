#pragma once

#include <pcl/point_types.h>
// #include <pcl/octree/octree_pointcloud.h>
// #include <pcl/octree/impl/octree_pointcloud.hpp>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/impl/octree_search.hpp>

#include <cassert>
#include <queue>
#include <limits>
#include <iostream>


namespace util
{
using namespace pcl::octree;

template<typename PointT>
class MapOctree : public OctreePointCloudSearch<PointT>
{
    using Super_T = OctreePointCloudSearch<PointT>;
public:
    MapOctree(const double res) :
        OctreePointCloudSearch<PointT>(res),
        cloud_buff{ std::make_shared<typename Super_T::PointCloud>() }
        // indices_buff{ std::make_shared<pcl::Indices>() }
    {
        this->input_ = this->cloud_buff;
        this->indices_ = typename Super_T::IndicesConstPtr();
        this->cloud_buff->is_dense = false;
    }

    // Super_T::PointCloudConstPtr getCloud() const { return this->cloud_buff; }
    // Super_T::IndicesConstPtr getIndices() const { return this->indices_buff; }

    void addPoint(const PointT& pt);
    void addPoints(
        const typename Super_T::PointCloudConstPtr& pts,
        const typename Super_T::IndicesConstPtr& indices = typename Super_T::IndicesConstPtr() );
    void deletePoint(const pcl::index_t pt_idx);

protected:
    pcl::Indices& getOctreePoints(const PointT& pt, OctreeKey& key);
    void addPointIdxAndVoxelize(const size_t idx);

    typename Super_T::PointCloudPtr cloud_buff;
    // Super_T::IndicesPtr indices_buff;
    std::deque<size_t> hole_indices;

};


template<typename PointT>
void MapOctree<PointT>::addPoint(const PointT& pt)
{
    while(this->hole_indices.size() > 0)
    {
        const size_t hole_idx = this->hole_indices.front();
        this->hole_indices.pop_front();
        if(hole_idx < this->cloud_buff->size())
        {
            (*this->cloud_buff)[hole_idx] = pt;
            this->addPointIdxAndVoxelize(hole_idx);
            return;
        }
    }
    this->cloud_buff->push_back(pt);
    this->addPointIdxAndVoxelize(this->cloud_buff->size() - 1);
}

template<typename PointT>
void MapOctree<PointT>::addPoints(
    const typename Super_T::PointCloudConstPtr& pts,
    const typename Super_T::IndicesConstPtr& indices )
{
    if(!indices)
    {
        for(const PointT& pt : pts->points)
        {
            this->addPoint(pt);
        }
    }
    else
    {
        for(const pcl::index_t i : *indices)
        {
            this->addPoint(pts->points[i]);
        }
    }
}

template<typename PointT>
void MapOctree<PointT>::deletePoint(const pcl::index_t pt_idx)
{
    // assert(this->input_ == cloud);
    const size_t _pt_idx = static_cast<size_t>(pt_idx);
    assert(_pt_idx < this->cloud_buff->size());

    PointT& pt = (*this->cloud_buff)[_pt_idx];

    OctreeKey key;
    auto& pts = this->getOctreePoints(pt, key);
    for(size_t i = 0; i < pts.size(); i++)
    {
        if(pts[i] == pt_idx)
        {
            pts[i] = pts[pts.size() - 1];
            pts.resize(pts.size() - 1);
            if constexpr(std::numeric_limits<decltype(pt.x)>::has_quiet_NaN)
            {
                pt.x = pt.y = pt.z = std::numeric_limits<decltype(pt.x)>::quiet_NaN();
            }
            this->hole_indices.push_back(_pt_idx);
            break;
        }
    }

    if(pts.size() <= 0)
    {
        this->removeLeaf(key);
    }
}


template<typename PointT>
pcl::Indices& MapOctree<PointT>::getOctreePoints(const PointT& pt, OctreeKey& key)
{
    // make sure bounding box is big enough
    this->adoptBoundingBoxToPoint(pt);

    // generate key
    this->genOctreeKeyforPoint(pt, key);

    typename Super_T::LeafNode* leaf_node;
    typename Super_T::BranchNode* parent_branch_of_leaf_node;
    auto depth_mask = this->createLeafRecursive(
        key, this->depth_mask_, this->root_node_, leaf_node, parent_branch_of_leaf_node);

    if (this->dynamic_depth_enabled_ && depth_mask) {
        // get amount of objects in leaf container
        std::size_t leaf_obj_count = (*leaf_node)->getSize();

        while (leaf_obj_count >= this->max_objs_per_leaf_ && depth_mask) {
            // index to branch child
            unsigned char child_idx = key.getChildIdxWithDepthMask(depth_mask * 2);

            this->expandLeafNode(leaf_node, parent_branch_of_leaf_node, child_idx, depth_mask);

            depth_mask = this->createLeafRecursive(key,
                                                    this->depth_mask_,
                                                    this->root_node_,
                                                    leaf_node,
                                                    parent_branch_of_leaf_node);
            leaf_obj_count = (*leaf_node)->getSize();
        }
    }

    return (*leaf_node)->getPointIndicesVector();
}

template<typename PointT>
void MapOctree<PointT>::addPointIdxAndVoxelize(const size_t idx)
{
    assert(idx < this->cloud_buff->size());

    const PointT& pt = (*this->cloud_buff)[idx];

    OctreeKey key;
    auto& pts = this->getOctreePoints(pt, key);

    if(pts.empty()) pts.push_back(idx);
    else
    {
        float count = 0;
        Eigen::Vector3f vox = Eigen::Vector3f::Zero();
        for(auto& pt : pts)
        {
            vox += (*this->cloud_buff)[pt].getVector3fMap();
            count += 1.f;
        }
        const float lpf_factor = std::min( (count / (count + 1.f)), 0.9f );
        (*this->cloud_buff)[pts[0]].getVector3fMap() =
            vox * lpf_factor +
            pt.getVector3fMap() * (1.f - lpf_factor);

        for(size_t i = 1; i < pts.size(); i++)
        {
            auto _idx = pts[i];
            auto& _pt = (*this->cloud_buff)[_idx];
            if constexpr(std::numeric_limits<decltype(_pt.x)>::has_quiet_NaN)
            {
                _pt.x = _pt.y = _pt.z = std::numeric_limits<decltype(_pt.x)>::quiet_NaN();
            }
            this->hole_indices.push_back(_idx);
        }
    }
}

};
