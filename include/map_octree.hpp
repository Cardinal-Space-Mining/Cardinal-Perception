#pragma once

#include <pcl/point_types.h>
#include <pcl/common/point_tests.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/impl/octree_search.hpp>

#include <cassert>
#include <limits>
// #include <iostream>
#include <atomic>
#include <vector>


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
    {
        this->input_ = this->cloud_buff;
        this->indices_ = typename Super_T::IndicesConstPtr();
        this->cloud_buff->is_dense = false;
    }

    void addPoint(const PointT& pt);
    void addPoints(
        const typename Super_T::PointCloudConstPtr& pts,
        const typename Super_T::IndicesConstPtr& indices = typename Super_T::IndicesConstPtr() );
    void deletePoint(const pcl::index_t pt_idx, bool trim_nodes = false);
    void deletePoints(const pcl::Indices& indices, bool trim_nodes = false);

    void normalizeCloud();

    // std::atomic<size_t> holes_added{0}, holes_removed{0}, voxel_attempts{0};

protected:
    pcl::Indices* getOctreePoints(const PointT& pt, OctreeKey& key);
    pcl::Indices& getOrCreateOctreePoints(const PointT& pt, OctreeKey& key);

    typename Super_T::PointCloudPtr cloud_buff;
    std::vector<size_t> hole_indices;

};


template<typename PointT>
void MapOctree<PointT>::addPoint(const PointT& pt)
{
    OctreeKey key;
    auto& pts = this->getOrCreateOctreePoints(pt, key);

    if(pts.empty())
    {
        while(this->hole_indices.size() > 0)
        {
            const size_t hole_idx = this->hole_indices.back();
            this->hole_indices.pop_back();
            // this->holes_removed++;
            if(hole_idx < this->cloud_buff->size())
            {
                (*this->cloud_buff)[hole_idx] = pt;
                pts.push_back(hole_idx);
                return;
            }
        }
        this->cloud_buff->push_back(pt);
        pts.push_back(this->cloud_buff->size() - 1);
    }
    else
    {
        // voxel_attempts++;
        float count = 0.f;
        Eigen::Vector3f vox = Eigen::Vector3f::Zero();
        for(auto& pt : pts)
        {
            vox += (*this->cloud_buff)[pt].getVector3fMap();
            count += 1.f;
        }
        if(count > 1.f) vox /= count;
        const float lpf_factor = std::min( (count / (count + 1.f)), 0.95f );
        (*this->cloud_buff)[pts[0]].getVector3fMap() =
            vox * lpf_factor +
            pt.getVector3fMap() * (1.f - lpf_factor);

        static_assert(std::numeric_limits<decltype(pt.x)>::has_quiet_NaN, "Point type must support NaN!");
        for(size_t i = 1; i < pts.size(); i++)
        {
            auto _idx = pts[i];
            auto& _pt = (*this->cloud_buff)[_idx];
            if constexpr(std::numeric_limits<decltype(_pt.x)>::has_quiet_NaN)
            {
                _pt.x = _pt.y = _pt.z = std::numeric_limits<decltype(_pt.x)>::quiet_NaN();
            }
            this->hole_indices.push_back(_idx);
            // holes_added++;
        }
        pts.resize(1);
    }
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
void MapOctree<PointT>::deletePoint(const pcl::index_t pt_idx, bool trim_nodes)
{
    const size_t _pt_idx = static_cast<size_t>(pt_idx);
    assert(_pt_idx < this->cloud_buff->size());

    PointT& pt = (*this->cloud_buff)[_pt_idx];

    OctreeKey key;
    auto* pts = this->getOctreePoints(pt, key);
    if(!pts) return;
    for(size_t i = 0; i < pts->size(); i++)
    {
        if(pts->at(i) == pt_idx)
        {
            pts->at(i) = pts->at(pts->size() - 1);
            pts->resize(pts->size() - 1);
            if constexpr(std::numeric_limits<decltype(pt.x)>::has_quiet_NaN)
            {
                pt.x = pt.y = pt.z = std::numeric_limits<decltype(pt.x)>::quiet_NaN();
            }
            this->hole_indices.push_back(_pt_idx);
            // holes_added++;
            break;
        }
    }

    if(pts->size() <= 0 && trim_nodes)
    {
        this->removeLeaf(key);
    }
}

template<typename PointT>
void MapOctree<PointT>::deletePoints(const pcl::Indices& indices, bool trim_nodes)
{
    for(pcl::index_t i : indices)
    {
        this->deletePoint(i, trim_nodes);
    }
}


template<typename PointT>
void MapOctree<PointT>::normalizeCloud()
{
    // std::cout << "exhibit a" << std::endl;

    if(this->hole_indices.size() >= this->cloud_buff->size())
    {
        this->cloud_buff->clear();
        this->hole_indices.clear();
        return;
    }

    // std::cout << "exhibit b" << std::endl;

    const size_t target_len = this->cloud_buff->size() - this->hole_indices.size();
    int64_t end_idx = static_cast<int64_t>(this->cloud_buff->size()) - 1;

    // std::cout << "exhibit c" << std::endl;

    OctreeKey key;
    for(const size_t idx : this->hole_indices)
    {
        // std::cout << "exhibit d" << std::endl;

        pcl::Indices* pts = nullptr;
        while( end_idx >= 0 && (
            !pcl::isFinite( (*this->cloud_buff)[end_idx] ) ||
            !(pts = this->getOctreePoints( (*this->cloud_buff)[end_idx], key ))) ) end_idx--;

        // std::cout << "exhibit e" << std::endl;

        if(!pts) break;
        if(idx >= (size_t)end_idx) continue;

        // std::cout << "exhibit f" << std::endl;

        for(size_t i = 0; i < pts->size(); i++)
        {
            if(pts->at(i) == end_idx)
            {
                (*this->cloud_buff)[idx] = (*this->cloud_buff)[end_idx];
                pts->at(i) = idx;
                end_idx--;
            }
        }

        // std::cout << "exhibit g" << std::endl;

        if(end_idx < (int64_t)target_len) break;
    }

    // std::cout << "exhibit h" << std::endl;

    this->cloud_buff->resize(end_idx + 1);
    this->hole_indices.clear();

    // std::cout << "exhibit i" << std::endl;
}


template<typename PointT>
pcl::Indices* MapOctree<PointT>::getOctreePoints(const PointT& pt, OctreeKey& key)
{
    this->genOctreeKeyforPoint(pt, key);
    auto* leaf_container = this->findLeaf(key);
    if(leaf_container)
    {
        return &leaf_container->getPointIndicesVector();
    }
    else
    {
        return nullptr;
    }
}

template<typename PointT>
pcl::Indices& MapOctree<PointT>::getOrCreateOctreePoints(const PointT& pt, OctreeKey& key)
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

};
