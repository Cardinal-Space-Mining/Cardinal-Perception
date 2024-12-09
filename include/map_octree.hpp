/*******************************************************************************
*   Copyright (C) 2024 Cardinal Space Mining Club                              *
*                                                                              *
*   Unless required by applicable law or agreed to in writing, software        *
*   distributed under the License is distributed on an "AS IS" BASIS,          *
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
*   See the License for the specific language governing permissions and        *
*   limitations under the License.                                             *
*                                                                              *
*                                ;xxxxxxx:                                     *
*                               ;$$$$$$$$$       ...::..                       *
*                               $$$$$$$$$$x   .:::::::::::..                   *
*                            x$$$$$$$$$$$$$$::::::::::::::::.                  *
*                        :$$$$$&X;      .xX:::::::::::::.::...                 *
*                .$$Xx++$$$$+  :::.     :;:   .::::::.  ....  :                *
*               :$$$$$$$$$  ;:      ;xXXXXXXXx  .::.  .::::. .:.               *
*              :$$$$$$$$: ;      ;xXXXXXXXXXXXXx: ..::::::  .::.               *
*             ;$$$$$$$$ ::   :;XXXXXXXXXXXXXXXXXX+ .::::.  .:::                *
*              X$$$$$X : +XXXXXXXXXXXXXXXXXXXXXXXX; .::  .::::.                *
*               .$$$$ :xXXXXXXXXXXXXXXXXXXXXXXXXXXX.   .:::::.                 *
*                X$$X XXXXXXXXXXXXXXXXXXXXXXXXXXXXx:  .::::.                   *
*                $$$:.XXXXXXXXXXXXXXXXXXXXXXXXXXX  ;; ..:.                     *
*                $$& :XXXXXXXXXXXXXXXXXXXXXXXX;  +XX; X$$;                     *
*                $$$::XXXXXXXXXXXXXXXXXXXXXX: :XXXXX; X$$;                     *
*                X$$X XXXXXXXXXXXXXXXXXXX; .+XXXXXXX; $$$                      *
*                $$$$ ;XXXXXXXXXXXXXXX+  +XXXXXXXXx+ X$$$+                     *
*              x$$$$$X ;XXXXXXXXXXX+ :xXXXXXXXX+   .;$$$$$$                    *
*             +$$$$$$$$ ;XXXXXXx;;+XXXXXXXXX+    : +$$$$$$$$                   *
*              +$$$$$$$$: xXXXXXXXXXXXXXX+      ; X$$$$$$$$                    *
*               :$$$$$$$$$. +XXXXXXXXX:      ;: x$$$$$$$$$                     *
*               ;x$$$$XX$$$$+ .;+X+      :;: :$$$$$xX$$$X                      *
*              ;;;;;;;;;;X$$$$$$$+      :X$$$$$$&.                             *
*              ;;;;;;;:;;;;;x$$$$$$$$$$$$$$$$x.                                *
*              :;;;;;;;;;;;;.  :$$$$$$$$$$X                                    *
*               .;;;;;;;;:;;    +$$$$$$$$$                                     *
*                 .;;;;;;.       X$$$$$$$:                                     *
*                                                                              *
*******************************************************************************/

#pragma once

// #define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>
#include <pcl/common/point_tests.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/impl/octree_base.hpp>
#include <pcl/octree/impl/octree_pointcloud.hpp>
#include <pcl/octree/impl/octree_search.hpp>

#include <cassert>
#include <limits>
// #include <iostream>
#include <atomic>
#include <vector>
#include <type_traits>


namespace csm
{
namespace perception
{


template<typename PointT>
class MapOctree :
    public pcl::octree::OctreePointCloudSearch<PointT, pcl::octree::OctreeContainerPointIndex>
{
    using Super_T = pcl::octree::OctreePointCloudSearch<PointT, pcl::octree::OctreeContainerPointIndex>;
    using LeafContainer_T = typename Super_T::OctreeT::Base::LeafContainer;
public:
    MapOctree(const double res) :
        Super_T(res),
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
    LeafContainer_T* getOctreePoint(const PointT& pt, pcl::octree::OctreeKey& key);
    LeafContainer_T* getOrCreateOctreePoint(const PointT& pt, pcl::octree::OctreeKey& key);

    typename Super_T::PointCloudPtr cloud_buff;
    std::vector<size_t> hole_indices;

};


template<typename PointT>
void MapOctree<PointT>::addPoint(const PointT& pt)
{
    pcl::octree::OctreeKey key;
    auto* pt_idx = this->getOrCreateOctreePoint(pt, key);
    if(!pt_idx) return;

    if(pt_idx->getSize() <= 0)
    {
        while(this->hole_indices.size() > 0)
        {
            const size_t hole_idx = this->hole_indices.back();
            this->hole_indices.pop_back();
            // this->holes_removed++;
            if(hole_idx < this->cloud_buff->size())
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
        constexpr static float LPF_FACTOR = 0.95f;
        ((*this->cloud_buff)[pt_idx->getPointIndex()].getVector3fMap() *= LPF_FACTOR)
            += (pt.getVector3fMap() * (1.f - LPF_FACTOR));
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

    pcl::octree::OctreeKey key;
    auto* idx = this->getOctreePoint(pt, key);
    if(!idx) return;

    if(idx->getSize() > 0 && idx->getPointIndex() == pt_idx)
    {
        if constexpr(std::numeric_limits<decltype(pt.x)>::has_quiet_NaN)
        {
            pt.x = pt.y = pt.z = std::numeric_limits<decltype(pt.x)>::quiet_NaN();
        }
        this->hole_indices.push_back(_pt_idx);
        idx->reset();
    }

    if(idx->getSize() <= 0 && trim_nodes)
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

    pcl::octree::OctreeKey key;
    for(const size_t idx : this->hole_indices)
    {
        // std::cout << "exhibit d" << std::endl;

        LeafContainer_T* pt_idx = nullptr;
        while( end_idx >= 0 && (
            !pcl::isFinite( (*this->cloud_buff)[end_idx] ) ||
            !(pt_idx = this->getOctreePoint( (*this->cloud_buff)[end_idx], key ))) ) end_idx--;

        // std::cout << "exhibit e" << std::endl;

        if(!pt_idx) break;  // logically equivalent to end_idx < 0
        if(idx >= (size_t)end_idx) continue;

        // std::cout << "exhibit f" << std::endl;

        if(pt_idx->getSize() > 0 && pt_idx->getPointIndex() == end_idx)
        {
            {
                (*this->cloud_buff)[idx] = (*this->cloud_buff)[end_idx];
                pt_idx->addPointIndex(idx);
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
typename MapOctree<PointT>::LeafContainer_T*
MapOctree<PointT>::getOctreePoint(const PointT& pt, pcl::octree::OctreeKey& key)
{
    this->genOctreeKeyforPoint(pt, key);
    return this->findLeaf(key);
}

template<typename PointT>
typename MapOctree<PointT>::LeafContainer_T*
MapOctree<PointT>::getOrCreateOctreePoint(const PointT& pt, pcl::octree::OctreeKey& key)
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

    return (leaf_node ? leaf_node->getContainerPtr() : nullptr);
}

};
};
