/*******************************************************************************
*   Copyright (C) 2024-2025 Cardinal Space Mining Club                         *
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
*                $$$: XXXXXXXXXXXXXXXXXXXXXX; :XXXXX; X$$;                     *
*                X$$X XXXXXXXXXXXXXXXXXXX; .+XXXXXXX; $$$                      *
*                $$$$ ;XXXXXXXXXXXXXXX+  +XXXXXXXXx+ X$$$+                     *
*              x$$$$$X ;XXXXXXXXXXX+ :xXXXXXXXX+   .;$$$$$$                    *
*             +$$$$$$$$ ;XXXXXXx;;+XXXXXXXXX+    : +$$$$$$$$                   *
*              +$$$$$$$$: xXXXXXXXXXXXXXX+      ; X$$$$$$$$                    *
*               :$$$$$$$$$. +XXXXXXXXX;      ;: x$$$$$$$$$                     *
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

#include <cassert>
#include <limits>
// #include <iostream>
#include <atomic>
#include <vector>
#include <type_traits>

#include <pcl/pcl_config.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/point_tests.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/impl/octree_base.hpp>
#include <pcl/octree/impl/octree_pointcloud.hpp>
#include <pcl/octree/impl/octree_search.hpp>

#include "point_def.hpp"


namespace csm
{
namespace perception
{

#if PCL_VERSION < PCL_VERSION_CALC(1, 13, 0)    // https://github.com/PointCloudLibrary/pcl/commit/7992dc3598c8f05187d084aa3b1c7c28f2653c00
    class OctreeContainerPointIndex_Patched :
        public pcl::octree::OctreeContainerBase
    {
    public:
        OctreeContainerPointIndex_Patched() { reset(); }

    public:
        pcl::uindex_t getSize() const override
            { return data_ == static_cast<pcl::index_t>(-1) ? 0 : 1; }

        pcl::index_t getPointIndex() const
            { return data_; }

        void getPointIndices(pcl::Indices& data_vector_arg) const
            { if(data_ != static_cast<pcl::index_t>(-1)) data_vector_arg.push_back(data_); }

        void addPointIndex(pcl::index_t data_arg)
            { data_ = data_arg; }

        void reset() override
            { data_ = static_cast<pcl::index_t>(-1); }

        virtual OctreeContainerPointIndex_Patched* deepCopy() const
            { return (new OctreeContainerPointIndex_Patched(*this)); }

        bool operator==(const OctreeContainerBase& other) const override
        {
            const auto* otherConDataT = dynamic_cast<const OctreeContainerPointIndex_Patched*>(&other);
            return (this->data_ == otherConDataT->data_);
        }

    protected:
        pcl::index_t data_;

    };

    using MappingLeafT = csm::perception::OctreeContainerPointIndex_Patched;
#else
    using MappingLeafT = pcl::octree::OctreeContainerPointIndex;
#endif



template<typename PointT, typename ChildT = void>
class MapOctree :
    public pcl::octree::OctreePointCloudSearch<PointT, MappingLeafT>
{
    static_assert(pcl::traits::has_xyz<PointT>::value);

    using Super_T = pcl::octree::OctreePointCloudSearch<PointT, MappingLeafT>;
    using LeafContainer_T = typename Super_T::OctreeT::Base::LeafContainer;
    using Derived_T = typename std::conditional<
        // !std::is_base_of< MapOctree<PointT, ChildT>, ChildT >::value,
        std::is_same<ChildT, void>::value,
        MapOctree<PointT, void>, ChildT >::type;

    using typename Super_T::IndicesPtr;
    using typename Super_T::IndicesConstPtr;

    using typename Super_T::PointCloud;
    using typename Super_T::PointCloudPtr;
    using typename Super_T::PointCloudConstPtr;

    constexpr static float POINT_MERGE_LPF_FACTOR = 0.95f;

public:
    MapOctree(const double voxel_res) :
        Super_T(voxel_res),
        cloud_buff{ std::make_shared<PointCloud>() }
    {
        this->input_ = this->cloud_buff;
        this->indices_ = IndicesConstPtr{};
        this->cloud_buff->is_dense = false;
    }

    void setInputCloud(const PointCloudConstPtr&, const IndicesConstPtr&)
    {
        static_assert(0,
            "This method should not be used with MapOctree! "
            "Use addPoint(...) or addPoints(...) instead to initialize a pointcloud!");
    }

    void addPoint(const PointT& pt);
    void addPoints(
        const pcl::PointCloud<PointT>& pts,
        const pcl::Indices* indices = nullptr );
    void deletePoint(const pcl::index_t pt_idx, bool trim_nodes = false);
    void deletePoints(const pcl::Indices& indices, bool trim_nodes = false);

    void normalizeCloud();

    // std::atomic<size_t> holes_added{0}, holes_removed{0}, voxel_attempts{0};

protected:
    /* Returns true if the point should be deleted (default always false) */
    static bool mergePointFields(PointT& map_point, const PointT& new_point);

    LeafContainer_T* getOctreePoint(const PointT& pt, pcl::octree::OctreeKey& key);
    LeafContainer_T* getOrCreateOctreePoint(const PointT& pt, pcl::octree::OctreeKey& key);

    typename Super_T::PointCloudPtr cloud_buff;
    std::vector<size_t> hole_indices;

};





template<typename PointT, typename ChildT>
void MapOctree<PointT, ChildT>::addPoint(const PointT& pt)
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
        auto& map_point = (*this->cloud_buff)[pt_idx->getPointIndex()];

        if(Derived_T::mergePointFields(map_point, pt))
        {
            this->hole_indices.push_back(pt_idx->getPointIndex());
            pt_idx->reset();
        }
    }
}

template<typename PointT, typename ChildT>
void MapOctree<PointT, ChildT>::addPoints(
    const pcl::PointCloud<PointT>& pts,
    const pcl::Indices* indices )
{
    if(!indices)
    {
        for(const PointT& pt : pts.points)
        {
            this->addPoint(pt);
        }
    }
    else
    {
        for(const pcl::index_t i : *indices)
        {
            this->addPoint(pts.points[i]);
        }
    }
}

template<typename PointT, typename ChildT>
void MapOctree<PointT, ChildT>::deletePoint(const pcl::index_t pt_idx, bool trim_nodes)
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

template<typename PointT, typename ChildT>
void MapOctree<PointT, ChildT>::deletePoints(const pcl::Indices& indices, bool trim_nodes)
{
    for(pcl::index_t i : indices)
    {
        this->deletePoint(i, trim_nodes);
    }
}


template<typename PointT, typename ChildT>
void MapOctree<PointT, ChildT>::normalizeCloud()
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
        if(idx >= static_cast<size_t>(end_idx)) continue;

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



template<typename PointT, typename ChildT>
bool MapOctree<PointT, ChildT>::mergePointFields(PointT& map_point, const PointT& new_point)
{
    static constexpr float INV_LPF_FACTOR = (1.f - POINT_MERGE_LPF_FACTOR);

    (map_point.getVector3fMap() *= POINT_MERGE_LPF_FACTOR)
        += (new_point.getVector3fMap() * INV_LPF_FACTOR);

    if constexpr(util::traits::has_intensity<PointT>::value)
    {
        (map_point.intensity *= POINT_MERGE_LPF_FACTOR)
            += (new_point.intensity * INV_LPF_FACTOR);
    }
    if constexpr(util::traits::has_reflective<PointT>::value)
    {
        (map_point.reflective *= POINT_MERGE_LPF_FACTOR)
            += (new_point.reflective * INV_LPF_FACTOR);
    }
    if constexpr(pcl::traits::has_normal<PointT>::value)
    {
        (map_point.getNormalVector3fMap() *= POINT_MERGE_LPF_FACTOR)
            += (new_point.getNormalVector3fMap() * INV_LPF_FACTOR);
    }
    if constexpr(pcl::traits::has_curvature<PointT>::value)
    {
        (map_point.curvature *= POINT_MERGE_LPF_FACTOR)
            += (new_point.curvature * INV_LPF_FACTOR);
    }
    if constexpr(pcl::traits::has_label<PointT>::value)
    {
        map_point.label = new_point.label;
    }
    // TODO?: handle color fields

    return false;
}

template<typename PointT, typename ChildT>
typename MapOctree<PointT, ChildT>::LeafContainer_T*
MapOctree<PointT, ChildT>::getOctreePoint(const PointT& pt, pcl::octree::OctreeKey& key)
{
    this->genOctreeKeyforPoint(pt, key);
    return this->findLeaf(key);
}

template<typename PointT, typename ChildT>
typename MapOctree<PointT, ChildT>::LeafContainer_T*
MapOctree<PointT, ChildT>::getOrCreateOctreePoint(const PointT& pt, pcl::octree::OctreeKey& key)
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
