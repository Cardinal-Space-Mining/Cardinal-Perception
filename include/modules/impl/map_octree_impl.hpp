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

#include "../map_octree.hpp"

#include <array>
#include <limits>
// #include <iostream>

#include <pcl/features/normal_3d.h>



namespace csm
{
namespace perception
{

#if PCL_VERSION < PCL_VERSION_CALC(1, 13, 0)
OctreeContainerPointIndex_Patched::OctreeContainerPointIndex_Patched()
{
    this->reset();
}

pcl::uindex_t OctreeContainerPointIndex_Patched::getSize() const
{
    return this->data_ == static_cast<pcl::index_t>(-1) ? 0 : 1;
}
pcl::index_t OctreeContainerPointIndex_Patched::getPointIndex() const
{
    return this->data_;
}
void OctreeContainerPointIndex_Patched::getPointIndices(
    pcl::Indices& data_vector_arg) const
{
    if (this->data_ != static_cast<pcl::index_t>(-1))
    {
        data_vector_arg.push_back(this->data_);
    }
}
void OctreeContainerPointIndex_Patched::addPointIndex(pcl::index_t data_arg)
{
    data_ = data_arg;
}
void OctreeContainerPointIndex_Patched::reset()
{
    data_ = static_cast<pcl::index_t>(-1);
}

OctreeContainerPointIndex_Patched*
    OctreeContainerPointIndex_Patched::deepCopy() const
{
    return (new OctreeContainerPointIndex_Patched(*this));
}

bool OctreeContainerPointIndex_Patched::operator==(
    const OctreeContainerBase& other) const
{
    const auto* otherConDataT =
        dynamic_cast<const OctreeContainerPointIndex_Patched*>(&other);
    return (this->data_ == otherConDataT->data_);
}
#endif



const std::vector<uint64_t>& MapOctreeBase::StampStorageBase::pointStamps()
    const
{
    return this->pt_stamps;
}
uint64_t& MapOctreeBase::StampStorageBase::pointStamp(pcl::index_t pt_idx)
{
    const size_t pt_idx_ = static_cast<size_t>(pt_idx);
    assert(pt_idx_ < this->pt_stamps.size());

    return this->pt_stamps[pt_idx_];
}

const std::vector<Eigen::Vector<float, 5>>&
    MapOctreeBase::NormalStorageBase::pointNormals() const
{
    return this->pt_normals;
}



template<typename PointT, int ConfigV, typename ChildT>
MapOctree<PointT, ConfigV, ChildT>::MapOctree(const double vox_res) :
    Super_T(vox_res),
    cloud_buff{std::make_shared<PointCloud>()}
{
    this->input_ = this->cloud_buff;
    this->cloud_buff->is_dense = false;
}


template<typename PointT, int ConfigV, typename ChildT>
size_t MapOctree<PointT, ConfigV, ChildT>::addPoint(
    const PointT& pt,
    uint64_t stamp,
    bool compute_normal)
{
    pcl::octree::OctreeKey key;
    auto* pt_idx = this->getOrCreateOctreePoint(pt, key);
    if (!pt_idx)
    {
        return std::numeric_limits<size_t>::max();
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
                if constexpr (HAS_POINT_STAMPS)
                {
                    this->pt_stamps[hole_idx] = stamp;
                }
                if constexpr (HAS_POINT_NORMALS)
                {
                    if (compute_normal)
                    {
                        this->computePointNormal(hole_idx);
                    }
                }
                pt_idx->addPointIndex(hole_idx);
                return hole_idx;
            }
        }

        this->cloud_buff->push_back(pt);
        const size_t end_idx = this->cloud_buff->size() - 1;
        if constexpr (HAS_POINT_STAMPS)
        {
            this->pt_stamps.push_back(stamp);
        }
        if constexpr (HAS_POINT_NORMALS)
        {
            this->pt_normals.emplace_back();
            if (compute_normal)
            {
                this->computePointNormal(end_idx);
            }
        }
        pt_idx->addPointIndex(end_idx);
        return end_idx;
    }
    else
    {
        const size_t idx_ = pt_idx->getPointIndex();
        auto& map_point = (*this->cloud_buff)[idx_];

        if (Derived_T::mergePointFields(map_point, pt))
        {
            this->hole_indices.push_back(idx_);
            pt_idx->reset();
        }
        else
        {
            if constexpr (HAS_POINT_STAMPS)
            {
                this->pt_stamps[idx_] = stamp;
            }
            if constexpr (HAS_POINT_NORMALS)
            {
                if (compute_normal)
                {
                    this->computePointNormal(idx_);
                }
            }
        }
        return idx_;
    }

    if constexpr (!HAS_POINT_STAMPS)
    {
        (void)stamp;
    }
    if constexpr (!HAS_POINT_NORMALS)
    {
        (void)compute_normal;
    }
}

template<typename PointT, int ConfigV, typename ChildT>
void MapOctree<PointT, ConfigV, ChildT>::addPoints(
    const pcl::PointCloud<PointT>& pts,
    const pcl::Indices* indices)
{
    std::vector<size_t> map_indices;
    if (!indices)
    {
        if constexpr (HAS_POINT_NORMALS)
        {
            map_indices.reserve(pts.points.size());
        }
        for (const PointT& pt : pts.points)
        {
            size_t i = this->addPoint(pt, pts.header.stamp, false);
            if constexpr (HAS_POINT_NORMALS)
            {
                map_indices.push_back(i);
            }
            else
            {
                (void)i;
            }
        }
    }
    else
    {
        if constexpr (HAS_POINT_NORMALS)
        {
            map_indices.reserve(indices->size());
        }
        for (const pcl::index_t idx : *indices)
        {
            size_t i = this->addPoint(pts.points[idx], pts.header.stamp, false);
            if constexpr (HAS_POINT_NORMALS)
            {
                map_indices.push_back(i);
            }
            else
            {
                (void)i;
            }
        }
    }

    if constexpr (HAS_POINT_NORMALS)
    {
        for (size_t i : map_indices)
        {
            if (i != std::numeric_limits<size_t>::max())
            {
                this->computePointNormal(i);
            }
        }
    }
}

template<typename PointT, int ConfigV, typename ChildT>
void MapOctree<PointT, ConfigV, ChildT>::deletePoint(
    const pcl::index_t pt_idx,
    bool trim_nodes)
{
    const size_t pt_idx_ = static_cast<size_t>(pt_idx);
    assert(pt_idx_ < this->cloud_buff->size());

    PointT& pt = (*this->cloud_buff)[pt_idx_];

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
        this->hole_indices.push_back(pt_idx_);
        idx->reset();
    }

    if (idx->getSize() <= 0 && trim_nodes)
    {
        this->removeLeaf(key);
    }
}

template<typename PointT, int ConfigV, typename ChildT>
void MapOctree<PointT, ConfigV, ChildT>::deletePoints(
    const pcl::Indices& indices,
    bool trim_nodes)
{
    for (pcl::index_t i : indices)
    {
        this->deletePoint(i, trim_nodes);
    }
}


template<typename PointT, int ConfigV, typename ChildT>
void MapOctree<PointT, ConfigV, ChildT>::crop(
    const Vec3f& min,
    const Vec3f& max,
    bool trim_nodes)
{
#define N_INF (-std::numeric_limits<float>::infinity())
#define P_INF (std::numeric_limits<float>::infinity())

    const std::array<Vec3f, 12> inv_crop_sections{
        Vec3f{  N_INF,   N_INF, max.z()},
        Vec3f{  P_INF,   P_INF,   P_INF},

        Vec3f{min.x(), min.y(),   N_INF},
        Vec3f{max.x(), max.y(), min.z()},

        Vec3f{min.x(),   N_INF,   N_INF},
        Vec3f{  P_INF, min.y(), max.z()},

        Vec3f{max.x(), min.y(),   N_INF},
        Vec3f{  P_INF,   P_INF, max.z()},

        Vec3f{  N_INF, max.y(),   N_INF},
        Vec3f{max.x(),   P_INF, max.z()},

        Vec3f{  N_INF,   N_INF,   N_INF},
        Vec3f{min.x(), max.y(), max.z()}
    };

#undef N_INF
#undef P_INF

    pcl::Indices indices;
    for (size_t i = 0; i < inv_crop_sections.size(); i += 2)
    {
        this->boxSearch(
            inv_crop_sections[i],
            inv_crop_sections[i + 1],
            indices);
        this->deletePoints(indices, trim_nodes);
    }
}

template<typename PointT, int ConfigV, typename ChildT>
void MapOctree<PointT, ConfigV, ChildT>::optimizeStorage()
{
    // std::cout << "exhibit a" << std::endl;

    if (this->hole_indices.size() >= this->cloud_buff->size())
    {
        this->cloud_buff->clear();
        if constexpr(HAS_POINT_STAMPS)
        {
            this->pt_stamps.clear();
        }
        if constexpr(HAS_POINT_NORMALS)
        {
            this->pt_normals.clear();
        }
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
                if constexpr(HAS_POINT_STAMPS)
                {
                    this->pt_stamps[idx] = this->pt_stamps[end_idx];
                }
                if constexpr(HAS_POINT_NORMALS)
                {
                    this->pt_normals[idx] = this->pt_normals[end_idx];
                }
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
    if constexpr(HAS_POINT_STAMPS)
    {
        this->pt_stamps.resize(end_idx + 1);
    }
    if constexpr(HAS_POINT_NORMALS)
    {
        this->pt_normals.resize(end_idx + 1);
    }
    this->hole_indices.clear();

    // std::cout << "exhibit i" << std::endl;
}



template<typename PointT, int ConfigV, typename ChildT>
bool MapOctree<PointT, ConfigV, ChildT>::mergePointFields(
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

template<typename PointT, int ConfigV, typename ChildT>
void MapOctree<PointT, ConfigV, ChildT>::computePointNormal(size_t idx)
{
    if constexpr(HAS_POINT_NORMALS)
    {
        pcl::Indices neighbors;
        std::vector<float> dists;
        if (this->radiusSearch(
                this->cloud_buff->points[idx],
                this->getResolution() * 2.5,
                neighbors,
                dists))
        {
            neighbors.push_back(idx);

            Vec4f plane;
            float curvature;
            pcl::computePointNormal(*this->cloud_buff, neighbors, plane, curvature);
            this->pt_normals[idx].template head<4>() = plane;
            this->pt_normals[idx][4] = curvature;
        }
        else
        {
            this->pt_normals[idx][4] = std::numeric_limits<float>::quiet_NaN();
        }
    }
    else
    {
        (void)idx;
    }
}

template<typename PointT, int ConfigV, typename ChildT>
typename MapOctree<PointT, ConfigV, ChildT>::LeafContainer_T*
    MapOctree<PointT, ConfigV, ChildT>::getOctreePoint(
        const PointT& pt,
        pcl::octree::OctreeKey& key)
{
    this->genOctreeKeyforPoint(pt, key);
    return this->findLeaf(key);
}

template<typename PointT, int ConfigV, typename ChildT>
typename MapOctree<PointT, ConfigV, ChildT>::LeafContainer_T*
    MapOctree<PointT, ConfigV, ChildT>::getOrCreateOctreePoint(
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

};  // namespace perception
};  // namespace csm
