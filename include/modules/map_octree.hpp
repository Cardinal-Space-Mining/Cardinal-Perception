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

// #include <atomic>
#include <vector>
#include <cassert>
#include <type_traits>

#include <Eigen/Core>

#include <pcl/pcl_config.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/point_tests.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree_base.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_search.h>

#include "point_def.hpp"


namespace csm
{
namespace perception
{

// https://github.com/PointCloudLibrary/pcl/commit/7992dc3598c8f05187d084aa3b1c7c28f2653c00
#if PCL_VERSION < PCL_VERSION_CALC(1, 13, 0)

class OctreeContainerPointIndex_Patched :
    public pcl::octree::OctreeContainerBase
{
public:
    OctreeContainerPointIndex_Patched() { reset(); }

public:
    pcl::uindex_t getSize() const override
    {
        return data_ == static_cast<pcl::index_t>(-1) ? 0 : 1;
    }

    pcl::index_t getPointIndex() const { return data_; }

    void getPointIndices(pcl::Indices& data_vector_arg) const
    {
        if (data_ != static_cast<pcl::index_t>(-1))
        {
            data_vector_arg.push_back(data_);
        }
    }

    void addPointIndex(pcl::index_t data_arg) { data_ = data_arg; }

    void reset() override { data_ = static_cast<pcl::index_t>(-1); }

    virtual OctreeContainerPointIndex_Patched* deepCopy() const
    {
        return (new OctreeContainerPointIndex_Patched(*this));
    }

    bool operator==(const OctreeContainerBase& other) const override
    {
        const auto* otherConDataT =
            dynamic_cast<const OctreeContainerPointIndex_Patched*>(&other);
        return (this->data_ == otherConDataT->data_);
    }

protected:
    pcl::index_t data_;
};

using MapOctreeLeafT = csm::perception::OctreeContainerPointIndex_Patched;
#else
using MapOctreeLeafT = pcl::octree::OctreeContainerPointIndex;
#endif



template<typename PointT, typename ChildT = void>
class MapOctree :
    public pcl::octree::OctreePointCloudSearch<PointT, MapOctreeLeafT>
{
    static_assert(pcl::traits::has_xyz<PointT>::value);

    using Super_T = pcl::octree::OctreePointCloudSearch<PointT, MapOctreeLeafT>;
    using LeafContainer_T = typename Super_T::OctreeT::Base::LeafContainer;
    using Derived_T = typename std::conditional<
        // !std::is_base_of< MapOctree<PointT, ChildT>, ChildT >::value,
        std::is_same<ChildT, void>::value,
        MapOctree<PointT, void>,
        ChildT>::type;

    using typename Super_T::IndicesPtr;
    using typename Super_T::IndicesConstPtr;

    using typename Super_T::PointCloud;
    using typename Super_T::PointCloudPtr;
    using typename Super_T::PointCloudConstPtr;

    using Vec3f = Eigen::Vector3f;

    constexpr static float POINT_MERGE_LPF_FACTOR = 0.95f;

public:
    MapOctree(const double voxel_res) :
        Super_T(voxel_res),
        cloud_buff{std::make_shared<PointCloud>()}
    {
        this->input_ = this->cloud_buff;
        // this->indices_ = IndicesConstPtr{};
        this->cloud_buff->is_dense = false;
    }

public:
    void setInputCloud(const PointCloudConstPtr&, const IndicesConstPtr&)
    {
        assert(
            !"This method should not be used with MapOctree! "
             "Use addPoint(...) or addPoints(...) instead to initialize a pointcloud!");
    }
    // TODO: invalidate other pcl::octree::OctreePointCloud point insertion methods

    void addPoint(const PointT& pt, uint64_t stamp = 0);
    void addPoints(
        const pcl::PointCloud<PointT>& pts,
        const pcl::Indices* indices = nullptr);
    void deletePoint(const pcl::index_t pt_idx, bool trim_nodes = false);
    void deletePoints(const pcl::Indices& indices, bool trim_nodes = false);

    uint64_t getPointStamp(pcl::index_t pt_idx);
    void setPointStamp(pcl::index_t pt_idx, uint64_t stamp);

    void crop(const Vec3f& min, const Vec3f& max, bool trim_nodes = true);
    void optimizeStorage();

    // std::atomic<size_t> holes_added{0}, holes_removed{0}, voxel_attempts{0};

protected:
    /* Returns true if the point should be deleted (default always false) */
    static bool mergePointFields(PointT& map_point, const PointT& new_point);

    LeafContainer_T* getOctreePoint(
        const PointT& pt,
        pcl::octree::OctreeKey& key);
    LeafContainer_T* getOrCreateOctreePoint(
        const PointT& pt,
        pcl::octree::OctreeKey& key);

    typename Super_T::PointCloudPtr cloud_buff;
    std::vector<uint64_t> pt_stamps;
    std::vector<size_t> hole_indices;
};

};  // namespace perception
};  // namespace csm





#ifndef MAP_OCTREE_PRECOMPILED

#include "impl/map_octree_impl.hpp"

// clang-format off
#define MAP_OCTREE_INSTANTIATE_CLASS_TEMPLATE(POINT_TYPE, ...)          \
    template class csm::perception::MapOctree<POINT_TYPE __VA_OPT__(, ) \
                                                    __VA_ARGS__>;

#define MAP_OCTREE_INSTANTIATE_PCL_DEPENDENCIES(POINT_TYPE) \
    template class pcl::octree::OctreePointCloudSearch<     \
        POINT_TYPE,                                         \
        csm::perception::MapOctreeLeafT>;
// clang-format on

#endif
