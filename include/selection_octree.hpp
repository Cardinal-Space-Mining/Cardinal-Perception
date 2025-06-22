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

#include <pcl/pcl_config.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/point_tests.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/impl/octree_base.hpp>
#include <pcl/octree/impl/octree_pointcloud.hpp>
#include <pcl/octree/impl/octree_search.hpp>


namespace csm
{
namespace perception
{

template<typename PointT>
class SelectionOctree :
    public pcl::octree::
        OctreePointCloudSearch<PointT, pcl::octree::OctreeContainerPointIndices>
{
    static_assert(pcl::traits::has_xyz<PointT>::value);

    using Super_T = pcl::octree::OctreePointCloudSearch<
        PointT,
        pcl::octree::OctreeContainerPointIndices>;
    using LeafContainer_T = typename Super_T::OctreeT::Base::LeafContainer;

    using typename Super_T::IndicesPtr;
    using typename Super_T::IndicesConstPtr;

    using typename Super_T::PointCloud;
    using typename Super_T::PointCloudPtr;
    using typename Super_T::PointCloudConstPtr;

public:
    inline SelectionOctree(const double voxel_res) : Super_T(voxel_res) {}

    void initPoints(
        const PointCloudConstPtr& cloud,
        const IndicesConstPtr& indices = IndicesConstPtr{});

    void removeIndex(const pcl::index_t pt_idx, bool trim_nodes = false);
    void removeIndices(const pcl::Indices& indices, bool trim_nodes = false);
};




template<typename PointT>
void SelectionOctree<PointT>::initPoints(
    const PointCloudConstPtr& cloud,
    const IndicesConstPtr& indices)
{
    if (this->input_)
    {
        Super_T::
            deleteTree();  // remove old tree
                           // indices get reset when they are copied
    }

    Super_T::setInputCloud(cloud, indices);
    Super_T::addPointsFromInputCloud();
}

template<typename PointT>
void SelectionOctree<PointT>::removeIndex(
    const pcl::index_t pt_idx,
    bool trim_nodes)
{
    const size_t _pt_idx = static_cast<size_t>(pt_idx);
    assert(_pt_idx < this->input_->size());

    const PointT& pt = (*this->input_)[_pt_idx];

    pcl::octree::OctreeKey key;
    this->genOctreeKeyforPoint(pt, key);
    auto* idx = this->findLeaf(key);

    if (!idx || !idx->getSize())
    {
        return;
    }

    pcl::Indices& leaf_pts = idx->getPointIndicesVector();
    for (pcl::index_t& i : leaf_pts)
    {
        if (i == pt_idx)
        {
            i = leaf_pts.back();
            leaf_pts.resize(leaf_pts.size() - 1);
        }
    }

    if (idx->getSize() <= 0 && trim_nodes)
    {
        this->removeLeaf(key);
    }
}

template<typename PointT>
void SelectionOctree<PointT>::removeIndices(
    const pcl::Indices& indices,
    bool trim_nodes)
{
    for (pcl::index_t i : indices)
    {
        this->removeIndex(i, trim_nodes);
    }
}

};  // namespace perception
};  // namespace csm
