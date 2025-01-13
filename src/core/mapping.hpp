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

#include <point_def.hpp>    // needs to come before PCL includes when using custom types

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <nano_gicp/nano_gicp.hpp>

#include <map_octree.hpp>
#include <kfc_map.hpp>
#include <util.hpp>
#include <cloud_ops.hpp>


namespace csm
{
namespace perception
{

template<typename PointT>
class FiducialMapOctree :
    public csm::perception::MapOctree<PointT, FiducialMapOctree<PointT>>
{
    static_assert(util::traits::has_reflective<PointT>::value);

    using Super_T = csm::perception::MapOctree<PointT, FiducialMapOctree<PointT>>;
    friend Super_T;

    constexpr static float REFLECTIVE_MIN = 0.8f;

public:
    FiducialMapOctree(const double voxel_res) : Super_T(voxel_res) {}

protected:
    inline static bool mergePointFields(PointT& map_point, const PointT& new_point)
    {
        Super_T::mergePointFields(map_point, new_point);

        return map_point.reflective < REFLECTIVE_MIN;
    }

};


template<
    typename PointT,
    typename CollisionPointT>
using EnvironmentMap =
    csm::perception::KFCMap<
        PointT, csm::perception::MapOctree<PointT>, CollisionPointT >;

template<
    typename PointT,
    typename CollisionPointT>
using FiducialMap =
    csm::perception::KFCMap<
        PointT, csm::perception::FiducialMapOctree<PointT>, CollisionPointT >;


template<typename PointT>
class RetroFiducialDetector
{
    static_assert(util::traits::has_reflective<PointT>::value);

public:
    RetroFiducialDetector() = default;
    ~RetroFiducialDetector() = default;

public:
    void registerScan(const pcl::PointCloud<PointT>& scan)
    {
        if(this->fiducial_map.empty())
        {
            this->fiducial_map = scan;
        }
        else
        {
            this->gicp.setInputTarget(util::wrap_unmanaged(&this->fiducial_map));
            this->gicp.setInputSource(util::wrap_unmanaged(&scan));
            this->gicp.calculateSourceCovariances();
            this->gicp.calculateTargetCovariances();

            this->scratch_cloud.clear();
            this->gicp.align(this->scratch_cloud);

            this->gicp.clearTarget();
            this->gicp.clearSource();

            this->fiducial_map += this->scratch_cloud;
            util::voxel_filter(
                this->fiducial_map,
                this->scratch_cloud,
                Eigen::Vector3f::Constant(0.03) );
            this->fiducial_map.swap(this->scratch_cloud);
        }
    }

    inline const pcl::PointCloud<PointT>& getPoints() const
        { return this->fiducial_map; }

public:
    nano_gicp::NanoGICP<PointT, PointT> gicp;
    pcl::PointCloud<PointT> fiducial_map, scratch_cloud;

};

};
};
