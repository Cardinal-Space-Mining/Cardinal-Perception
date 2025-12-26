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

#include <config.hpp>

#include <atomic>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <modules/kfc_map.hpp>
#include <modules/map_octree.hpp>

#include <util/pub_map.hpp>
#include <util/synchronization.hpp>

#include "shared_resources.hpp"
#include "../perception_presets.hpp"


namespace csm
{
namespace perception
{

class MappingWorker
{
    friend class PerceptionNode;

    using RclNode = rclcpp::Node;

    template<typename PointT, typename CollisionPointT>
    using SparseMap = KFCMap<
        PointT,
        MapOctree<PointT, MAP_OCTREE_STORE_NORMALS>,
        CollisionPointT>;

public:
    MappingWorker(RclNode& node);
    ~MappingWorker();

public:
    void configure(
        const std::string& odom_frame,
        double map_crop_horizontal_range,
        double map_crop_vertical_range,
        double map_export_horizontal_range,
        double map_export_vertical_range);

    util::ResourcePipeline<MappingResources>& getInput();
    void connectOutput(
        util::ResourcePipeline<TraversibilityResources>&
            traversibility_resources);

    void startThreads();
    void stopThreads();

protected:
    void mapping_thread_worker();
    void mapping_callback(MappingResources& buff);

protected:
    RclNode& node;
    util::GenericPubMap pub_map;

    std::string odom_frame;
    double map_crop_horizontal_range{0.};
    double map_crop_vertical_range{0.};
    double map_export_horizontal_range{0.};
    double map_export_vertical_range{0.};

    std::atomic<bool> threads_running{false};

    SparseMap<MappingPointType, CollisionPointType> sparse_map;
    util::ResourcePipeline<MappingResources> mapping_resources;
    util::ResourcePipeline<TraversibilityResources>* traversibility_resources{
        nullptr};
    std::thread mapping_thread;
};

};  // namespace perception
};  // namespace csm
