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

#include <mutex>
#include <string>
#include <vector>
#include <utility>
#include <string_view>
#include <unordered_map>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tf2_ros/buffer.h>

#include <pcl_conversions/pcl_conversions.h>

#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <geometry.hpp>
#include <cloud_ops.hpp>
#include <point_def.hpp>


namespace csm
{
namespace perception
{

template<
    typename Point_T,
    typename Ray_T,
    typename SDir_T,
    typename PointTime_T>
class ScanPreprocessor
{
    static_assert(
        pcl::traits::has_xyz<Point_T>::value &&
        pcl::traits::has_normal<Ray_T>::value &&
        util::traits::has_spherical_dir<SDir_T>::value &&
        util::traits::has_time<PointTime_T>::value);

    using PointT = Point_T;
    using RayT = Ray_T;
    using SDirT = SDir_T;
    using PointTimeT = PointTime_T;

    using PointCloudT = pcl::PointCloud<PointT>;
    using SDirCloudT = pcl::PointCloud<SDir_T>;
    using TimeCloudT = pcl::PointCloud<PointTimeT>;

    using HeaderMsg = std_msgs::msg::Header;
    using PointCloudMsg = sensor_msgs::msg::PointCloud2;

    using Box3f = Eigen::AlignedBox3f;
    using Mat4f = Eigen::Matrix4f;
    using Iso3f = Eigen::Isometry3f;

    struct TransparentStringHash
    {
        using is_transparent = void;
        size_t operator()(std::string_view s) const noexcept
        {
            return std::hash<std::string_view>{}(s);
        }
    };
    struct TransparentStringEq
    {
        using is_transparent = void;
        bool operator()(std::string_view a, std::string_view b) const noexcept
        {
            return a == b;
        }
    };

    template<typename T>
    using UoStrMultiMap = std::unordered_multimap<
        std::string,
        T,
        TransparentStringHash,
        TransparentStringEq>;

public:
    enum
    {
        PREPROC_EXCLUDE_ZONES = 0b001,
        PREPROC_PERFORM_DESKEW = 0b010,
        PREPROC_EXPORT_NULL_RAYS = 0b100
    };

public:
    ScanPreprocessor() = default;
    ~ScanPreprocessor() = default;

public:
    void addExclusionZone(std::string_view frame_id, const Box3f& bounds);
    template<
        int ConfigV =
            (PREPROC_EXCLUDE_ZONES | PREPROC_PERFORM_DESKEW |
             PREPROC_EXPORT_NULL_RAYS)>
    int process(
        const PointCloudMsg::ConstSharedPtr& scan,
        const tf2_ros::Buffer& tf_buffer);

protected:
    void computeExclusionIndices(
        const HeaderMsg& src_header,
        const tf2_ros::Buffer& tf_buffer);

protected:
    UoStrMultiMap<Box3f> excl_zones;

    PointCloudT point_buff;
    SDirCloudT sdir_buff;
    TimeCloudT times_buff;
    std::vector<RayT> null_ray_buff;

    pcl::Indices null_indices;
    pcl::Indices excl_indices;
    pcl::Indices remove_indices;
    //
};





template<typename P, typename S, typename R, typename T>
void ScanPreprocessor<P, S, R, T>::addExclusionZone(
    std::string_view frame_id,
    const Box3f& bounds)
{
    this->excl_zones.emplace(frame_id, bounds);
}

template<typename P, typename S, typename R, typename T>
template<int ConfigV>
int ScanPreprocessor<P, S, R, T>::process(
    const PointCloudMsg::ConstSharedPtr& scan,
    const tf2_ros::Buffer& tf_buffer)
{
}

template<typename P, typename S, typename R, typename T>
void ScanPreprocessor<P, S, R, T>::computeExclusionIndices(
    const HeaderMsg& src_header,
    const tf2_ros::Buffer& tf_buffer)
{
    auto tf2_tp_stamp = util::toTf2TimePoint(src_header.stamp);

    std::vector<std::pair<Iso3f, const Box3f&>> zones;
    zones.reserve(this->excl_zones.size());

    for (const auto& val : this->excl_zones)
    {
        zones.emplace_back(Iso3f{}, val.second);
        try
        {
            tf_buffer
                    .lookupTransform(
                        val.first,
                        src_header.frame_id,
                        tf2_tp_stamp)
                    .transform >>
                zones.back().first;
        }
        catch (const std::exception& e)
        {
            // std::cerr << e.what() << '\n';
            zones.pop_back();
        }
    }

    
}

};  // namespace perception
};  // namespace csm
