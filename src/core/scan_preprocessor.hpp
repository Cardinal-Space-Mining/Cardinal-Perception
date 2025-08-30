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
#include <imu_integrator.hpp>
#include <tsq.hpp>

#define DESKEW_ROT_THRESH_RAD 1e-3


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
        util::traits::has_spherical<SDir_T>::value &&
        util::traits::has_integer_time<PointTime_T>::value);

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

    using TfZoneList = std::pair<Iso3f, std::vector<const Box3f&>>;

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
        int Config_Value =
            (PREPROC_EXCLUDE_ZONES | PREPROC_PERFORM_DESKEW |
             PREPROC_EXPORT_NULL_RAYS),
        typename ImuFloatT = double>
    int process(
        const PointCloudMsg::ConstSharedPtr& scan,
        std::string_view target_frame_id,
        const tf2_ros::Buffer& tf_buffer,
        const ImuIntegrator<ImuFloatT>& imu_samples);

protected:
    template<int Config_Value>
    int computeTfZones(
        const HeaderMsg& scan_header,
        std::string_view target_frame_id,
        const tf2_ros::Buffer& tf_buffer);
    template<int Config_Value>
    int extractPoints(const PointCloudMsg& scan);
    int computeNullRays(const PointCloudMsg& scan);
    template<int Config_Value, typename ImuFloatT>
    int deskewPoints(
        const PointCloudMsg& scan,
        const ImuIntegrator<ImuFloatT>& imu_samples);

protected:
    UoStrMultiMap<Box3f> excl_zones;

    std::vector<TfZoneList> computed_tf_zones;

    PointCloudT point_buff, tf_point_buff;
    SDirCloudT sdir_buff;
    TimeCloudT times_buff;
    std::vector<RayT> null_ray_buff;

    pcl::Indices null_indices;
    pcl::Indices excl_indices;
    pcl::Indices remove_indices;

    std::mutex mtx;
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
template<int Config_Value, typename ImuFloatT>
int ScanPreprocessor<P, S, R, T>::process(
    const PointCloudMsg::ConstSharedPtr& scan,
    std::string_view target_frame_id,
    const tf2_ros::Buffer& tf_buffer,
    const ImuIntegrator<ImuFloatT>& imu_samples)
{
    this->computeTfZones<Config_Value>(
        scan->header,
        target_frame_id,
        tf_buffer);
    this->extractPoints<Config_Value>(*scan);
    if constexpr (Config_Value & PREPROC_EXPORT_NULL_RAYS)
    {
        this->computeNullRays(*scan);
    }
    if constexpr (Config_Value & PREPROC_PERFORM_DESKEW)
    {
        this->deskewPoints<Config_Value, ImuFloatT>(*scan, imu_samples);
    }
}

template<typename P, typename S, typename R, typename T>
template<int Config_Value>
int ScanPreprocessor<P, S, R, T>::computeTfZones(
    const HeaderMsg& scan_header,
    std::string_view target_frame_id,
    const tf2_ros::Buffer& tf_buffer)
{
    auto tf2_tp_stamp = util::toTf2TimePoint(scan_header.stamp);

    auto target_itr_range = this->excl_zones.equal_range(target_frame_id);
    int64_t target_tf_idx = -1;

    this->computed_tf_zones.clear();
    if constexpr (Config_Value & PREPROC_EXCLUDE_ZONES)
    {
        for (auto it = this->excl_zones.begin(); it != this->excl_zones.end();)
        {
            auto range = this->excl_zones.equal_range(it->first);

            try
            {
                auto& zone = this->computed_tf_zones.emplace_back(Iso3f{}, {});
                tf_buffer
                        .lookupTransform(
                            it->first,
                            scan_header.frame_id,
                            tf2_tp_stamp)
                        .transform >>
                    zone.first;

                auto& box_refs = zone.second;
                box_refs.reserve(std::distance(range.first, range.second));
                for (auto val_it = range.first; val_it != range.second;
                     val_it++)
                {
                    box_refs.push_back(val_it->second);
                }

                if (range.first == target_itr_range.first)
                {
                    target_tf_idx = this->computed_tf_zones.size() - 1;
                }
            }
            catch (...)
            {
                this->computed_tf_zones.pop_back();
            }

            it = range.second;
        }
    }

    if (target_tf_idx < 0)
    {
        try
        {
            auto& zone = this->computed_tf_zones.emplace_back(Iso3f{}, {});
            tf_buffer
                    .lookupTransform(
                        target_frame_id,
                        scan_header.frame_id,
                        tf2_tp_stamp)
                    .transform >>
                zone.first;
            target_tf_idx = this->computed_tf_zones.size() - 1;
        }
        catch (...)
        {
            return -1;
        }
    }

    if (target_tf_idx != this->computed_tf_zones.size() - 1)
    {
        std::swap(
            this->computed_tf_zones[target_tf_idx],
            this->computed_tf_zones.back());
    }

    return 0;
}

template<typename P, typename S, typename R, typename T>
template<int Config_Value>
int ScanPreprocessor<P, S, R, T>::extractPoints(const PointCloudMsg& scan)
{
    // convert, resize tf output
    this->point_buff.clear();
    this->tf_point_buff.clear();
    pcl::fromROSMsg(scan, this->point_buff);
    this->tf_point_buff.points.resize(this->point_buff.size());

    this->null_indices.clear();
    this->excl_indices.clear();
    this->remove_indices.clear();

    // loop through all points
    for (size_t i = 0; i < this->point_buff.size(); i++)
    {
        // access input and output
        const auto& in_pt = this->point_buff.points[i];
        auto& out_pt = this->tf_point_buff.points[i];

        // check validity
        if (in_pt.getArray3f().isNaN().any() || in_pt.getVector3f().isZero())
        {
            // zero output and add indices if null
            out_pt.getVector3f().setZero();
            this->null_indices.push_back(i);
            this->remove_indices.push_back(i);
            continue;
        }

        // loop through each zone tf
        for (const auto& zones_tf : this->computed_tf_zones)
        {
            // transform point to output
            out_pt.getVector3f() = zones_tf.first * in_pt.getVector3f();

            bool br = false;
            // check all bounding zones in this frame
            for (const auto& zone : zones_tf.second)
            {
                // if in exclusion zone, add to indices and zero
                if (zone.contains(out_pt.getVector3f()))
                {
                    out_pt.setZero();
                    this->excl_indices.push_back(i);
                    this->remove_indices.push_back(i);
                    br = true;
                    break;
                }
            }
            // don't test other transforms
            if (br)
            {
                break;
            }
        }
    }

    return 0;
}

template<typename P, typename S, typename R, typename T>
int ScanPreprocessor<P, S, R, T>::computeNullRays(const PointCloudMsg& scan)
{
    if (!this->null_indices.empty())
    {
        this->sdir_buff.clear();
        pcl::fromROSMsg(scan, this->sdir_buff);

        this->null_ray_buff.clear();
        this->null_ray_buff.reserve(this->null_indices.size());

        for (auto i : this->null_indices)
        {
            const auto& sdir = this->sdir_buff.points[i];

            const float sin_phi = std::sin(sdir.phi());
            this->null_ray_buff.emplace_back(
                std::cos(sdir.theta()) * sin_phi,
                std::sin(sdir.theta()) * sin_phi,
                std::cos(sdir.phi()));
        }
    }

    return 0;
}

template<typename P, typename S, typename R, typename T>
template<int Config_Value, typename ImuFloatT>
int ScanPreprocessor<P, S, R, T>::deskewPoints(
    const PointCloudMsg& scan,
    const ImuIntegrator<ImuFloatT>& imu_samples)
{
    this->times_buff.clear();
    pcl::fromROSMsg(scan, this->times_buff);

    if (this->times_buff.size() != this->point_buff.size())
    {
        return -1;
    }

    uint64_t min_ts = std::numeric_limits<uint64_t>::max();
    uint64_t max_ts = std::numeric_limits<uint64_t>::min();
    for (const auto& t : this->times_buff)
    {
        if (t.integer_time() < min_ts)
        {
            min_ts = t.integer_time();
        }
        if (t.integer_time() > min_ts)
        {
            max_ts = t.integer_time();
        }
    }

    double ts_diff_nn = static_cast<double>(
        max_ts - min_ts);  // 'non-normalized' (ie. not divided by timebase)
    double beg_range = util::toFloatSeconds(scan->header.stamp);
    double end_range = beg_range + (ts_diff_nn / PointTimeT::time_base());

    using Quat = typename ImuIntegrator<ImuFloatT>::Quat;
    util::tsq::TSQ<Quat> offsets;
    if (imu_samples.getNormalizedOffsets(offsets, beg_range, end_range) &&
        offsets.front().second.angularDistance(offsets.back().second) >=
            DESKEW_ROT_THRESH_RAD)
    {
        size_t skip_i = 0;
        size_t null_i = 0;
        for (size_t i = 0; i < this->point_buff.size(); i++)
        {
            pcl::Vector4fMap v{nullptr};
            if constexpr (Config_Value & PREPROC_EXPORT_NULL_RAYS)
            {
                if (!this->null_indices.empty() &&
                    static_cast<size_t>(this->null_indices[null_i]) == i)
                {
                    new(&v)
                        pcl::Vector4fMap{this->null_ray_buff[null_i].data_n};
                    null_i++;
                    skip_i++;
                    goto evaluate_time;
                }
            }
            if (!this->remove_indices.empty() &&
                static_cast<size_t>(this->remove_indices[skip_i]) == i)
            {
                skip_i++;
                continue;
            }
            else
            {
                new(&v) pcl::Vector4fMap{this->point_buff[i].data};
            }

        evaluate_time:
            double rel_t =
                (static_cast<double>(
                     this->times_buff[i].integer_time() - min_ts) /
                 ts_diff_nn);
            Quat q;
            size_t idx = util::tsq::binarySearchIdx(offsets, rel_t);
            if (offsets[idx].first == rel_t)
            {
                q = offsets[idx].second;
            }
            else
            {
                const auto& a = offsets[idx];
                const auto& b = offsets[idx - 1];

                q = a.second.slerp(
                    (rel_t - a.first) / (b.first - a.first),
                    b.second);
            }
        }
    }

    return 0;
}



};  // namespace perception
};  // namespace csm
