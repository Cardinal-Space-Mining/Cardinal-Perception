/*******************************************************************************
*   Copyright (C) 2024-2026 Cardinal Space Mining Club                         *
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

#include <vector>
#include <limits>
#include <memory>
#include <algorithm>
#include <type_traits>

#include <Eigen/Core>

#include <pcl/types.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/point_tests.h>
#include <pcl/filters/impl/voxel_grid.hpp>
// ^ includes <pcl/common/centroid.h> and
// <boost/sort/spreadsort/integer_sort.hpp> which we use
#include <pcl/filters/impl/morphological_filter.hpp>
// ^ includes <pcl/octree/octree_search.h>
#include <pcl/common/impl/transforms.hpp>

#include "std_utils.hpp"


namespace util
{

#define ASSERT_POINT_HAS_XYZ(x) static_assert(pcl::traits::has_xyz<x>::value);
#define ASSERT_FLOATING_POINT(x)                     \
    static_assert(std::is_floating_point<x>::value);



/** minMaxND<>() alias for getting min/max for x, y, and z */
template<typename PointT = pcl::PointXYZ, typename IntT = pcl::index_t>
inline void minMaxXYZ(
    const pcl::PointCloud<PointT>& cloud,
    Eigen::Vector3f& min,
    Eigen::Vector3f& max,
    const std::vector<IntT>* selection = nullptr)
{
    ASSERT_POINT_HAS_XYZ(PointT)

    min.setConstant(std::numeric_limits<float>::max());
    max.setConstant(std::numeric_limits<float>::min());

    for (size_t idx = 0;; idx++)
    {
        size_t i = idx;
        if (selection)
        {
            if (idx >= selection->size())
            {
                break;
            }
            i = static_cast<size_t>((*selection)[idx]);
        }
        else if (idx >= cloud.points.size())
        {
            break;
        }

        const auto& pt = cloud.points[i].getVector3fMap();
        min = min.cwiseMin(pt);
        max = max.cwiseMax(pt);
    }
}





/** Voxelization static reimpl -- copied from VoxelGrid<>:eapplyFilter() 
  * and simplified. Input and Output clouds CANNOT BE THE SAME! */
template<
    typename PointT = pcl::PointXYZ,
    typename IntT = pcl::index_t,
    bool DownSampleAllData = false>
void voxelFilter(
    const pcl::PointCloud<PointT>& cloud,
    pcl::PointCloud<PointT>& voxelized,
    Eigen::Vector3f leaf_size_,
    const std::vector<IntT>* selection = nullptr,
    unsigned int min_points_per_voxel_ = 0)
{
    ASSERT_POINT_HAS_XYZ(PointT)

    // const Eigen::Vector3f
    //     leaf_size_{ leaf_x, leaf_y, leaf_z };
    const Eigen::Array3f inverse_leaf_size_{
        Eigen::Array3f::Ones() / leaf_size_.array()};

    // Copy the header (and thus the frame_id) + allocate enough space for
    // points
    voxelized.height = 1;       // downsampling breaks the organized structure
    voxelized.is_dense = true;  // we filter out invalid points

    Eigen::Vector3f min_p, max_p;
    util::minMaxXYZ<PointT>(cloud, min_p, max_p, selection);

    // Check that the leaf size is not too small, given the size of the data
    int64_t dx =
        static_cast<int64_t>((max_p[0] - min_p[0]) * inverse_leaf_size_[0]) + 1;
    int64_t dy =
        static_cast<int64_t>((max_p[1] - min_p[1]) * inverse_leaf_size_[1]) + 1;
    int64_t dz =
        static_cast<int64_t>((max_p[2] - min_p[2]) * inverse_leaf_size_[2]) + 1;

    if ((dx * dy * dz) >
        static_cast<int64_t>(std::numeric_limits<int32_t>::max()))
    {
        return;
    }

    Eigen::Vector4i min_b_, max_b_, div_b_, divb_mul_;

    // Compute the minimum and maximum bounding box values
    min_b_[0] = static_cast<int>(std::floor(min_p[0] * inverse_leaf_size_[0]));
    max_b_[0] = static_cast<int>(std::floor(max_p[0] * inverse_leaf_size_[0]));
    min_b_[1] = static_cast<int>(std::floor(min_p[1] * inverse_leaf_size_[1]));
    max_b_[1] = static_cast<int>(std::floor(max_p[1] * inverse_leaf_size_[1]));
    min_b_[2] = static_cast<int>(std::floor(min_p[2] * inverse_leaf_size_[2]));
    max_b_[2] = static_cast<int>(std::floor(max_p[2] * inverse_leaf_size_[2]));

    // Compute the number of divisions needed along all axis
    div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones();
    div_b_[3] = 0;

    // Set up the division multiplier
    divb_mul_ = Eigen::Vector4i{1, div_b_[0], div_b_[0] * div_b_[1], 0};

    // Storage for mapping leaf and pointcloud indexes
    std::vector<cloud_point_index_idx> index_vector;

    // First pass: go over all points and insert them into the index_vector
    // vector with calculated idx. Points with the same idx value will
    // contribute to the same point of resulting CloudPoint
    index_vector.reserve(selection ? selection->size() : cloud.size());
    for (size_t idx = 0;; idx++)
    {
        size_t i = idx;
        if (selection)
        {
            if (idx >= selection->size())
            {
                break;
            }
            i = static_cast<size_t>((*selection)[idx]);
        }
        else if (idx >= cloud.points.size())
        {
            break;
        }

        if (!cloud.is_dense && !pcl::isXYZFinite(cloud[i]))
        {
            continue;
        }

        int ijk0 = static_cast<int>(
            std::floor(cloud[i].x * inverse_leaf_size_[0]) -
            static_cast<float>(min_b_[0]));
        int ijk1 = static_cast<int>(
            std::floor(cloud[i].y * inverse_leaf_size_[1]) -
            static_cast<float>(min_b_[1]));
        int ijk2 = static_cast<int>(
            std::floor(cloud[i].z * inverse_leaf_size_[2]) -
            static_cast<float>(min_b_[2]));

        // Compute the centroid leaf index
        int idx_ =
            (ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2]);
        index_vector.emplace_back(static_cast<unsigned int>(idx_), i);
    }

    // Second pass: sort the index_vector vector using value representing target
    // cell as index in effect all points belonging to the same output cell will
    //  be next to each other
    auto rightshift_func =
        [](const cloud_point_index_idx& x, const unsigned offset)
    { return x.idx >> offset; };
    boost::sort::spreadsort::integer_sort(
        index_vector.begin(),
        index_vector.end(),
        rightshift_func);

    // Third pass: count output cells
    // we need to skip all the same, adjacent idx values
    unsigned int total = 0;
    unsigned int index = 0;
    // first_and_last_indices_vector[i] represents the index in index_vector of
    // the first point in index_vector belonging to the voxel which corresponds
    // to the i-th output point, and of the first point not belonging to.
    std::vector<std::pair<unsigned int, unsigned int> >
        first_and_last_indices_vector;
    // Worst case size
    first_and_last_indices_vector.reserve(index_vector.size());
    while (index < index_vector.size())
    {
        unsigned int i = index + 1;
        for (; (i < index_vector.size() &&
                index_vector[i].idx == index_vector[index].idx);
             ++i)
        {
        }
        if (i - index >= min_points_per_voxel_)
        {
            ++total;
            first_and_last_indices_vector.emplace_back(index, i);
        }
        index = i;
    }

    // Fourth pass: compute centroids, insert them into their final position
    voxelized.resize(total);

    index = 0;
    for (const auto& cp : first_and_last_indices_vector)
    {
        // calculate centroid - sum values from all input points, that have the
        // same idx value in index_vector array
        unsigned int first_index = cp.first;
        unsigned int last_index = cp.second;

        //Limit downsampling to coords
        if constexpr (!DownSampleAllData)
        {
            Eigen::Vector4f centroid{Eigen::Vector4f::Zero()};

            for (unsigned int li = first_index; li < last_index; ++li)
            {
                centroid +=
                    cloud[index_vector[li].cloud_point_index].getVector4fMap();
            }
            centroid /= static_cast<float>(last_index - first_index);
            voxelized[index].getVector4fMap() = centroid;
        }
        else
        {
            pcl::CentroidPoint<PointT> centroid;

            // fill in the accumulator with leaf points
            for (unsigned int li = first_index; li < last_index; ++li)
            {
                centroid.add(cloud[index_vector[li].cloud_point_index]);
            }
            centroid.get(voxelized[index]);
        }
        ++index;
    }
    voxelized.width = voxelized.size();
}





/** Cartesian "crop box" filter reimpl -- copied from 
  * pcl::CropBox<>::applyFilter() and simplified (no extra transforms) */
template<
    typename PointT = pcl::PointXYZ,
    typename IntT = pcl::index_t,
    bool UseNegative = false>
void cropFilter(
    const pcl::PointCloud<PointT>& cloud,
    std::vector<IntT>& filtered,
    const Eigen::Vector3f min_pt,
    const Eigen::Vector3f max_pt,
    const std::vector<IntT>* selection = nullptr)
{
    ASSERT_POINT_HAS_XYZ(PointT)

    filtered.clear();
    // reserve maximum size
    filtered.reserve(selection ? selection->size() : cloud.size());

    for (size_t idx = 0;; idx++)
    {
        size_t i = idx;
        if (selection)
        {
            if (idx >= selection->size())
            {
                break;
            }
            i = static_cast<size_t>((*selection)[idx]);
        }
        else if (idx >= cloud.points.size())
        {
            break;
        }

        const PointT& pt = cloud.points[i];
        if (!cloud.is_dense && !pcl::isFinite(pt))
        {
            continue;
        }

        if ((pt.getArray3fMap() < min_pt.array()).any() ||
            (pt.getArray3fMap() > max_pt.array()).any())
        {
            if constexpr (UseNegative)
            {
                // outside the cropbox --> push on negative
                filtered.push_back(i);
            }
        }
        else if constexpr (!UseNegative)
        {
            // inside the cropbox and not negative
            filtered.push_back(i);
        }
    }
}



template<
    typename PointT = pcl::PointXYZ,
    typename IntT = pcl::index_t,
    typename FloatT = float,
    bool CopyFields = false>
void transformAndFilterNull(
    const pcl::PointCloud<PointT>& cloud_in,
    pcl::PointCloud<PointT>& cloud_out,
    std::vector<IntT>& nan_indices,
    const Eigen::Matrix<FloatT, 4, 4>& transform,
    const std::vector<IntT>* selection = nullptr)
{
    ASSERT_POINT_HAS_XYZ(PointT)

    cloud_out.is_dense = cloud_in.is_dense;
    cloud_out.header = cloud_in.header;
    cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
    cloud_out.sensor_origin_ = cloud_in.sensor_origin_;

    const bool diff_output = &cloud_out.points != &cloud_in.points || selection;
    if (&cloud_out.points != &cloud_in.points)
    {
        cloud_out.points.clear();
        cloud_out.points.resize(
            selection ? selection->size() : cloud_in.points.size());
    }
    nan_indices.clear();

    pcl::detail::Transformer<FloatT> tf{transform};

    for (size_t idx = 0;; idx++)
    {
        size_t i = idx;
        if (selection)
        {
            if (idx >= selection->size())
            {
                break;
            }
            i = static_cast<size_t>((*selection)[idx]);
        }
        else if (idx >= cloud_in.points.size())
        {
            break;
        }

        const PointT& p = cloud_in.points[i];

        if constexpr (CopyFields)
        {
            if (diff_output)
            {
                cloud_out.points[idx] = p;
            }
        }
        if (!pcl::isFinite(p) || (p.x == 0 && p.y == 0 && p.z == 0))
        {
            nan_indices.push_back(i);
            continue;
        }

        tf.se3(p.data, cloud_out.points[idx].data);
    }

    if (diff_output)
    {
        cloud_out.width = cloud_out.points.size();
        cloud_out.height = 1;
    }
}





/** PMF filter reimpl -- See <pcl/filters/progressive_morphological_filter.h> */
template<
    // bool mirror_z = false,
    typename PointT = pcl::PointXYZ,
    typename IntT = pcl::index_t>
void progressiveMorphFilter(
    const pcl::PointCloud<PointT>& cloud_,
    pcl::Indices& ground,
    const float base_,
    const float max_window_size_,
    const float cell_size_,
    const float initial_distance_,
    const float max_distance_,
    const float slope_,
    const bool exponential_,
    const std::vector<IntT>* selection = nullptr)
{
    ASSERT_POINT_HAS_XYZ(PointT)

    // Compute the series of window sizes and height thresholds
    std::vector<float> height_thresholds;
    std::vector<float> window_sizes;

    float window_size = 0.0f;
    float height_threshold = 0.0f;
    for (size_t itr = 0; window_size < max_window_size_; itr++)
    {
        // Determine the initial window size.
        if (exponential_ && base_ >= 1.f)
        {
            // << this becomes an issue when base_ is less than 0
            // since the loop never exits! :O
            window_size = cell_size_ * (2.0f * std::pow(base_, itr) + 1.0f);
        }
        else
        {
            window_size = cell_size_ * (2.0f * (itr + 1) * base_ + 1.0f);
        }

        // Calculate the height threshold to be used in the next iteration.
        if (itr == 0)
        {
            height_threshold = initial_distance_;
        }
        else
        {
            height_threshold =
                (slope_ * (window_size - window_sizes[itr - 1]) * cell_size_ +
                 initial_distance_);
        }

        // Enforce max distance on height threshold
        if (height_threshold > max_distance_)
        {
            height_threshold = max_distance_;
        }

        window_sizes.push_back(window_size);
        height_thresholds.push_back(height_threshold);
    }

    // Ground indices are initially limited to those points in the input cloud
    // we wish to process
    if (selection && selection->size() <= cloud_.size())
    {
        ground = *selection;
    }
    else
    {
        ground.resize(cloud_.size());
        for (size_t i = 0; i < cloud_.size(); i++)
        {
            ground[i] = i;
        }
    }


    const std::shared_ptr<const pcl::PointCloud<PointT> > cloud_shared_ref =
        util::wrapUnmanaged(cloud_);
    const std::shared_ptr<const pcl::Indices> ground_shared_ref =
        util::wrapUnmanaged(ground);

    pcl::octree::OctreePointCloudSearch<PointT> tree{1.f};
    // csm::perception::SelectionOctree<PointT> tree{ window_sizes[0] };
    // tree.initPoints(cloud_shared_ref, ground_shared_ref);

    // reused buffers
    std::vector<pcl::Indices> pt_window_indices{};
    std::vector<float> zp_temp{}, zp_final{}, zn_temp{}, zn_final{};
    zp_temp.resize(cloud_.size());
    zp_final.resize(cloud_.size());
    zn_temp.resize(cloud_.size());
    zn_final.resize(cloud_.size());

    // Progressively filter ground returns using morphological open
    for (size_t i = 0; i < window_sizes.size(); i++)
    {
        // reset tree and reinit to new window size and narrowed selection of
        // points
        tree.deleteTree();
        tree.setResolution(window_sizes[i]);
        tree.setInputCloud(
            cloud_shared_ref,
            ground_shared_ref);  // points in the tree will be in the domain of
                                 // the full cloud
        tree.addPointsFromInputCloud();

        pt_window_indices.resize(ground.size());

        const float half_res = window_sizes[i] / 2.0f;
        // calculate points within each window (for each point in the selection)
        for (size_t idx = 0; idx < ground.size(); idx++)
        {
            const PointT& pt =
                cloud_[ground[idx]];  // retrieve source (x, y) for each pt in
                                      // selection
            tree.boxSearch(
                Eigen::Vector3f{
                    pt.x - half_res,
                    pt.y - half_res,
                    std::numeric_limits<float>::lowest()},
                Eigen::Vector3f{
                    pt.x + half_res,
                    pt.y + half_res,
                    std::numeric_limits<float>::max()},
                pt_window_indices[idx]  // output into the cache
            );
        }

        // morph op stage 1
        for (size_t p_idx = 0; p_idx < ground.size(); p_idx++)
        {
            const pcl::Indices& pt_indices = pt_window_indices[p_idx];
            float& zp_temp_i = zp_temp[ground[p_idx]];
            float& zn_temp_i = zn_temp[ground[p_idx]];
            zp_temp_i = zn_temp_i = cloud_[ground[p_idx]].z;

            for (const pcl::index_t window_idx : pt_indices)
            {
                const float z = cloud_[window_idx].z;

                if (z < zp_temp_i)
                {
                    zp_temp_i = z;
                }
                if (z > zn_temp_i)
                {
                    zn_temp_i = z;
                }
            }
        }

        // morph op stage 2
        for (size_t p_idx = 0; p_idx < ground.size(); p_idx++)
        {
            const pcl::Indices& pt_indices = pt_window_indices[p_idx];
            float& zp_final_i = zp_final[ground[p_idx]];
            float& zn_final_i = zn_final[ground[p_idx]];
            zp_final_i = zp_temp[ground[p_idx]];
            zn_final_i = zn_temp[ground[p_idx]];

            for (const pcl::index_t window_idx : pt_indices)
            {
                const float zp = zp_temp[window_idx], zn = zn_temp[window_idx];

                if (zp > zp_final_i)
                {
                    zp_final_i = zp;
                }
                if (zn < zn_final_i)
                {
                    zn_final_i = zn;
                }
            }
        }

        // Find indices of the points whose difference between the source and
        // filtered point clouds is less than the current height threshold.
        size_t slot = 0;
        for (size_t p_idx = 0; p_idx < ground.size(); p_idx++)
        {
            const float diff_p =
                cloud_[ground[p_idx]].z - zp_final[ground[p_idx]];
            const float diff_n =
                zn_final[ground[p_idx]] - cloud_[ground[p_idx]].z;

            // pt is part of ground
            if (diff_p < height_thresholds[i] && diff_n < height_thresholds[i])
            {
                if (slot != p_idx)
                {
                    // tree.removeIndex(ground[_slot], true);
                    ground[slot] = ground[p_idx];
                }
                slot++;
            }
        }
        ground.resize(slot);
    }
}





/** Generate a set of ranges for each point in the provided cloud */
template<
    typename PointT = pcl::PointXYZ,
    typename AllocT =
        typename pcl::PointCloud<PointT>::VectorType::allocator_type,
    typename IntT = pcl::index_t,
    typename FloatT = float>
void generateRanges(
    const std::vector<PointT, AllocT>& points,
    std::vector<FloatT>& out_ranges,
    const Eigen::Vector3<FloatT> origin = Eigen::Vector3<FloatT>::Zero(),
    const std::vector<IntT>* selection = nullptr)
{
    ASSERT_POINT_HAS_XYZ(PointT)
    ASSERT_FLOATING_POINT(FloatT)

    const Eigen::Vector3f o = origin.template cast<float>();
    out_ranges.resize(selection ? selection->size() : points.size());

    for (size_t idx = 0;; idx++)
    {
        size_t i = idx;
        if (selection)
        {
            if (idx >= selection->size())
            {
                break;
            }
            i = static_cast<size_t>((*selection)[idx]);
        }
        else if (idx >= points.size())
        {
            break;
        }

        out_ranges[i] =
            static_cast<FloatT>((o - points[i].getVector3fMap()).norm());
    }
}

template<
    typename PointT = pcl::PointXYZ,
    typename IntT = pcl::index_t,
    typename FloatT = float>
inline void generateRanges(
    const pcl::PointCloud<PointT>& points,
    std::vector<FloatT>& out_ranges,
    const std::vector<IntT>* selection = nullptr)
{
    util::generateRanges(
        points.points,
        out_ranges,
        Eigen::Vector3<FloatT>{
            points.sensor_origin_.template head<3>().template cast<FloatT>()},
        selection);
}

/** Filter a set of ranges to an inclusive set of indices */
template<typename FloatT = float, typename IntT = pcl::index_t>
void filterRanges(
    const std::vector<FloatT>& ranges,
    std::vector<IntT>& filtered,
    const FloatT min,
    const FloatT max,
    const std::vector<IntT>* selection = nullptr)
{
    ASSERT_FLOATING_POINT(FloatT)

    filtered.clear();
    filtered.reserve(selection ? selection->size() : ranges.size());

    for (size_t idx = 0;; idx++)
    {
        size_t i = idx;
        if (selection)
        {
            if (idx >= selection->size())
            {
                break;
            }
            i = static_cast<size_t>((*selection)[idx]);
        }
        else if (idx >= ranges.size())
        {
            break;
        }

        const FloatT r = ranges[i];
        if (r <= max && r >= min)
        {
            filtered.push_back(i);
        }
    }
}

/** Filter a set of points by their distance from a specified origin point 
  * (<0, 0, 0> by default) */
template<
    typename PointT = pcl::PointXYZ,
    typename AllocT =
        typename pcl::PointCloud<PointT>::VectorType::allocator_type,
    typename IntT = pcl::index_t,
    typename FloatT = float>
void filterDistance(
    const std::vector<PointT, AllocT>& points,
    std::vector<IntT>& filtered,
    const FloatT min,
    const FloatT max,
    const Eigen::Vector3f origin = Eigen::Vector3f::Zero(),
    const std::vector<IntT>* selection = nullptr)
{
    ASSERT_POINT_HAS_XYZ(PointT)
    ASSERT_FLOATING_POINT(FloatT)

    filtered.clear();
    filtered.reserve(selection ? selection->size() : points.size());

    for (size_t idx = 0;; idx++)
    {
        size_t i = idx;
        if (selection)
        {
            if (idx >= selection->size())
            {
                break;
            }
            i = static_cast<size_t>((*selection)[idx]);
        }
        else if (idx >= points.size())
        {
            break;
        }

        const FloatT r =
            static_cast<FloatT>((origin - points[i].getVector3fMap()).norm());
        if (r <= max && r >= min)
        {
            filtered.push_back(i);
        }
    }
}

template<
    typename PointT = pcl::PointXYZ,
    typename IntT = pcl::index_t,
    typename FloatT = float>
inline void filterDistance(
    const pcl::PointCloud<PointT>& points,
    std::vector<IntT>& filtered,
    const FloatT min,
    const FloatT max,
    const std::vector<IntT>* selection = nullptr)
{
    filterDistance(
        points.points,
        filtered,
        min,
        max,
        Eigen::Vector3f{points.sensor_origin_.template head<3>()},
        selection);
}





template<typename IntT = pcl::index_t>
inline void sortIndices(std::vector<IntT>& indices)
{
    std::sort(indices.begin(), indices.end());
}

template<typename IntT = pcl::index_t>
inline void reverseSortIndices(std::vector<IntT>& indices)
{
    std::sort(indices.begin(), indices.end(), std::greater<IntT>{});
}


/** Given a base set of indices A and a subset of indices B, get (A - B).
  * prereq: selection indices must be in ascending order */
template<typename IntT = pcl::index_t>
void negateSelection(
    const std::vector<IntT>& input,
    const std::vector<IntT>& selection,
    std::vector<IntT>& negated)
{
    if (input.size() <= selection.size())
    {
        return;
    }

    negated.resize(input.size() - selection.size());
    size_t base = 0, select = 0, negate = 0;
    for (; base < input.size() && negate < negated.size(); base++)
    {
        if (select < selection.size() && input[base] == selection[select])
        {
            select++;
        }
        else
        {
            negated[negate] = input[base];
            negate++;
        }
    }
}

/** Given a base set of indices A and a subset of indices B, get (A - B).
  * prereq: selection indices must be in ascending order */
template<typename IntT = pcl::index_t>
void negateSelection(
    const IntT input_range,
    const std::vector<IntT>& selection,
    std::vector<IntT>& negated)
{
    if (input_range <= selection.size())
    {
        return;
    }

    negated.resize(input_range - selection.size());
    size_t base = 0, select = 0, negate = 0;
    for (; base < input_range && negate < negated.size(); base++)
    {
        if (select < selection.size() && base == selection[select])
        {
            select++;
        }
        else  // if (base < selection[select])
        {
            negated[negate] = base;
            negate++;
        }
    }
}


/** Merge two presorted selections into a single sorted selection 
  * (non-descending)
  * prereq: both selections must be sorted in non-descending order */
template<typename IntT = pcl::index_t>
void combineSortedSelections(
    const std::vector<IntT>& sel1,
    const std::vector<IntT>& sel2,
    std::vector<IntT>& out)
{
    out.clear();
    out.reserve(sel1.size() + sel2.size());

    size_t _p1 = 0, _p2 = 0;
    while (_p1 < sel1.size() && _p2 < sel2.size())
    {
        const IntT _a = sel1[_p1], _b = sel2[_p2];

        if (_a <= _b)
        {
            out.push_back(_a);
            _p1++;
            _p2 += (_a == _b);
        }
        else
        {
            out.push_back(_b);
            _p2++;
        }
    }
    for (; _p1 < sel1.size(); _p1++)
    {
        out.push_back(sel1[_p1]);
    }
    for (; _p2 < sel2.size(); _p2++)
    {
        out.push_back(sel2[_p2]);
    }
}

/** Remove the points at the each index in the provided set. 
  * Prereq: selection must be valid for the point set and in increasing order! */
template<
    typename PointT = pcl::PointXYZ,
    typename AllocT =
        typename pcl::PointCloud<PointT>::VectorType::allocator_type,
    typename IntT = pcl::index_t>
void removeSelection(
    std::vector<PointT, AllocT>& points,
    const std::vector<IntT>& selection)
{
    size_t last = points.size() - 1;
    for (size_t i = selection.size(); i-- > 0;)
    {
        points[selection[i]] = std::move(points[last]);
        last--;
    }
    points.resize(last + 1);
}

template<typename PointT = pcl::PointXYZ, typename IntT = pcl::index_t>
inline void removeSelection(
    pcl::PointCloud<PointT>& points,
    const std::vector<IntT>& selection)
{
    removeSelection(points.points, selection);
    points.width = points.points.size();
    points.height = 1;
}


/** Normalize the set of points to only include the selected indices. 
  * Prereq: selection indices must be sorted in non-descending order! */
template<
    typename PointT = pcl::PointXYZ,
    typename AllocT =
        typename pcl::PointCloud<PointT>::VectorType::allocator_type,
    typename IntT = pcl::index_t>
inline void trimToSelection(
    std::vector<PointT, AllocT>& points,
    const std::vector<IntT>& selection)
{
    for (size_t i = 0; i < selection.size(); i++)
    {
        points[i] = std::move(points[selection[i]]);
    }
    points.resize(selection.size());
}

template<typename PointT = pcl::PointXYZ, typename IntT = pcl::index_t>
inline void trimToSelection(
    pcl::PointCloud<PointT>& points,
    const std::vector<IntT>& selection)
{
    trimToSelection(points.points, selection);
    points.width = points.points.size();
    points.height = 1;
}


/** Copy a selection of points to another buffer */
template<
    typename PointT = pcl::PointXYZ,
    typename AllocT =
        typename pcl::PointCloud<PointT>::VectorType::allocator_type,
    typename IntT = pcl::index_t>
inline void copySelection(
    const std::vector<PointT, AllocT>& points,
    const std::vector<IntT>& selection,
    std::vector<PointT, AllocT>& buffer)
{
    buffer.resize(selection.size());
    for (size_t i = 0; i < selection.size(); i++)
    {
        buffer[i] = points[selection[i]];
    }
}

template<typename PointT = pcl::PointXYZ, typename IntT = pcl::index_t>
inline void copySelection(
    const pcl::PointCloud<PointT>& points,
    const std::vector<IntT>& selection,
    pcl::PointCloud<PointT>& buffer)
{
    copySelection(points.points, selection, buffer.points);
    buffer.width = buffer.points.size();
    buffer.height = 1;
}


/**  */
template<
    typename PointT = pcl::PointXYZ,
    typename AllocT =
        typename pcl::PointCloud<PointT>::VectorType::allocator_type,
    typename IntT = pcl::index_t>
void copyInverseSelection(
    const std::vector<PointT, AllocT>& points,
    const std::vector<IntT>& selection,
    std::vector<PointT, AllocT>& buffer)
{
    buffer.resize(points.size() - selection.size());
    size_t base = 0, select = 0, negate = 0;
    for (; base < points.size() && negate < buffer.size(); base++)
    {
        if (select < selection.size() &&
            base == static_cast<size_t>(selection[select]))
        {
            select++;
        }
        else  // if (base < selection[select])
        {
            buffer[negate] = points[base];
            negate++;
        }
    }
}

template<typename PointT = pcl::PointXYZ, typename IntT = pcl::index_t>
inline void copyInverseSelection(
    const pcl::PointCloud<PointT>& points,
    const std::vector<IntT>& selection,
    pcl::PointCloud<PointT>& buffer)
{
    copyInverseSelection(points.points, selection, buffer.points);
    buffer.width = buffer.points.size();
    buffer.height = 1;
}

#undef ASSERT_POINT_HAS_XYZ
#undef ASSERT_FLOATING_POINT

};  // namespace util
