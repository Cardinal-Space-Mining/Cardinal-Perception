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

#include "point_def.hpp"

#include <vector>
#include <memory>
#include <array>
#include <mutex>
#include <type_traits>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/intersections.h>

#include "pub_map.hpp"
#include "geometry.hpp"
#include "cloud_ops.hpp"
#include "util.hpp"

#ifndef LFD_USE_ORTHO_PLANE_INTERSECTION
    #define LFD_USE_ORTHO_PLANE_INTERSECTION 1
#endif


namespace csm
{
namespace perception
{

/** Containerized detection of fiducials comprised of orthogonal 
  * retro-reflective planes, utilizing LiDAR reflector returns. */
template<typename PointT = csm::perception::PointXYZR>
class LidarFiducialDetector
{
    static_assert(util::traits::has_reflective<PointT>::value);

public:
    LidarFiducialDetector();
    ~LidarFiducialDetector() = default;

public:
    struct DetectionStatus
    {
        union
        {
            struct
            {
                // >= minimum input points (after voxelized)
                bool has_point_num : 1;
                // >= minimum segmented points for ALL planes
                bool has_seg_point_num : 1;
                // <= maximum proportion remaining points
                bool has_remaining_point_num : 1;
                // number of iterations completed
                uint32_t iterations : 2;
            };
            uint64_t data;
        };

        inline DetectionStatus(uint64_t v = 0) : data{v} {}
        inline operator uint64_t() const { return this->data; }
        inline operator bool() const
        {
            return (
                this->has_point_num && this->has_seg_point_num &&
                this->has_remaining_point_num);
        }
    };

public:
    DetectionStatus calculatePose(
        const pcl::PointCloud<PointT>& local_cloud,
        util::geom::Pose3f& pose);
    DetectionStatus calculatePose(
        const pcl::PointCloud<PointT>& local_cloud,
        const Eigen::Vector3f& local_grav,
        util::geom::Pose3f& pose);

    void applyParams(
        double point_range_thresh,
        double plane_dist_thresh,
        double plane_eps_angle,
        double vox_filter_res,
        double avg_center_offset,
        size_t num_points_thresh,
        size_t num_seg_points_thresh,
        double max_leftover_points_rel_thresh);

    inline const pcl::PointCloud<pcl::PointXYZ>& getInputPoints() const
    {
        return this->in_cloud;
    }
    inline const pcl::PointCloud<pcl::PointXYZ>& getRemainingPoints() const
    {
        return this->leftover_cloud;
    }
    inline const std::array<pcl::PointCloud<pcl::PointXYZ>, 3>& getSegClouds()
        const
    {
        return this->seg_clouds;
    }
    inline const std::array<Eigen::Vector4f, 3>& getSegPlanes() const
    {
        return this->seg_planes;
    }
    inline const std::array<Eigen::Vector3f, 3>& getPlaneCenters() const
    {
        return this->plane_centers;
    }

protected:
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointCloud<pcl::PointXYZ> in_cloud, leftover_cloud;
    pcl::PointIndices seg_indices;
    pcl::Indices refl_selection;
    pcl::ModelCoefficients plane_coeffs;
    std::array<pcl::PointCloud<pcl::PointXYZ>, 3> seg_clouds;
    std::array<pcl::PointCloud<pcl::PointXYZ>::Ptr, 3> seg_cloud_ptrs;
    std::array<Eigen::Vector4f, 3> seg_planes;
    std::array<Eigen::Vector3f, 3> plane_centers;

    std::mutex mtx;

    struct
    {
        double point_range_thresh{2.};
        double plane_distance_thresh{0.005};
        double plane_eps_angle{0.1};
        double vox_filter_res{0.03};
        double avg_center_offset{0.4};

        size_t num_points_thresh{100};
        size_t num_seg_points_thresh{15};
        double max_leftover_points_rel_thresh{0.05};
    } param;
};





template<typename PointT>
LidarFiducialDetector<PointT>::LidarFiducialDetector()
{
    this->seg.setOptimizeCoefficients(true);
    this->seg.setMethodType(pcl::SAC_RANSAC);
    this->seg.setDistanceThreshold(this->param.plane_distance_thresh);
    this->seg.setEpsAngle(this->param.plane_eps_angle);

    for (size_t i = 0; i < this->seg_clouds.size(); i++)
    {
        this->seg_cloud_ptrs[i] = util::wrap_unmanaged(this->seg_clouds[i]);
    }
}





template<typename PointT>
void LidarFiducialDetector<PointT>::applyParams(
    double point_range_thresh,
    double plane_dist_thresh,
    double plane_eps_angle,
    double vox_filter_res,
    double avg_center_offset,
    size_t num_points_thresh,
    size_t num_seg_points_thresh,
    double max_leftover_points_rel_thresh)
{
    this->param.point_range_thresh = point_range_thresh;
    this->param.plane_distance_thresh = plane_dist_thresh;
    this->param.plane_eps_angle = plane_eps_angle;
    this->param.vox_filter_res = vox_filter_res;
    this->param.avg_center_offset = avg_center_offset;
    this->param.num_points_thresh = num_points_thresh;
    this->param.num_seg_points_thresh = num_seg_points_thresh;
    this->param.max_leftover_points_rel_thresh = max_leftover_points_rel_thresh;

    this->seg.setDistanceThreshold(this->param.plane_distance_thresh);
    this->seg.setEpsAngle(this->param.plane_eps_angle);
}


template<typename PointT>
typename LidarFiducialDetector<PointT>::DetectionStatus
    LidarFiducialDetector<PointT>::calculatePose(
        const pcl::PointCloud<PointT>& local_cloud,
        util::geom::Pose3f& pose)
{
    std::unique_lock lock{this->mtx};
    DetectionStatus status{0};

    this->in_cloud.clear();
    this->in_cloud.reserve(local_cloud.size());

    const float squared_range_thresh = static_cast<float>(
        this->param.point_range_thresh * this->param.point_range_thresh);
    for (const PointT& pt : local_cloud)
    {
        if (pt.reflective > 0.f &&
            pt.getVector3fMap().squaredNorm() <= squared_range_thresh)
        {
            this->in_cloud.transient_emplace_back(pt.x, pt.y, pt.z);
        }
    }

    this->in_cloud.height = 1;
    this->in_cloud.width = this->in_cloud.points.size();
    this->in_cloud.is_dense = true;

    // reflective points ~ refl_selection
    if (this->refl_selection.size() < this->param.num_points_thresh)
    {
        return status;
    }

    // extract reflective points and voxelize them into the first seg buffer
    util::voxel_filter(
        this->in_cloud,
        this->seg_clouds[0],
        Eigen::Vector3f::Constant(this->param.vox_filter_res),
        &this->refl_selection);

    const size_t starting_num_points = this->seg_clouds[0].size();
    if (starting_num_points < this->param.num_points_thresh)
    {
        return status;
    }

    status.has_point_num = true;

    this->seg.setModelType(pcl::SACMODEL_PLANE);
    this->seg.setInputCloud(this->seg_cloud_ptrs[0]);

    for (size_t iter = 0; iter < 3; iter++)
    {
        this->seg_indices.indices.clear();
        this->plane_coeffs.values.clear();
        this->seg.segment(this->seg_indices, this->plane_coeffs);

        if (this->seg_indices.indices.size() >
            this->param.num_seg_points_thresh)
        {
            if (iter < 2)
            {
                // copy the rest of the points to the next seg cloud
                util::pc_copy_inverse_selection(
                    this->seg_clouds[iter],
                    this->seg_indices.indices,
                    this->seg_clouds[iter + 1]);

                this->seg.setInputCloud(this->seg_cloud_ptrs[iter + 1]);
            }
            else
            {
                util::pc_copy_inverse_selection(
                    this->seg_clouds[iter],
                    this->seg_indices.indices,
                    this->leftover_cloud);
            }

            // each cloud ends up with only the segmented points
            util::pc_normalize_selection(
                this->seg_clouds[iter],
                this->seg_indices.indices);

            this->seg_planes[iter] =
                Eigen::Vector4f{this->plane_coeffs.values.data()};

            switch (iter)
            {
                case 0:
                {
                    this->seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
                    this->seg.setAxis(this->seg_planes[0].head<3>());
                    break;
                }
                case 1:
                {
                    this->seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
                    this->seg.setAxis(this->seg_planes[0]
                                          .cross3(this->seg_planes[1])
                                          .head<3>());
                    break;
                }
            }

            status.iterations++;
        }
        else
        {
            return status;
        }
    }

    status.has_seg_point_num = true;
    status.has_remaining_point_num =
        ((static_cast<double>(this->leftover_cloud.size()) /
          static_cast<double>(starting_num_points)) <=
         this->param.max_leftover_points_rel_thresh);

    if (status)
    {
        // std::array<Eigen::Vector3f, 3> bbox_diffs;
        // Eigen::Vector3f _min, _max;
        // for(size_t i = 0; i < 3; i++)
        // {
        //     util::minMaxXYZ(this->seg_clouds[i], _min, _max);
        //     bbox_diffs[i] = _max - _min;
        // }

        Eigen::Matrix3f rotation, hhr;
        for (size_t i = 0; i < 3; i++)
        {
            this->plane_centers[i] = Eigen::Vector3f::Zero();
            size_t num = 0;
            for (const auto& pt : this->seg_clouds[i].points)
            {
                this->plane_centers[i] += pt.getVector3fMap();
                num++;
            }
            this->plane_centers[i] /= static_cast<float>(num);

            // vector from origin to plane center should be the opposite
            // direction of the normal
            if (this->plane_centers[i].dot(this->seg_planes[i].head<3>()) > 0.f)
            {
                this->seg_planes[i] *= -1.f;
            }

            rotation.block<1, 3>(i, 0) = this->seg_planes[i].head<3>();
        }

        auto hh = rotation.householderQr();
        rotation = hh.householderQ();
        hhr = hh.matrixQR().triangularView<Eigen::Upper>();
        for (size_t i = 0; i < 3; i++)
        {
            if (hhr(i, i) < 0.f)
            {
                rotation.block<3, 1>(0, i) *= -1.f;
            }
        }

#if LFD_USE_ORTHO_PLANE_INTERSECTION > 0
        for (size_t i = 0; i < 3; i++)
        {
            this->seg_planes[i].head<3>() = rotation.block<1, 3>(i, 0);
            this->seg_planes[i][3] =
                this->seg_planes[i].head<3>().dot(this->plane_centers[i]);
            //      ^ ax + by + cz = d --> (a, b, c)*(x, y, z) = d
        }
#endif

        pose.quat = Eigen::Quaternionf{rotation}.inverse();
        pcl::threePlanesIntersection(
            this->seg_planes[0],
            this->seg_planes[1],
            this->seg_planes[2],
            pose.vec);
        pose.vec *= -1.f;
    }

    return status;
}





// #include <iostream>

template<typename PointT>
typename LidarFiducialDetector<PointT>::DetectionStatus
    LidarFiducialDetector<PointT>::calculatePose(
        const pcl::PointCloud<PointT>& local_cloud,
        const Eigen::Vector3f& local_grav,
        util::geom::Pose3f& pose)
{
    std::unique_lock lock{this->mtx};
    DetectionStatus status{0};

    // Instead of storing only reflective points, we keep all points in range,
    // and store reflective subset indices
    this->in_cloud.clear();
    this->in_cloud.reserve(local_cloud.size());
    this->refl_selection.clear();

    const float squared_range_thresh = static_cast<float>(
        this->param.point_range_thresh * this->param.point_range_thresh);

    pcl::index_t sel = 0;
    for (const PointT& pt : local_cloud)
    {
        if (pt.getVector3fMap().squaredNorm() <= squared_range_thresh)
        {
            this->in_cloud.transient_emplace_back(pt.x, pt.y, pt.z);

            if (pt.reflective > 0.f)
            {
                refl_selection.push_back(sel);
            }

            sel++;
        }
    }

    this->in_cloud.height = 1;
    this->in_cloud.width = this->in_cloud.points.size();
    this->in_cloud.is_dense = true;

    // std::cout << "[LDRF(GRAV)] (S1) : " << this->in_cloud.size()
    //     << " points in range, " << this->refl_selection.size()
    //     << " reflective points.\n";

    // reflective points ~ refl_selection
    if (this->refl_selection.size() < this->param.num_points_thresh)
    {
        return status;
    }

    // extract reflective points and voxelize them into the first seg buffer
    util::voxel_filter(
        this->in_cloud,
        this->seg_clouds[0],
        Eigen::Vector3f::Constant(this->param.vox_filter_res),
        &this->refl_selection);

    // std::cout << "[LDRF(GRAV)] (S2) : " << this->seg_clouds[0].size()
    //           << " voxelized points.\n";

    const size_t starting_num_points = this->seg_clouds[0].size();
    if (starting_num_points < this->param.num_points_thresh)
    {
        return status;
    }

    status.has_point_num = true;

    this->seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
    this->seg.setInputCloud(this->seg_cloud_ptrs[0]);
    this->seg.setAxis(local_grav);

    for (size_t iter = 0; iter < 3; iter++)
    {
        this->seg_indices.indices.clear();
        this->plane_coeffs.values.clear();
        this->seg.segment(this->seg_indices, this->plane_coeffs);

        if (this->seg_indices.indices.size() >
            this->param.num_seg_points_thresh)
        {
            if (iter < 2)
            {
                // copy the rest of the points to the next seg cloud
                util::pc_copy_inverse_selection(
                    this->seg_clouds[iter],
                    this->seg_indices.indices,
                    this->seg_clouds[iter + 1]);

                this->seg.setInputCloud(this->seg_cloud_ptrs[iter + 1]);
            }
            else
            {
                util::pc_copy_inverse_selection(
                    this->seg_clouds[iter],
                    this->seg_indices.indices,
                    this->leftover_cloud);
            }

            // each cloud ends up with only the segmented points
            util::pc_normalize_selection(
                this->seg_clouds[iter],
                this->seg_indices.indices);

            this->seg_planes[iter] =
                Eigen::Vector4f{this->plane_coeffs.values.data()};

            // std::cout << "[LDRF(GRAV)] (S3; I" << iter
            //     << ") : segmented " << this->seg_indices.indices.size()
            //     << " points into plane. ";

            {
                this->plane_centers[iter].setZero();
                for (const auto& pt : this->seg_clouds[iter].points)
                {
                    this->plane_centers[iter] += pt.getVector3fMap();
                }
                this->plane_centers[iter] /=
                    static_cast<float>(this->seg_clouds[iter].points.size());

                // vector from origin to plane center should be the opposite
                // direction of the normal
                if (this->plane_centers[iter].dot(
                        this->seg_planes[iter].head<3>()) > 0.f)
                {
                    this->seg_planes[iter] *= -1.f;
                    // std::cout << " <INVERTED>";
                }
                // std::cout << std::endl;
            }

            switch (iter)
            {
                case 0:
                {
                    this->seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
                    this->seg.setAxis(
                        local_grav.cross(this->seg_planes[0].head<3>()));
                    break;
                }
                case 1:
                {
                    this->seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
                    this->seg.setAxis(this->seg_planes[0]
                                          .cross3(this->seg_planes[1])
                                          .head<3>());
                    break;
                }
            }

            status.iterations++;
        }
        else if (iter < 1)
        {
            // std::cout << "[LDRF(GRAV)] (S3; I" << iter
            //           << ") : no planes could be segmented." << std::endl;

            return status;
        }
        else
        {
            // std::cout << "[LDRF(GRAV)] (S3; I" << iter
            //           << ") : failed to segment plane - continuing...\n";

            break;
        }
    }

    // if we found 3 planes, continue as normal, otherwise search for the
    // remaining ones using non-reflective points
    if (status.iterations < 3)
    {
        // 1. compute "center of activity" --> radius search on non-refl points
        Eigen::Vector3f fid_center = Eigen::Vector3f::Zero();
        for (uint32_t i = 0; i < status.iterations; i++)
        {
            fid_center +=
                (this->plane_centers[i] +
                 this->seg_planes[i].head<3>().normalized() *
                     this->param.avg_center_offset);
        }
        fid_center /= static_cast<float>(status.iterations);

        // std::cout << "[LDRF(GRAV)] (S4) : estimated fiducial center.\n";

        // 2. combine remaining refl points with radius search
        const double squared_center_offset =
            this->param.avg_center_offset * this->param.avg_center_offset * 3.;

        size_t refl_selection_idx = 0;
        size_t back_idx = 0;
        for (pcl::index_t i = 0; static_cast<size_t>(i) < this->in_cloud.size();
             i++)
        {
            if (this->refl_selection[refl_selection_idx] == i)
            {
                refl_selection_idx++;
            }
            else if (
                (this->in_cloud[i].getVector3fMap() - fid_center)
                    .squaredNorm() <= squared_center_offset)
            {
                this->in_cloud[back_idx] = this->in_cloud[i];
                back_idx++;
            }
        }
        this->in_cloud.points.resize(back_idx);
        this->in_cloud.height = 1;
        this->in_cloud.width = this->in_cloud.points.size();
        this->in_cloud.is_dense = true;
        this->in_cloud += this->seg_clouds[status.iterations];

        // std::cout << "[LDRF(GRAV)] (S5) : combined " << this->in_cloud.size()
        //           << " remaining points in fiducial range\n";

        util::voxel_filter(
            this->in_cloud,
            this->seg_clouds[status.iterations],
            Eigen::Vector3f::Constant(this->param.vox_filter_res));

        // std::cout << "[LDRF(GRAV)] (S6) : voxelized "
        //           << this->seg_clouds[status.iterations].size()
        //           << " points in fiducial range\n";

        // 3. search for remaining planes
        for (size_t iter = status.iterations; iter < 3; iter++)
        {
            this->seg_indices.indices.clear();
            this->plane_coeffs.values.clear();
            this->seg.segment(this->seg_indices, this->plane_coeffs);

            if (this->seg_indices.indices.size() >
                this->param.num_seg_points_thresh)
            {
                this->seg_planes[iter] =
                    Eigen::Vector4f{this->plane_coeffs.values.data()};

                if (iter < 2)
                {
                    // copy the rest of the points to the next seg cloud
                    util::pc_copy_inverse_selection(
                        this->seg_clouds[iter],
                        this->seg_indices.indices,
                        this->seg_clouds[iter + 1]);

                    this->seg.setInputCloud(this->seg_cloud_ptrs[iter + 1]);

                    this->seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
                    this->seg.setAxis(this->seg_planes[0]
                                          .cross3(this->seg_planes[1])
                                          .head<3>());
                }
                else
                {
                    util::pc_copy_inverse_selection(
                        this->seg_clouds[iter],
                        this->seg_indices.indices,
                        this->leftover_cloud);
                }

                // each cloud ends up with only the segmented points
                util::pc_normalize_selection(
                    this->seg_clouds[iter],
                    this->seg_indices.indices);

                // std::cout << "[LDRF(GRAV)] (S7; I" << iter
                //     << ") : segmented " << this->seg_indices.indices.size()
                //     << " points into plane.";

                {
                    this->plane_centers[iter].setZero();
                    for (const auto& pt : this->seg_clouds[iter].points)
                    {
                        this->plane_centers[iter] += pt.getVector3fMap();
                    }
                    this->plane_centers[iter] /= static_cast<float>(
                        this->seg_clouds[iter].points.size());

                    // vector from origin to plane center should be the opposite
                    // direction of the normal
                    if (this->plane_centers[iter].dot(
                            this->seg_planes[iter].head<3>()) > 0.f)
                    {
                        this->seg_planes[iter] *= -1.f;
                        // std::cout << " <INVERTED>";
                    }
                    // std::cout << std::endl;
                }

                status.iterations++;
            }
            else
            {
                return status;
            }
        }

        status.has_seg_point_num = true;
        status.has_remaining_point_num =
            true;  // no longer a meaningful statistic in this case
    }
    else
    {
        status.has_seg_point_num = true;
        status.has_remaining_point_num =
            ((static_cast<double>(this->leftover_cloud.size()) /
              static_cast<double>(starting_num_points)) <=
             this->param.max_leftover_points_rel_thresh);
    }

    // std::cout << "[LDRF(GRAV)] (S8) : completed plane segmentation"
    //           << std::endl;
    if (status)
    {
        // x cross y should be in the same direction as grav vec (upwards) - if
        // not then swap
        if (this->seg_planes[0]
                .head<3>()
                .cross(this->seg_planes[1].head<3>())
                .dot(local_grav) < 0.f)
        {
            std::swap(this->seg_planes[0], this->seg_planes[1]);
            std::swap(this->plane_centers[0], this->plane_centers[1]);
            std::swap(this->seg_clouds[0], this->seg_clouds[1]);

            // std::cout << "[LDRF(GRAV)] (S9) : swapped x and y planes\n";
        }

        Eigen::Matrix3f rotation, hhr;
        for (size_t i = 0; i < 3; i++)
        {
            rotation.block<1, 3>(i, 0) = this->seg_planes[i].head<3>();
        }

        auto hh = rotation.householderQr();
        rotation = hh.householderQ();
        hhr = hh.matrixQR().triangularView<Eigen::Upper>();
        for (size_t i = 0; i < 3; i++)
        {
            if (hhr(i, i) < 0.f)
            {
                rotation.block<3, 1>(0, i) *= -1.f;
            }
        }

#if LFD_USE_ORTHO_PLANE_INTERSECTION > 0
        for (size_t i = 0; i < 3; i++)
        {
            this->seg_planes[i].head<3>() = rotation.block<1, 3>(i, 0);
            this->seg_planes[i][3] =
                this->seg_planes[i].head<3>().dot(this->plane_centers[i]);
            //      ^ ax + by + cz = d --> (a, b, c)*(x, y, z) = d
        }
#endif

        pose.quat = Eigen::Quaternionf{rotation}.inverse();
        pcl::threePlanesIntersection(
            this->seg_planes[0],
            this->seg_planes[1],
            this->seg_planes[2],
            pose.vec);
        pose.vec *= -1.f;

        // std::cout << "[LDRF(GRAV)] (S10) : computed pose\n" << std::endl;
    }

    return status;
}

};  // namespace perception
};  // namespace csm
