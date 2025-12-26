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

#include "../lf_detector.hpp"

#include <pcl/common/intersections.h>
#include <pcl/filters/filter.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>

#include <util/cloud_ops.hpp>
#include <util/std_utils.hpp>


#if LFD_PRINT_DEBUG
    #include <iostream>
    #define DEBUG_COUT(...) std::cout << __VA_ARGS__ << std::endl;
#else
    #define DEBUG_COUT(...)
#endif


namespace csm
{
namespace perception
{

template<typename P>
LidarFiducialDetector<P>::LidarFiducialDetector()
{
    this->seg.setOptimizeCoefficients(true);
    this->seg.setMethodType(pcl::SAC_RANSAC);
    // this->seg.setDistanceThreshold(this->param.plane_seg_thickness);
    // this->seg.setEpsAngle(this->param.plane_eps_angle);

    for (size_t i = 0; i < this->seg_clouds.size(); i++)
    {
        this->seg_cloud_ptrs[i] = util::wrapUnmanaged(this->seg_clouds[i]);
    }
}


template<typename P>
void LidarFiducialDetector<P>::configDetector(uint32_t config)
{
    this->param.detector_config =
        (std::min<uint32_t>(
             (config & LFD_SEG_OPTIONS_MASK),
             LFD_ESTIMATE_MULTIPLE) |
         (config & ~LFD_SEG_OPTIONS_MASK));
}

template<typename P>
uint32_t LidarFiducialDetector<P>::estimatorConfig() const
{
    return this->param.detector_config & LFD_SEG_OPTIONS_MASK;
}
template<typename P>
bool LidarFiducialDetector<P>::preferUseGroundSamples() const
{
    return (this->param.detector_config & LFD_PREFER_USE_GROUND_SAMPLE) > 0;
}

template<typename P>
void LidarFiducialDetector<P>::applyParams(
    double detection_radius,
    double plane_seg_thickness,
    double ground_seg_thickness,
    double up_vec_max_angular_dev,
    double planes_max_angular_dev,
    double vox_filter_res,
    size_t min_num_input_points,
    size_t min_plane_seg_points,
    double max_proportion_leftover)
{
    this->param.detection_radius_sqrd = (detection_radius * detection_radius);
    this->param.plane_seg_thickness = plane_seg_thickness;
    this->param.ground_seg_thickness = ground_seg_thickness;
    this->param.up_vec_max_angular_dev = up_vec_max_angular_dev;
    this->param.planes_max_angular_dev = planes_max_angular_dev;
    this->param.vox_filter_res = vox_filter_res;
    this->param.min_num_input_points = min_num_input_points;
    this->param.min_plane_seg_points = min_plane_seg_points;
    this->param.max_proportion_leftover = max_proportion_leftover;

    // this->seg.setDistanceThreshold(this->param.plane_seg_thickness);
    // this->seg.setEpsAngle(this->param.plane_eps_angle);
}



template<typename P>
typename LidarFiducialDetector<P>::DetectionStatus
    LidarFiducialDetector<P>::calculatePose(
        util::geom::Pose3f& pose,
        const PointCloudT& local_cloud,
        const Vec3f* up_vec,
        const Vec3f* ground_sample)
{
    std::unique_lock lock{this->mtx};
    DetectionStatus status{0};

    DEBUG_COUT("\nLFD: BEGINNING DETECTION ----------------------------------");

    // Instead of storing only reflective points, we keep all points in range,
    // and store reflective subset indices
    this->in_cloud.clear();
    this->in_cloud.reserve(local_cloud.size());
    this->refl_selection.clear();

    this->redetect_cloud.clear();
    for (auto& cloud : this->seg_clouds)
    {
        cloud.clear();
    }
    for (auto& plane : this->seg_planes)
    {
        plane.setZero();
        plane[0] = 1;
    }
    for (auto& center : this->plane_centers)
    {
        center.setZero();
    }

    // filter points in range, and keep track of reflector indices
    pcl::index_t sel = 0;
    for (const PointT& pt : local_cloud)
    {
        if (pt.getVector3fMap().squaredNorm() <=
            this->param.detection_radius_sqrd)
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

    DEBUG_COUT(
        "LFD: using " << this->in_cloud.size() << " out of "
                      << local_cloud.size() << " points, "
                      << this->refl_selection.size()
                      << " filtered as reflective");

    // voxelization will only lower the number of reflective points
    if (this->refl_selection.size() < this->param.min_num_input_points)
    {
        DEBUG_COUT(
            "LFD: failed due to low number (" << this->refl_selection.size()
                                              << ") of reflective points");
        return status;
    }

    // extract reflective points and voxelize them into the first seg buffer
    util::voxelFilter(
        this->in_cloud,
        this->seg_clouds[0],
        Vec3f::Constant(this->param.vox_filter_res),
        &this->refl_selection);

    DEBUG_COUT(
        "LFD: voxelized " << this->refl_selection.size()
                          << " reflective points to "
                          << this->seg_clouds[0].size());

    // check input point num
    const size_t starting_num_points = this->seg_clouds[0].size();
    if (starting_num_points < this->param.min_num_input_points)
    {
        DEBUG_COUT(
            "LFD: failed due to low number ("
            << starting_num_points << ") of voxelized reflective points");
        return status;
    }
    status.has_point_num = true;

    // run plane segmentation
    status.iterations = this->segmentPlanes(0, up_vec, ground_sample);

    // if we found 3 planes, continue as normal, otherwise search for the
    // remaining ones using non-reflective points
    if (status.iterations < 3)
    {
        if (status.iterations == 2 &&
            this->estimatorConfig() == LFD_ESTIMATE_GROUND_PLANE)
        {
            if (!this->estimateBounds_2())
            {
                DEBUG_COUT(
                    "LFD: attempted bounds estimation using 2 segmented refl planes, but failed");

                return status;
            }

            DEBUG_COUT(
                "LFD: completed bounds estimation using 2 segmented refl planes -- center: (x: "
                << this->redetect_bounds[0]
                << ", y: " << this->redetect_bounds[1]
                << ", z: " << this->redetect_bounds[2]
                << "), radius: " << std::sqrt(this->redetect_bounds[3]));
        }
        else if (
            status.iterations == 1 &&
            this->estimatorConfig() == LFD_ESTIMATE_MULTIPLE)
        {
            this->estimateBounds_1();

            DEBUG_COUT(
                "LFD: completed bounds estimation using 1 segmented refl plane -- center: (x: "
                << this->redetect_bounds[0]
                << ", y: " << this->redetect_bounds[1]
                << ", z: " << this->redetect_bounds[2]
                << "), radius: " << std::sqrt(this->redetect_bounds[3]));
        }
        else
        {
            DEBUG_COUT(
                "LFD: insufficient reflective planes ("
                << status.iterations
                << ") and estimator config does not allow redetection - exiting with failure");

            return status;
        }

        size_t refl_selection_idx = 0;
        // this->redetect_cloud.clear();
        this->redetect_cloud.reserve(this->in_cloud.size());
        for (size_t i = 0; i < this->in_cloud.size(); i++)
        {
            if (this->refl_selection[refl_selection_idx] ==
                static_cast<pcl::index_t>(i))
            {
                refl_selection_idx++;
            }
            else if (
                (this->in_cloud[i].getVector3fMap() -
                 this->redetect_bounds.head<3>())
                    .squaredNorm() <= this->redetect_bounds[3])
            {
                this->redetect_cloud.transient_emplace_back(this->in_cloud[i]);
            }
        }

        DEBUG_COUT(
            "LFD: filtered "
            << this->redetect_cloud.points.size()
            << " non-reflective points in estimated range to be used for redetection");

        this->redetect_cloud.height = 1;
        this->redetect_cloud.width = this->redetect_cloud.points.size();
        this->redetect_cloud.is_dense = true;
        this->redetect_cloud += this->seg_clouds[status.iterations];

        DEBUG_COUT(
            "LFD: combined unused reflective points - using "
            << this->redetect_cloud.size() << " total points for redetection");

        util::voxelFilter(
            this->redetect_cloud,
            this->seg_clouds[status.iterations],
            Vec3f::Constant(this->param.vox_filter_res));

        DEBUG_COUT(
            "LFD: voxelized redetection points to "
            << this->seg_clouds[status.iterations].size());

        // 3. search for remaining planes
        status.iterations =
            this->segmentPlanes(status.iterations, up_vec, ground_sample, true);

        if (status.iterations < 3)
        {
            DEBUG_COUT(
                "LFD: redetection found insufficient number of planes ("
                << status.iterations << ") - exiting with failure");

            return status;
        }

        status.has_seg_point_num = true;
        // no longer a meaningful statistic in this case
        status.has_remaining_point_num = true;
    }
    else
    {
        status.has_seg_point_num = true;
        status.has_remaining_point_num =
            ((static_cast<double>(this->seg_clouds[3].size()) /
              static_cast<double>(starting_num_points)) <=
             this->param.max_proportion_leftover);

        DEBUG_COUT(
            "LFD: segmentation found 3 planes with "
            << this->seg_clouds[3].size() << "leftover points - proportion is "
            << status.has_remaining_point_num);
    }

    if (status)
    {
        // x cross y should be in the same direction as grav vec (upwards) - if
        // not then swap
        if (this->seg_planes[0]
                .head<3>()
                .cross(this->seg_planes[1].head<3>())
                .dot(*up_vec) < 0.f)
        {
            std::swap(this->seg_planes[0], this->seg_planes[1]);
            std::swap(this->plane_centers[0], this->plane_centers[1]);
            std::swap(this->seg_clouds[0], this->seg_clouds[1]);

            DEBUG_COUT(
                "LFD: swapped first two planes due to detected inversion");
        }

        Mat3f rotation, hhr;
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

        pose.quat = Quatf{rotation}.inverse();
        pcl::threePlanesIntersection(
            this->seg_planes[0],
            this->seg_planes[1],
            this->seg_planes[2],
            pose.vec);
        pose.vec *= -1.f;

        DEBUG_COUT("LFD: completed successfully");
    }
    else
    {
        DEBUG_COUT("LFD: failed");
    }

    return status;
}



template<typename P>
size_t LidarFiducialDetector<P>::segmentPlanes(
    size_t iter,
    const Vec3f* up_vec,
    const Vec3f* ground_sample,
    bool use_ground_thickness)
{
    size_t completed_iterations = iter;

    DEBUG_COUT(
        "LFD: segmenting planes >>\n\tstart iter: "
        << iter << "\n\tup vector: (x: " << up_vec->x()
        << ", y: " << up_vec->y() << ", z: " << up_vec->z() << ")");

    for (; iter < (2U + !this->preferUseGroundSamples()); iter++)
    {
        if (this->seg_clouds[iter].size() < this->param.min_plane_seg_points)
        {
            DEBUG_COUT(
                "LFD: insufficient input points ("
                << this->seg_clouds[iter].size()
                << ") to begin segment iteration " << iter);

            break;
        }

        DEBUG_COUT(
            "LFD: segment iteration " << iter << " ------------\n\t[using "
                                      << this->seg_clouds[iter].size()
                                      << " points]");

        this->setSegParams(iter, up_vec, use_ground_thickness);

        this->seg_indices.indices.clear();
        this->plane_coeffs.values.clear();
        this->seg.segment(this->seg_indices, this->plane_coeffs);

        if (this->seg_indices.indices.size() >=
            this->param.min_plane_seg_points)
        {
            DEBUG_COUT(
                "LFD: segmented "
                << this->seg_indices.indices.size()
                << " points for plane [a: " << this->plane_coeffs.values[0]
                << ", b: " << this->plane_coeffs.values[1]
                << ", c: " << this->plane_coeffs.values[2]
                << ", d: " << this->plane_coeffs.values[3] << "]");

            // copy the rest of the points to the next seg cloud
            util::copyInverseSelection(
                this->seg_clouds[iter],
                this->seg_indices.indices,
                this->seg_clouds[iter + 1]);

            DEBUG_COUT(
                "LFD: copied " << this->seg_clouds[iter + 1].size()
                               << " points to next seg buffer");

            // each cloud ends up with only the segmented points
            util::trimToSelection(
                this->seg_clouds[iter],
                this->seg_indices.indices);

            if (iter < 2)
            {
                this->seg.setInputCloud(this->seg_cloud_ptrs[iter + 1]);

                DEBUG_COUT(
                    "LFD: set next seg input cloud ("
                    << this->seg_cloud_ptrs[iter + 1]->size() << " points)");
            }

            this->seg_planes[iter] = Vec4f{this->plane_coeffs.values.data()};

            // calculate plane center pos
            this->plane_centers[iter].setZero();
            for (const auto& pt : this->seg_clouds[iter].points)
            {
                this->plane_centers[iter] += pt.getVector3fMap();
            }
            this->plane_centers[iter] /=
                static_cast<float>(this->seg_clouds[iter].points.size());

            DEBUG_COUT(
                "LFD: calculated plane center: (x: "
                << this->plane_centers[iter].x()
                << ", y: " << this->plane_centers[iter].y()
                << ", z: " << this->plane_centers[iter].z() << ")");

            // -d = center pt (vec from origin) dot normal --> if normal points away, invert!
            if (this->seg_planes[iter][3] < 0.f)
            {
                this->seg_planes[iter] *= -1.f;

                DEBUG_COUT(
                    "LFD: negated plane coeffs to orient towards origin");
            }

            completed_iterations++;
        }
        else
        {
            DEBUG_COUT(
                "LFD: segmented insufficient points ("
                << this->seg_indices.indices.size() << ") and returning early");

            break;
        }
    }

    if (completed_iterations == 2 && ground_sample)
    {
        this->plane_centers[2] = *ground_sample;
        this->seg_planes[2].head<3>() =
            this->seg_planes[0].cross3(this->seg_planes[1]).head<3>();
        this->seg_planes[2][3] =
            -ground_sample->dot(this->seg_planes[2].head<3>());

        if (this->seg_planes[2][3] < 0.f)
        {
            this->seg_planes[2] *= -1.f;
        }

        DEBUG_COUT(
            "LFD: segment iteration 2* -----------\n"
            "LFD: set ground plane from provided ground sample point\n"
            "     plane coeffs: [a: "
            << this->seg_planes[2][0] << ", b: " << this->seg_planes[2][1]
            << ", c: " << this->seg_planes[2][2]
            << ", d: " << this->seg_planes[2][3]
            << "]\n     center: (x: " << this->plane_centers[2].x()
            << ", y: " << this->plane_centers[2].y()
            << ", z: " << this->plane_centers[2].z() << ")")

        this->seg_clouds[3].clear();
        if (this->preferUseGroundSamples())
        {
            this->seg_clouds[2].clear();
        }
        else
        {
            this->seg_clouds[3].swap(this->seg_clouds[2]);
        }

        completed_iterations++;
    }

    DEBUG_COUT(
        "LFD: segmentation completed with " << completed_iterations
                                            << " iterations done");

    return completed_iterations;
}

template<typename P>
void LidarFiducialDetector<P>::setSegParams(
    size_t iter,
    const Vec3f* up_vec,
    bool use_ground_thickness)
{
    switch (iter)
    {
        case 0:
        {
            if (up_vec != nullptr)
            {
                this->seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
                this->seg.setEpsAngle(this->param.up_vec_max_angular_dev);
                this->seg.setAxis(*up_vec);

                DEBUG_COUT(
                    "LFD: set model type to SACMODEL_PARALLEL_PLANE, eps angle to "
                    << this->param.up_vec_max_angular_dev
                    << ", axis to (x: " << up_vec->x() << ", y: " << up_vec->y()
                    << ", z: " << up_vec->z() << ")");
            }
            else
            {
                this->seg.setModelType(pcl::SACMODEL_PLANE);

                DEBUG_COUT("LFD: set model type to SACMODEL_PLANE");
            }
            this->seg.setDistanceThreshold(this->param.plane_seg_thickness);
            this->seg.setInputCloud(this->seg_cloud_ptrs[0]);

            DEBUG_COUT(
                "LFD: set distance thresh to "
                << this->param.plane_seg_thickness);

            return;
        }
        case 1:
        {
            this->seg.setEpsAngle(this->param.planes_max_angular_dev);
            if (up_vec != nullptr)
            {
                Vec3f axis = up_vec->cross(this->seg_planes[0].head<3>());

                this->seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
                this->seg.setAxis(axis);

                DEBUG_COUT(
                    "LFD: set model type to SACMODEL_PERPENDICULAR_PLANE, eps angle to "
                    << this->param.planes_max_angular_dev
                    << ", axis to (x: " << axis.x() << ", y: " << axis.y()
                    << ", z: " << axis.z() << ")");
            }
            else
            {
                Vec3f axis = this->seg_planes[0].head<3>();

                this->seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
                this->seg.setAxis(axis);

                DEBUG_COUT(
                    "LFD: set model type to SACMODEL_PARALLEL_PLANE, eps angle to "
                    << this->param.planes_max_angular_dev
                    << ", axis to (x: " << axis.x() << ", y: " << axis.y()
                    << ", z: " << axis.z() << ")");
            }
            return;
        }
        case 2:
        {
            Vec3f axis =
                this->seg_planes[0].cross3(this->seg_planes[1]).head<3>();

            this->seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
            this->seg.setAxis(axis);

            DEBUG_COUT(
                "LFD: set model type to SACMODEL_PERPENDICULAR_PLANE, axis to (x: "
                << axis.x() << ", y: " << axis.y() << ", z: " << axis.z()
                << ")");

            if (use_ground_thickness)
            {
                this->seg.setDistanceThreshold(
                    this->param.ground_seg_thickness);

                DEBUG_COUT(
                    "LFD: set distance threshold to "
                    << this->param.ground_seg_thickness);
            }
            return;
        }
    }
}

template<typename P>
void LidarFiducialDetector<P>::estimateBounds_1()
{
    // find average point distance from plane center,
    // use statistics to estimate plane side length

    float avg_dist = 0.f;
    for (const pcl::PointXYZ& pt : this->seg_clouds[0].points)
    {
        avg_dist += (pt.getVector3fMap() - this->plane_centers[0]).norm();
    }
    avg_dist /= this->seg_clouds[0].points.size();

    // 1 / (2 * 0.3826 ~ avg point distance to side len) >>
    float half_side_len = (avg_dist * 1.3068478f);

    DEBUG_COUT(
        "LFD: bound estimation using 1 plane estimated plane side len to be "
        << (half_side_len * 2));

    this->redetect_bounds.head<3>() =
        (this->plane_centers[0] +
         this->seg_planes[0].head<3>().normalized() * half_side_len);
    this->redetect_bounds[3] =
        half_side_len * half_side_len * 3;  // radius squared
}
template<typename P>
bool LidarFiducialDetector<P>::estimateBounds_2()
{
    // find intersection approximation using plane
    // centers and normal vectors

    const Vec3f& pa = this->plane_centers[0];
    const Vec3f& pb = this->plane_centers[1];
    const Vec3f na = this->seg_planes[0].head<3>().normalized();
    const Vec3f nb = this->seg_planes[1].head<3>().normalized();

    const Vec3f w = pa - pb;

    // float a = na.squaredNorm();
    const float b = na.dot(nb);
    // float c = nb.squaredNorm();
    const float d = na.dot(w);
    const float e = nb.dot(w);

    const float denom = (1.f - b * b);  // (a * c - b * b), a & c = 1

    if (std::abs(denom) > 1e-6f)
    {
        const float s = (b * e - d) / denom;  // (b * e - c * d), c = 1
        const float t = (e - b * d) / denom;  // (a * e - b * d), a = 1

        DEBUG_COUT(
            "LFD: bound estimation using 2 planes estimated plane side lens to be "
            << (t * 2) << ", and " << (s * 2));

        const Vec3f m = pa + na * s;
        const Vec3f n = pb + nb * t;

        this->redetect_bounds.head<3>() = (m + n) / 2;
        this->redetect_bounds[3] =
            (  // s^2 + t^2 + ((s + t)/2)^2 (radius squared)
                1.25f * (s * s) + 1.25f * (t * t) + 0.5f * (s * t));

        return true;
    }
    else
    {
        return false;
    }
}

template<typename P>
size_t LidarFiducialDetector<P>::calculateReflectiveCentroid(
    Vec3f& centroid,
    float& variance)
{
    std::unique_lock lock{this->mtx};

    centroid.setZero();
    variance = 0.f;

    if (this->refl_selection.empty() || in_cloud.empty())
    {
        return 0;
    }

    for (const pcl::index_t& idx : this->refl_selection)
    {
        centroid += this->in_cloud.points[idx].getVector3fMap();
    }
    centroid /= static_cast<float>(this->refl_selection.size());

    for (const pcl::index_t& idx : this->refl_selection)
    {
        variance += (this->in_cloud.points[idx].getVector3fMap() - centroid)
                        .squaredNorm();
    }
    variance /= static_cast<float>(this->refl_selection.size());

    return this->refl_selection.size();
}

};  // namespace perception
};  // namespace csm
