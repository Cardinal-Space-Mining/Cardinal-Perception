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
#include <pcl/ModelCoefficients.h>
#include <pcl/common/intersections.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/filter.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>

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
template<typename Point_T = csm::perception::PointXYZR>
class LidarFiducialDetector
{
    static_assert(util::traits::has_reflective<Point_T>::value);

    using PointT = Point_T;
    using PointCloudT = pcl::PointCloud<PointT>;
    using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;

    using Vec3f = Eigen::Vector3f;
    using Vec4f = Eigen::Vector4f;
    using Mat3f = Eigen::Matrix3f;
    using Quatf = Eigen::Quaternionf;

public:
    enum
    {
        // require reflective planes, do not use estimation
        LFD_REFLECTIVE_ONLY = 0,
        // use plane segmentation to find a ground plane (if not a reflector)
        LFD_ESTIMATE_GROUND_PLANE = 1,
        // attempt to estimate up to two planes using non-reflective points, using a provided up-vector
        LFD_ESTIMATE_MULTIPLE = 2
    };

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
    LidarFiducialDetector();
    ~LidarFiducialDetector() = default;

public:
    /* Attempt to detect a fiducial from the provided point cloud.
     * @param pose The exported pose of the fiducial in the same coordinate
            frame as the input point cloud. This can alternatively be
            used as the transformation from the fiducial's coordinate
            frame to the point cloud's coordinate frame.
     * @param local_cloud The input point cloud.
     * @param local_grav An estimate normal vector for the ground
     *      (ie. gravity vector). This is optional, but significantly
     *      increases detection reliability when provided.
     * @return A status struct which encodes success or what failure
     *      was encountered (can be cast to bool to check success). */
    DetectionStatus calculatePose(
        util::geom::Pose3f& pose,
        const PointCloudT& local_cloud,
        const Vec3f& up_vec = Vec3f::Zero());

    /* Configure the detection behavior. Options are:
     * 1. LFD_REFLECTIVE_ONLY - Strictest mode. Only uses reflective points to detect.
     * 2. LFD_ESTIMATE_GROUND_PLANE - Attempt to estimate the ground plane using
     *      non-reflective points as long as two reflective planes.
     *      were found.
     * 3. LFD_ESTIMATE_MULTIPLE - Attempt to use even a single reflective plane
     *      to seed other plane detections using non-reflective points. */
    void configDetector(uint32_t config);

    /* NOTE: For all linear (distance) parameters, use the same units that
     * are used for the input point cloud!
     * @param detection_radius The range which will be searched
     * @param plane_seg_thickness The thickness for each plane segmentation
     *      which determines which points get included
     * @param ground_seg_thickness The thickness used for segmenting the
     *      ground plane when non-reflective points are used
     * @param up_vec_max_angular_dev The maximum angle offset in radians that
     *      the first plane detection can be when seeded by an up vector
     * @param planes_max_angular_dev The maximum angle offset in radians that
     *      planes seeded from each other can deviate (from being perpendicular)
     * @param vox_filter_res The cell size used to voxelize the input cloud
     * @param min_num_input_points The minimum number of reflective points
     *      needed to proceed with detection
     * @param min_plane_seg_points The minimum number of segmented points
     *      needed for a plane detection to be used
     * @param max_proportion_leftover The maximum proportion of
     *      unsegmented reflective points to total reflective points
     *      which is allowed without invalidating the detection */
    void applyParams(
        double detection_radius,
        double plane_seg_thickness,
        double ground_seg_thickness,
        double up_vec_max_angular_dev,
        double planes_max_angular_dev,
        double vox_filter_res,
        size_t min_num_input_points,
        size_t min_plane_seg_points,
        double max_proportion_leftover);

    inline const PointCloudXYZ& getInputPoints() const
    {
        return this->in_cloud;
    }
    inline const PointCloudXYZ& getRemainingPoints() const
    {
        return this->seg_clouds[3];
    }
    inline const std::array<PointCloudXYZ, 4>& getSegClouds() const
    {
        return this->seg_clouds;
    }
    inline const std::array<Vec4f, 3>& getSegPlanes() const
    {
        return this->seg_planes;
    }
    inline const std::array<Vec3f, 3>& getPlaneCenters() const
    {
        return this->plane_centers;
    }

protected:
    size_t segmentPlanes(
        size_t iter,
        const Vec3f& up_vec,
        bool use_ground_thickness = false);

    Vec4f estimateBounds_1();
    Vec4f estimateBounds_2();

protected:
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    PointCloudXYZ in_cloud;
    pcl::PointIndices seg_indices;
    pcl::Indices refl_selection;
    pcl::ModelCoefficients plane_coeffs;
    std::array<PointCloudXYZ, 4> seg_clouds;
    std::array<PointCloudXYZ::Ptr, 3> seg_cloud_ptrs;
    std::array<Vec4f, 3> seg_planes;
    std::array<Vec3f, 3> plane_centers;

    std::mutex mtx;

    struct
    {
        double detection_radius_sqrd{(2 * 2)};
        double plane_seg_thickness{0.01};
        double ground_seg_thickness{0.02};
        double up_vec_max_angular_dev{0.2};
        double planes_max_angular_dev{0.1};
        double vox_filter_res{0.03};

        size_t min_num_input_points{100};
        size_t min_plane_seg_points{15};
        double max_proportion_leftover{0.05};

        uint32_t estimation_config{LFD_ESTIMATE_GROUND_PLANE};
    }  //
    param;
    //
};





#ifndef LFD_PRECOMPILED

template<typename P>
LidarFiducialDetector<P>::LidarFiducialDetector()
{
    this->seg.setOptimizeCoefficients(true);
    this->seg.setMethodType(pcl::SAC_RANSAC);
    // this->seg.setDistanceThreshold(this->param.plane_seg_thickness);
    // this->seg.setEpsAngle(this->param.plane_eps_angle);

    for (size_t i = 0; i < this->seg_clouds.size(); i++)
    {
        this->seg_cloud_ptrs[i] = util::wrap_unmanaged(this->seg_clouds[i]);
    }
}


template<typename P>
void LidarFiducialDetector<P>::configDetector(uint32_t config)
{
    this->param.estimation_config =
        std::max<uint32_t>(config, LFD_ESTIMATE_MULTIPLE);
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



// #include <iostream>

template<typename P>
typename LidarFiducialDetector<P>::DetectionStatus
    LidarFiducialDetector<P>::calculatePose(
        util::geom::Pose3f& pose,
        const PointCloudT& local_cloud,
        const Vec3f& up_vec)
{
    std::unique_lock lock{this->mtx};
    DetectionStatus status{0};

    // Instead of storing only reflective points, we keep all points in range,
    // and store reflective subset indices
    this->in_cloud.clear();
    this->in_cloud.reserve(local_cloud.size());
    this->refl_selection.clear();

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

    // voxelization will only lower the number of reflective points
    if (this->refl_selection.size() < this->param.min_num_input_points)
    {
        return status;
    }

    // extract reflective points and voxelize them into the first seg buffer
    util::voxel_filter(
        this->in_cloud,
        this->seg_clouds[0],
        Vec3f::Constant(this->param.vox_filter_res),
        &this->refl_selection);

    // check input point num
    const size_t starting_num_points = this->seg_clouds[0].size();
    if (starting_num_points < this->param.min_num_input_points)
    {
        return status;
    }
    status.has_point_num = true;

    // run plane segmentation
    status.iterations = this->segmentPlanes(0, up_vec);

    // if we found 3 planes, continue as normal, otherwise search for the
    // remaining ones using non-reflective points
    if (status.iterations < 3)
    {
        Vec4f bound_estimate;

        if (status.iterations == 2 &&
            this->param.estimation_config == LFD_ESTIMATE_GROUND_PLANE)
        {
            bound_estimate = this->estimateBounds_2();
        }
        else if (
            status.iterations == 1 &&
            this->param.estimation_config == LFD_ESTIMATE_MULTIPLE)
        {
            bound_estimate = this->estimateBounds_1();
        }
        else
        {
            return status;
        }

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
                (this->in_cloud[i].getVector3fMap() - bound_estimate.head<3>())
                    .squaredNorm() <= bound_estimate[3])
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

        util::voxel_filter(
            this->in_cloud,
            this->seg_clouds[status.iterations],
            Vec3f::Constant(this->param.vox_filter_res));

        // 3. search for remaining planes
        status.iterations =
            this->segmentPlanes(status.iterations, up_vec, true);

        if (status.iterations < 3)
        {
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
    }

    if (status)
    {
        // x cross y should be in the same direction as grav vec (upwards) - if
        // not then swap
        if (this->seg_planes[0]
                .head<3>()
                .cross(this->seg_planes[1].head<3>())
                .dot(up_vec) < 0.f)
        {
            std::swap(this->seg_planes[0], this->seg_planes[1]);
            std::swap(this->plane_centers[0], this->plane_centers[1]);
            std::swap(this->seg_clouds[0], this->seg_clouds[1]);
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
    }

    return status;
}



template<typename P>
size_t LidarFiducialDetector<P>::segmentPlanes(
    size_t iter,
    const Vec3f& up_vec,
    bool use_ground_thickness)
{
    const bool use_up_vec = (up_vec != Vec3f::Zero());
    size_t completed_iterations = iter;

    if (iter == 0)
    {
        if (use_up_vec)
        {
            this->seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
            this->seg.setEpsAngle(this->param.up_vec_max_angular_dev);
            this->seg.setAxis(up_vec);
        }
        else
        {
            this->seg.setModelType(pcl::SACMODEL_PLANE);
        }
        this->seg.setDistanceThreshold(this->param.plane_seg_thickness);
        this->seg.setInputCloud(this->seg_cloud_ptrs[0]);
    }

    for (; iter < 3; iter++)
    {
        this->seg_indices.indices.clear();
        this->plane_coeffs.values.clear();
        this->seg.segment(this->seg_indices, this->plane_coeffs);

        if (this->seg_indices.indices.size() > this->param.min_plane_seg_points)
        {
            // copy the rest of the points to the next seg cloud
            util::pc_copy_inverse_selection(
                this->seg_clouds[iter],
                this->seg_indices.indices,
                this->seg_clouds[iter + 1]);

            // each cloud ends up with only the segmented points
            util::pc_normalize_selection(
                this->seg_clouds[iter],
                this->seg_indices.indices);

            if (iter < 2)
            {
                this->seg.setInputCloud(this->seg_cloud_ptrs[iter + 1]);
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

            // vector from origin to plane center should be the opposite
            // direction of the normal
            if (this->plane_centers[iter].dot(
                    this->seg_planes[iter].head<3>()) > 0.f)
            {
                this->seg_planes[iter] *= -1.f;
            }

            // setup next segmentation depending on iteration and parameters
            switch (iter)
            {
                case 0:
                {
                    this->seg.setEpsAngle(this->param.planes_max_angular_dev);
                    if (use_up_vec)
                    {
                        this->seg.setModelType(
                            pcl::SACMODEL_PERPENDICULAR_PLANE);
                        this->seg.setAxis(
                            up_vec.cross(this->seg_planes[0].head<3>()));
                    }
                    else
                    {
                        this->seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
                        this->seg.setAxis(this->seg_planes[0].head<3>());
                    }
                    break;
                }
                case 1:
                {
                    this->seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
                    this->seg.setAxis(this->seg_planes[0]
                                          .cross3(this->seg_planes[1])
                                          .head<3>());
                    if (use_ground_thickness)
                    {
                        this->seg.setDistanceThreshold(
                            this->param.ground_seg_thickness);
                    }
                    break;
                }
            }

            completed_iterations++;
        }
        else
        {
            break;
        }
    }

    return completed_iterations;
}

template<typename P>
typename LidarFiducialDetector<P>::Vec4f
    LidarFiducialDetector<P>::estimateBounds_1()
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

    Vec4f v;
    v.head<3>() =
        (this->plane_centers[0] +
         this->seg_planes[0].head<3>().normalized() * half_side_len);
    v[3] = half_side_len * half_side_len * 3;  // radius squared

    return v;
}
template<typename P>
typename LidarFiducialDetector<P>::Vec4f
    LidarFiducialDetector<P>::estimateBounds_2()
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

        const Vec3f m = pa + na * s;
        const Vec3f n = pb + nb * t;

        Vec4f v;
        v.head<3>() = (m + n) / 2;
        v[3] =
            (  // s^2 + t^2 + ((s + t)/2)^2 (radius squared)
                1.25f * (s * s) + 1.25f * (t * t) + 0.5f * (s * t));

        return v;
    }
    else
    {
        return Vec4f::Zero();
    }
}



// clang-format off
#define LFD_INSTANTIATE_CLASS_TEMPLATE(POINT_TYPE) \
    template class csm::perception::LidarFiducialDetector<POINT_TYPE>;
// clang-format on

#endif

};  // namespace perception
};  // namespace csm
