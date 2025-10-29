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

#include <array>
#include <mutex>
#include <memory>
#include <vector>
#include <type_traits>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "geometry.hpp"

#ifndef LFD_USE_ORTHO_PLANE_INTERSECTION
    #define LFD_USE_ORTHO_PLANE_INTERSECTION 1
#endif
#ifndef LFD_PRINT_DEBUG
    #define LFD_PRINT_DEBUG 0
#endif


namespace csm
{
namespace perception
{

enum
{
    // require reflective points for all estimation
    LFD_SEG_REFLECTIVE_ONLY = 0,
    // use plane segmentation to find a ground plane (if not a reflector)
    LFD_ESTIMATE_GROUND_PLANE = 1,
    // attempt to estimate up to two planes using non-reflective points, using a provided up-vector
    LFD_ESTIMATE_MULTIPLE = 2,
    // mask for previous 3 values, which are all mutually exclusive
    LFD_SEG_OPTIONS_MASK = 3,
    // immediately use ground sample to build 3rd plane (if available) rather than using it as a last-resort
    LFD_PREFER_USE_GROUND_SAMPLE = 4
};

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
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
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

#pragma GCC diagnostic pop

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
        const Vec3f* up_vec = nullptr,
        const Vec3f* ground_sample = nullptr);

    size_t calculateReflectiveCentroid(Vec3f& centroid, float& variance);

    /* Configure the detection behavior. The first 3 options are mutually exclusive:
     * > LFD_SEG_REFLECTIVE_ONLY - Strictest mode. Only uses reflective points to detect.
     * > LFD_ESTIMATE_GROUND_PLANE - Attempt to estimate the ground plane using
     *      non-reflective points as long as two reflective planes.
     *      were found.
     * > LFD_ESTIMATE_MULTIPLE - Attempt to use even a single reflective plane
     *      to seed other plane detections using non-reflective points.
     * 
     * > LFD_PREFER_USE_GROUND_SAMPLE - If set, ground samples are immediately used
     *      rather than acting as a last-resort solution. */
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
    inline const PointCloudXYZ& getRedetectPoints() const
    {
        return this->redetect_cloud;
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
    inline const Vec4f& getRedetectBounds() const
    {
        return this->redetect_bounds;
    }

protected:
    uint32_t estimatorConfig() const;
    bool preferUseGroundSamples() const;

    size_t segmentPlanes(
        size_t iter,
        const Vec3f* up_vec,
        const Vec3f* ground_sample,
        bool use_ground_thickness = false);
    void setSegParams(
        size_t iter,
        const Vec3f* up_vec,
        bool use_ground_thickness);

    void estimateBounds_1();
    bool estimateBounds_2();

protected:
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    PointCloudXYZ in_cloud, redetect_cloud;
    pcl::PointIndices seg_indices;
    pcl::Indices refl_selection;
    pcl::ModelCoefficients plane_coeffs;
    std::array<PointCloudXYZ, 4> seg_clouds;
    std::array<PointCloudXYZ::Ptr, 3> seg_cloud_ptrs;
    std::array<Vec4f, 3> seg_planes;
    std::array<Vec3f, 3> plane_centers;
    Vec4f redetect_bounds;

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

        uint32_t detector_config{LFD_ESTIMATE_GROUND_PLANE};
    }  //
    param;
    //
};

};  // namespace perception
};  // namespace csm





#ifndef LFD_PRECOMPILED

    #include "impl/lf_detector_impl.hpp"

// clang-format off
#define LFD_INSTANTIATE_CLASS_TEMPLATE(POINT_TYPE) \
    template class csm::perception::LidarFiducialDetector<POINT_TYPE>;
// clang-format on

#endif
