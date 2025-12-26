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

#include <set>
#include <mutex>
#include <limits>
#include <vector>
#include <type_traits>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <point_def.hpp>

#include "map_octree.hpp"

#ifndef KFC_MAP_STORE_INSTANCE_BUFFERS
    #define KFC_MAP_STORE_INSTANCE_BUFFERS 1
#endif


namespace csm
{
namespace perception
{

/** KFC Collision Modifiers
 1. KF_COLLISION_MODEL_USE_RADIAL - Instead of using the default conical 
        projection for computing collisions, only considers points a certain
        radial distance away from each lidar ray (producing a cylindrical 
        collision zone). This is more physically accurate and provides better
        mapping but comes with a slight performance hit.
 2. KF_COLLISION_MODEL_USE_DIFF - Enables the requirement for lidar points to be
        a certain delta farther than any given map point in order to trigger 
        deletion. See the delete_delta_coeff parameter for more details.
 3. KF_COLLISION_MODEL_USE_EXTENDED_PROJECTION - Enables the deletion of map
        points a dertain distance farther than any given lidar point. Used 
        primarily in specialty mapping applications where ghost points need to 
        be mitigated. Mutually exclusive with KF_COLLISION_MODEL_USE_DIFF, which
        takes precedence. */
enum
{
    KF_COLLISION_MODEL_DEFAULT = 0,
    KF_COLLISION_MODEL_USE_RADIAL = (1 << 0),
    KF_COLLISION_MODEL_REMOVE_INVALID_NORMALS = (1 << 2),

    KF_COLLISION_DEFAULT_PARAMS =
        (KF_COLLISION_MODEL_USE_RADIAL |
         KF_COLLISION_MODEL_REMOVE_INVALID_NORMALS)
};


/** KDTree Frustum Collision (KFC) mapping implementation */
template<
    typename PointT,
    typename MapT = MapOctree<PointT, MAP_OCTREE_STORE_NORMALS>,
    typename CollisionPointT = pcl::PointXYZLNormal>
class KFCMap
{
    static_assert(MapT::HAS_POINT_NORMALS);
    static_assert(
        pcl::traits::has_normal<CollisionPointT>::value &&
        pcl::traits::has_curvature<CollisionPointT>::value &&
        pcl::traits::has_label<CollisionPointT>::value);

    using Arr3f = Eigen::Array3f;
    using Vec3f = Eigen::Vector3f;

public:
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
    struct UpdateResult
    {
        union
        {
            uint64_t data;
            struct
            {
                uint32_t points_searched;
                uint32_t points_deleted;
            };
        };
#pragma GCC diagnostic pop

        inline UpdateResult(uint64_t v = 0) : data{v} {}
        inline operator uint64_t() const { return this->data; }
    };

public:
    KFCMap(double voxel_size = 0.1) : map_octree{voxel_size} {}
    ~KFCMap() = default;

public:
    /* @param frustum_search_radius The radius which is searched in the KDTree
     * for each lidar point when computing collisions. This value should 
     * optimally be set to half of the angular distance (radians) between 
     * consecutive points in the scan, and can be thought of as the radius of
     * the "frustum cone" computed for each ray.
     * @param radial_dist_thresh The radial distance from the lidar ray which
     * points must be within to be considered when computing collisions (for
     * KF_COLLISION_MODEL_RADIAL). Note that the internal computation is
     * approximated to save on resources, meaning a larger parameter value will
     * be needed to acheive a given threshold. With small angluar deviations (as
     * typical with most LiDARs), this affect is negligible.
     * @param delete_delta_coeff Scaled by the distance to each map point in
     * question, for which the lidar point range must be at least this value
     * larger before deletion of the map point occurs. Useful in negating
     * accidental deletions when rays are nearly parallel with a surface
     * (especially when not using KF_COLLISION_MODEL_RADIAL).
     * @param delete_max_range The maximum range for points that can be deleted
     * from the map.
     * @param add_max_range The maximum range for points that can be added to
     * the map.
     * @param voxel_res The octree resolution. Values <= 0 result in no change. 
     */
    void applyParams(
        double frustum_search_radius,
        double radial_dist_thresh,
        double surface_width,
        double delete_max_range,
        double add_max_range,
        double voxel_res = -1.);

    void setBounds(const Vec3f& min, const Vec3f& max);

    /* Update the map with a new set of scan points. */
    template<int CollisionV = KF_COLLISION_DEFAULT_PARAMS>
    inline UpdateResult updateMap(
        const Vec3f& origin,
        const pcl::PointCloud<PointT>& pts,
        const pcl::Indices* indices = nullptr)
    {
        return this
            ->updateMap<CollisionV, pcl::Axis>(origin, pts, nullptr, indices);
    }
    /* Update the map with a new set of scan points, utilizing the ray
     * directions of invalid points as additional samples. */
    template<
        int CollisionV = KF_COLLISION_DEFAULT_PARAMS,
        typename RayDirT = pcl::Axis>
    inline UpdateResult updateMap(
        const Vec3f& origin,
        const pcl::PointCloud<PointT>& pts,
        const std::vector<RayDirT>& inf_rays,
        const pcl::Indices* pt_indices = nullptr)
    {
        static_assert(pcl::traits::has_normal<RayDirT>::value);

        return this->updateMap<CollisionV, RayDirT>(
            origin,
            pts,
            &inf_rays,
            pt_indices);
    }

    inline typename pcl::PointCloud<PointT>::ConstPtr getPoints() const
    {
        return this->map_octree.getInputCloud();
    }

    inline const MapT& getMap() const { return this->map_octree; }

    inline size_t numPoints() const
    {
        return this->map_octree.getInputCloud()->size();
    }

protected:
    template<int CollisionModel, typename RayDirT>
    UpdateResult updateMap(
        const Vec3f& origin,
        const pcl::PointCloud<PointT>& pts,
        const std::vector<RayDirT>* inf_rays,
        const pcl::Indices* pt_indices);

protected:
    pcl::KdTreeFLANN<CollisionPointT> collision_kdtree;
    typename pcl::PointCloud<CollisionPointT>::Ptr submap_ranges{nullptr};
    MapT map_octree;

    Arr3f bounds_min = Arr3f::Constant(-std::numeric_limits<float>::infinity());
    Arr3f bounds_max = Arr3f::Constant(std::numeric_limits<float>::infinity());

    std::mutex mtx;

#if KFC_MAP_STORE_INSTANCE_BUFFERS
    struct
    {
        pcl::Indices search_indices, points_to_add;
        std::vector<float> dists;
        std::set<pcl::index_t> submap_remove_indices;
    } buff;
#endif

    double frustum_search_radius{0.01};
    double radial_dist_thresh{0.01};
    double half_surface_width{0.01};
    double delete_max_range{3.};
    double add_max_range{5.};
    //
};

};  // namespace perception
};  // namespace csm





#ifndef KFC_MAP_PRECOMPILED

    #include "impl/kfc_map_impl.hpp"

// clang-format off
#define KFC_MAP_INSTANTIATE_CLASS_TEMPLATE( \
    POINT_TYPE,                             \
    MAP_TYPE,                               \
    COLL_TYPE)                              \
    template class csm::perception::KFCMap<POINT_TYPE, MAP_TYPE, COLL_TYPE>;

#define KFC_MAP_INSTANTIATE_UPDATE_FUNC_TEMPLATE(                       \
    POINT_TYPE,                                                         \
    MAP_TYPE,                                                           \
    COLL_TYPE,                                                          \
    COLL_PARAMS,                                                        \
    RAY_TYPE)                                                           \
    template csm::perception::KFCMap<POINT_TYPE, MAP_TYPE, COLL_TYPE>:: \
        UpdateResult                                                    \
        csm::perception::KFCMap<POINT_TYPE, MAP_TYPE, COLL_TYPE>::      \
            updateMap<COLL_PARAMS, RAY_TYPE>(                           \
                const Eigen::Vector3f&,                                 \
                const pcl::PointCloud<POINT_TYPE>&,                     \
                const std::vector<RAY_TYPE>*,                           \
                const pcl::Indices*);

#define KFC_MAP_INSTANTIATE_PCL_DEPENDENCIES(   \
    POINT_TYPE,                                 \
    MAP_TYPE,                                   \
    COLL_TYPE)                                  \
    template class pcl::KdTreeFLANN<COLL_TYPE>;
// clang-format on

#endif
