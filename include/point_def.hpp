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

#ifndef CORE_MODULES_PCL_PRECOMPILED
    #define PCL_NO_PRECOMPILE
#endif
#include <pcl/pcl_macros.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#include <pcl/point_types.h>
#pragma GCC diagnostic pop


namespace csm
{
namespace perception
{

struct EIGEN_ALIGN16 PointXYZR
{
    PCL_ADD_POINT4D;
    float reflective;

    inline constexpr PointXYZR(const PointXYZR& p) :
        PointXYZR(p.x, p.y, p.z, p.reflective)
    {
    }
    inline constexpr PointXYZR() : PointXYZR(0.f, 0.f, 0.f, 0.f) {}
    inline constexpr PointXYZR(float _x, float _y, float _z) :
        PointXYZR(_x, _y, _z, 0.f)
    {
    }
    inline constexpr PointXYZR(
        float _x,
        float _y,
        float _z,
        float _reflective) :
        data{_x, _y, _z, 1.f},
        reflective{_reflective}
    {
    }

    inline constexpr PointXYZR& operator=(const PointXYZR& p)
    {
        x = p.x;
        y = p.y;
        z = p.z;
        reflective = p.reflective;
        return *this;
    }

    PCL_MAKE_ALIGNED_OPERATOR_NEW
};

struct EIGEN_ALIGN8 PointSDir
{
    float azimuth;
    float elevation;

    inline float theta() const { return this->azimuth; }
    inline float phi() const { return (3.1415926f / 2.f) - this->elevation; }
};

struct EIGEN_ALIGN8 PointT_32HL
{
    union
    {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
        struct
        {
            uint32_t tl, th;
        };
#pragma GCC diagnostic pop
        uint64_t t;
    };

    inline uint64_t integer_time() const { return this->t; }
    static inline uint64_t time_base() { return 1000000; }
};

};  // namespace perception
};  // namespace csm

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(
    csm::perception::PointXYZR,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, reflective, reflective))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    csm::perception::PointSDir,
    (float, azimuth, azimuth)
    (float, elevation, elevation))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    csm::perception::PointT_32HL,
    (uint32_t, tl, tl)
    (uint32_t, th, th))

// clang-format on



namespace util
{
namespace traits
{

template<typename PointT>
struct has_reflective :
    public std::bool_constant<
        std::is_same<PointT, csm::perception::PointXYZR>::value>
{
};

template<typename PointT>
struct has_spherical :
    public std::bool_constant<
        std::is_same<PointT, csm::perception::PointSDir>::value>
{
};

template<typename PointT>
struct has_integer_time :
    public std::bool_constant<
        std::is_same<PointT, csm::perception::PointT_32HL>::value>
{
};

};  // namespace traits
};  // namespace util
