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

#define PCL_NO_PRECOMPILE
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>


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

struct EIGEN_ALIGN16 PointXYZIR
{
    PCL_ADD_POINT4D;
    float intensity;
    float reflective;

    inline constexpr PointXYZIR(const PointXYZIR& p) :
        PointXYZIR(p.x, p.y, p.z, p.intensity, p.reflective)
    {
    }
    inline constexpr PointXYZIR() : PointXYZIR(0.f, 0.f, 0.f, 0.f, 0.f) {}
    inline constexpr PointXYZIR(float _x, float _y, float _z) :
        PointXYZIR(_x, _y, _z, 0.f, 0.f)
    {
    }
    inline constexpr PointXYZIR(
        float _x,
        float _y,
        float _z,
        float _intensity,
        float _reflective) :
        data{_x, _y, _z, 1.f},
        intensity{_intensity},
        reflective{_reflective}
    {
    }

    inline constexpr PointXYZIR& operator=(const PointXYZIR& p)
    {
        x = p.x;
        y = p.y;
        z = p.z;
        intensity = p.intensity;
        reflective = p.reflective;
        return *this;
    }

    PCL_MAKE_ALIGNED_OPERATOR_NEW
};

struct EIGEN_ALIGN16 PointXYZRT
{
    PCL_ADD_POINT4D;
    float reflective;
    union
    {
        struct
        {
            uint32_t tl, th;
        };
        uint64_t t;
    };
};

struct EIGEN_ALIGN8 PointSDir
{
    float azimuth;
    float elevation;
};

struct EIGEN_ALIGN8 PointT_32HL
{
    union
    {
        struct
        {
            uint32_t tl, th;
        };
        uint64_t t;
    };
};

struct NormalTraversal : public pcl::_Normal
{
    inline constexpr NormalTraversal(const _Normal& p) :
        NormalTraversal{
            p.normal_x,
            p.normal_y,
            p.normal_z,
            p.curvature,
            p.data_c[1]}
    {
    }
    inline constexpr NormalTraversal(
        float _curvature = 0.f,
        float _trav_weight = 0.f) :
        NormalTraversal{0.f, 0.f, 0.f, _curvature, _trav_weight}
    {
    }
    inline constexpr NormalTraversal(
        float n_x,
        float n_y,
        float n_z,
        float _curvature = 0.f,
        float _trav_weight = 0.f) :
        _Normal{{{n_x, n_y, n_z, 0.f}}, {{_curvature}}}
    {
        this->data_c[1] = _trav_weight;
    }

    inline float& trav_weight() { return this->data_c[1]; }
    inline float trav_weight() const { return this->data_c[1]; }

    PCL_MAKE_ALIGNED_OPERATOR_NEW
};

};  // namespace perception
};  // namespace csm

POINT_CLOUD_REGISTER_POINT_STRUCT(
    csm::perception::PointXYZR,
    (float, x, x)(float, y, y)(float, z, z)(float, reflective, reflective))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    csm::perception::PointXYZIR,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        float,
        reflective,
        reflective))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    csm::perception::PointXYZRT,
    (float, x, x)(float, y, y)(float, z, z)(float, reflective, reflective)(
        uint32_t,
        tl,
        tl)(uint32_t, th, th))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    csm::perception::PointSDir,
    (float, azimuth, azimuth)(float, elevation, elevation))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    csm::perception::PointT_32HL,
    (uint32_t, tl, tl)(uint32_t, th, th))

POINT_CLOUD_REGISTER_POINT_WRAPPER(
    csm::perception::NormalTraversal,
    pcl::_Normal)

namespace util
{
namespace traits
{
template<typename PointT>
struct has_reflective :
    public std::bool_constant<
        std::is_same<PointT, csm::perception::PointXYZR>::value ||
        std::is_same<PointT, csm::perception::PointXYZIR>::value ||
        std::is_same<PointT, csm::perception::PointXYZRT>::value>
{
};

template<typename PointT>
struct has_intensity :
    public std::bool_constant<
        std::is_same<PointT, csm::perception::PointXYZIR>::value ||
        pcl::traits::has_intensity<PointT>::value>
{
};

template<typename PointT>
struct has_trav_weight :
    public std::bool_constant<
        std::is_same<PointT, csm::perception::NormalTraversal>::value>
{
};
};  // namespace traits
};  // namespace util
