/*******************************************************************************
*   Copyright (C) 2024-2025 Cardinal Space Mining Club                         *
*                                                                              *
*   Unless required by applicable law or agreed to in writing, software        *
*   distributed under the License is distributed on an "AS IS" BASIS,          *
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
*   See the License for the specific language governing permissions and        *
*   limitations under the License.                                             *
*                                                                              *
*                                ;xxxxxxx:                                     *
*                               ;$$$$$$$$$       ...::..                       *
*                               $$$$$$$$$$x   .:::::::::::..                   *
*                            x$$$$$$$$$$$$$$::::::::::::::::.                  *
*                        :$$$$$&X;      .xX:::::::::::::.::...                 *
*                .$$Xx++$$$$+  :::.     :;:   .::::::.  ....  :                *
*               :$$$$$$$$$  ;:      ;xXXXXXXXx  .::.  .::::. .:.               *
*              :$$$$$$$$: ;      ;xXXXXXXXXXXXXx: ..::::::  .::.               *
*             ;$$$$$$$$ ::   :;XXXXXXXXXXXXXXXXXX+ .::::.  .:::                *
*              X$$$$$X : +XXXXXXXXXXXXXXXXXXXXXXXX; .::  .::::.                *
*               .$$$$ :xXXXXXXXXXXXXXXXXXXXXXXXXXXX.   .:::::.                 *
*                X$$X XXXXXXXXXXXXXXXXXXXXXXXXXXXXx:  .::::.                   *
*                $$$:.XXXXXXXXXXXXXXXXXXXXXXXXXXX  ;; ..:.                     *
*                $$& :XXXXXXXXXXXXXXXXXXXXXXXX;  +XX; X$$;                     *
*                $$$: XXXXXXXXXXXXXXXXXXXXXX; :XXXXX; X$$;                     *
*                X$$X XXXXXXXXXXXXXXXXXXX; .+XXXXXXX; $$$                      *
*                $$$$ ;XXXXXXXXXXXXXXX+  +XXXXXXXXx+ X$$$+                     *
*              x$$$$$X ;XXXXXXXXXXX+ :xXXXXXXXX+   .;$$$$$$                    *
*             +$$$$$$$$ ;XXXXXXx;;+XXXXXXXXX+    : +$$$$$$$$                   *
*              +$$$$$$$$: xXXXXXXXXXXXXXX+      ; X$$$$$$$$                    *
*               :$$$$$$$$$. +XXXXXXXXX;      ;: x$$$$$$$$$                     *
*               ;x$$$$XX$$$$+ .;+X+      :;: :$$$$$xX$$$X                      *
*              ;;;;;;;;;;X$$$$$$$+      :X$$$$$$&.                             *
*              ;;;;;;;:;;;;;x$$$$$$$$$$$$$$$$x.                                *
*              :;;;;;;;;;;;;.  :$$$$$$$$$$X                                    *
*               .;;;;;;;;:;;    +$$$$$$$$$                                     *
*                 .;;;;;;.       X$$$$$$$:                                     *
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

        inline constexpr PointXYZR(const PointXYZR &p) :
            PointXYZR(p.x, p.y, p.z, p.reflective) {}
        inline constexpr PointXYZR() :
            PointXYZR(0.f, 0.f, 0.f, 0.f) {}
        inline constexpr PointXYZR(float _x, float _y, float _z) :
            PointXYZR(_x, _y, _z, 0.f) {}
        inline constexpr PointXYZR(float _x, float _y, float _z, float _reflective) :
            data{ _x, _y, _z, 1.f }, reflective{ _reflective } {}

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

        inline constexpr PointXYZIR(const PointXYZIR &p) :
            PointXYZIR(p.x, p.y, p.z, p.intensity, p.reflective) {}
        inline constexpr PointXYZIR() :
            PointXYZIR(0.f, 0.f, 0.f, 0.f, 0.f) {}
        inline constexpr PointXYZIR(float _x, float _y, float _z) :
            PointXYZIR(_x, _y, _z, 0.f, 0.f) {}
        inline constexpr PointXYZIR(float _x, float _y, float _z, float _intensity, float _reflective) :
            data{ _x, _y, _z, 1.f }, intensity{ _intensity }, reflective{ _reflective } {}

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

    using OdomPointType = pcl::PointXYZ;
    using MappingPointType = pcl::PointXYZ;
    using FiducialPointType = csm::perception::PointXYZR;
    using CollisionPointType = pcl::PointXYZLNormal;

};
};

POINT_CLOUD_REGISTER_POINT_STRUCT ( csm::perception::PointXYZR,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (float, reflective, reflective) )

POINT_CLOUD_REGISTER_POINT_STRUCT ( csm::perception::PointXYZIR,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (float, intensity, intensity)
                                    (float, reflective, reflective) )

POINT_CLOUD_REGISTER_POINT_STRUCT ( csm::perception::PointXYZRT,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (float, reflective, reflective)
                                    (uint32_t, tl, tl)
                                    (uint32_t, th, th) )

namespace util
{
namespace traits
{
    template<typename PointT>
    struct has_reflective :
        public std::bool_constant<
            std::is_same<PointT, csm::perception::PointXYZR>::value ||
            std::is_same<PointT, csm::perception::PointXYZIR>::value ||
            std::is_same<PointT, csm::perception::PointXYZRT>::value >
    {};

    template<typename PointT>
    struct has_intensity :
        public std::bool_constant<
            std::is_same<PointT, csm::perception::PointXYZIR>::value ||
            pcl::traits::has_intensity<PointT>::value >
    {};
};
};
