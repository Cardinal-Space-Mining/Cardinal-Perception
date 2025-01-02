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

    using OdomPointType = pcl::PointXYZ;
    using CollisionPointType = pcl::PointXYZLNormal;
    using MappingPointType = pcl::PointXYZ;
    using FiducialPointType = csm::perception::PointXYZR;
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
