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

#include <Eigen/Core>


namespace util
{


template<typename Float_T = float, typename Index_T = int32_t>
class GridMeta
{
    static_assert(std::is_floating_point<Float_T>::value);

public:
    using FloatT = Float_T;
    using IndexT = Index_T;

    template<typename T>
    using Vec2 = Eigen::Vector<T, 2>;
    template<typename T>
    using Vec3 = Eigen::Vector<T, 3>;
    template<int D>
    using VecF = Eigen::Vector<FloatT, D>;

    using Vec2f = Vec2<FloatT>;
    using Vec3f = Vec3<FloatT>;
    using Vec2i = Vec2<IndexT>;
    using Vec3i = Vec3<IndexT>;
    using CellPos2 = Vec2i;
    using CellPos3 = Vec3i;

public:
    inline GridMeta() = default;
    inline ~GridMeta() = default;

public:
    template<int D>
    inline void
        reconfigure(const VecF<D>& min, const VecF<D>& max, FloatT cell_res)
    {
        static_assert(D == 2 || D == 3);

        this->reconfigure(min, max, VecF<D>::Constant(cell_res));
    }

    inline void
        reconfigure(const Vec2f& min, const Vec2f& max, const Vec2f& cell_res)
    {
        this->origin << min, 0.f;
        this->cell_res << cell_res, 0.f;
        this->dim.x() =
            static_cast<IndexT>(std::ceil((max.x() - min.x()) / cell_res.x()));
        this->dim.y() =
            static_cast<IndexT>(std::ceil((max.y() - min.y()) / cell_res.y()));
        this->dim.z() = 1;
    }
    inline void
        reconfigure(const Vec3f& min, const Vec3f& max, const Vec3f& cell_res)
    {
        this->origin = min;
        this->cell_res = cell_res;
        this->dim.x() =
            static_cast<IndexT>(std::ceil((max.x() - min.x()) / cell_res.x()));
        this->dim.y() =
            static_cast<IndexT>(std::ceil((max.y() - min.y()) / cell_res.y()));
        this->dim.z() =
            static_cast<IndexT>(std::ceil((max.z() - min.z()) / cell_res.z()));
    }

    template<typename T>
    inline void initBuffer2(std::vector<T>& buff) const
    {
        buff.resize(this->size2());
    }
    template<typename T>
    inline void initBuffer2(std::vector<T>& buff, const T& val) const
    {
        buff.resize(this->size2(), val);
    }
    template<typename T>
    inline void initBuffer3(std::vector<T>& buff) const
    {
        buff.resize(this->size3());
    }
    template<typename T>
    inline void initBuffer3(std::vector<T>& buff, const T& val) const
    {
        buff.resize(this->size3(), val);
    }

    inline IndexT flattenIdx(const CellPos2& pos) const
    {
        return (pos.y() * this->dim.x()) + pos.x();
    }
    inline IndexT flattenIdx(const CellPos3& pos) const
    {
        return (this->dim.x() * (this->dim.y() * pos.z() + pos.y())) + pos.x();
    }
    inline CellPos2 expandIdx2(IndexT idx) const
    {
        return this->expandIdx3().template head<2>();
    }
    inline CellPos3 expandIdx3(IndexT idx) const
    {
        IndexT xy_area = this->dim.x() * this->dim.y();
        IndexT idx_mod_xy = idx % xy_area;
        return CellPos3{
            (idx_mod_xy % this->dim.x()),
            (idx_mod_xy / this->dim.x()),
            (idx / xy_area)};
    }

    inline CellPos2 getBoundingCellPos(const Vec2f& pt) const
    {
        return CellPos2{
            static_cast<IndexT>(
                std::floor((pt.x() - this->origin.x()) / this->cell_res.x())),
            static_cast<IndexT>(
                std::floor((pt.y() - this->origin.y()) / this->cell_res.y())),
        };
    }
    inline CellPos3 getBoundingCellPos(const Vec3f& pt) const
    {
        return CellPos3{
            static_cast<IndexT>(
                std::floor((pt.x() - this->origin.x()) / this->cell_res.x())),
            static_cast<IndexT>(
                std::floor((pt.y() - this->origin.y()) / this->cell_res.y())),
            this->cell_res.z() == 0.
                ? 0
                : static_cast<IndexT>(std::floor(
                      (pt.z() - this->origin.z()) / this->cell_res.z())),
        };
    }
    template<int D>
    inline IndexT getBoundingCellIdx(const VecF<D>& pt) const
    {
        static_assert(D == 2 || D == 3);

        return this->flattenIdx(this->getBoundingCellPos(pt));
    }

    template<typename T, int D>
    inline typename std::vector<T>::reference getCell(
        std::vector<T>& data,
        const VecF<D>& pt) const
    {
        static_assert(D == 2 || D == 3);

        return data[this->getBoundingCellIdx(pt)];
    }
    template<typename T, int D>
    inline const typename std::vector<T>::reference getCell(
        const std::vector<T>& data,
        const VecF<D>& pt) const
    {
        static_assert(D == 2 || D == 3);

        return data[this->getBoundingCellIdx(pt)];
    }

    inline Vec2f getCellCenter2(const CellPos2& pos) const
    {
        return this->origin.template head<2>() +
               (pos.template cast<FloatT>() + Vec2f::Constant(0.5f))
                   .cwiseProduct(this->cell_res.template head<2>());
    }
    inline Vec3f getCellCenter3(const CellPos3& pos) const
    {
        return this->origin +
               (pos.template cast<FloatT>() + Vec3f::Constant(0.5f))
                   .cwiseProduct(this->cell_res);
    }

    inline bool isInBoundingBox(const Vec2f& pt) const
    {
        return (
            pt.x() >= this->origin.x() &&
            pt.x() < this->origin.x() + (this->dim.x() * this->cell_res.x()) &&
            pt.y() >= this->origin.y() &&
            pt.y() < this->origin.y() + (this->dim.y() * this->cell_res.y()));
    }
    inline bool isInBoundingBox(const Vec3f& pt) const
    {
        return (pt.x() >= this->origin.x() &&
                pt.x() <
                    this->origin.x() + (this->dim.x() * this->cell_res.x()) &&
                pt.y() >= this->origin.y() &&
                pt.y() <
                    this->origin.y() + (this->dim.y() * this->cell_res.y())) &&
               (this->cell_res.z() == 0. ||
                (pt.z() >= this->origin.z() &&
                 pt.z() <
                     this->origin.z() + (this->dim.z() * this->cell_res.z())));
    }

    inline Vec2f minBound2() const { return this->origin.template head<2>(); }
    inline Vec2f maxBound2() const
    {
        return this->origin.template head<2>() +
               this->dim.template head<2>()
                   .template cast<FloatT>()
                   .cwiseProduct(this->cell_res.template head<2>());
    }
    inline Vec3f minBound3() const { return this->origin; }
    inline Vec3f maxBound3() const
    {
        return this->origin +
               this->dim.template cast<FloatT>().cwiseProduct(this->cell_res);
    }

    inline const Vec2f cellRes2() const
    {
        return this->cell_res.template head<2>();
    }
    inline const Vec3f& cellRes3() const { return this->cell_res; }
    inline size_t size2() const { return static_cast<size_t>(this->maxIdx2()); }
    inline size_t size3() const { return static_cast<size_t>(this->maxIdx3()); }

protected:
    inline IndexT maxIdx2() const
    {
        return this->dim.template head<2>().prod();
    }
    inline IndexT maxIdx3() const { return this->dim.prod(); }

protected:
    Vec3f origin;
    Vec3f cell_res;
    Vec3i dim;
};


};  // namespace util
