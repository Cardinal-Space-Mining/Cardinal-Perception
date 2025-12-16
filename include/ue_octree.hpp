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

#include <cmath>
#include <cassert>
#include <type_traits>

#include <Eigen/Core>
#include <Eigen/Geometry>


namespace csm
{
namespace perception
{

template<typename Float_T = float, typename Index_T = uint32_t>
class UEOctree
{
    using FloatT = Float_T;
    using IndexT = std::make_unsigned<Index_T>::type;

    using Vec3f = Eigen::Vector<FloatT, 3>;
    using Vec3i = Eigen::Vector<IndexT, 3>;
    using Arr3f = Eigen::Array<FloatT, 3, 1>;
    using Arr3i = Eigen::Array<IndexT, 3, 1>;
    using Box3f = Eigen::AlignedBox<FloatT, 3>;

    using NodeDescriptor = Eigen::Vector<IndexT, 4>;

    static constexpr size_t MAX_DEPTH = (sizeof(IndexT) * 8 - 1);

public:
    UEOctree(FloatT max_res);
    ~UEOctree() = default;

public:
    void addExploredSpace(const Vec3f& min, const Vec3f& max);
    void clear();
    bool isExplored(const Vec3f& pt);

protected:
    struct Node
    {
        Node* children{nullptr};
        bool explored{false};

        Node() = default;
        inline ~Node() { this->clear(); }

        void init();
        void clear();

        Node& operator[](size_t i);
        const Node& operator[](size_t i) const;

        inline bool isNull() const { return !(this->children); }
        inline bool fullyExplored() const { return this->explored; }
        inline bool anyExplored() const
        {
            return !this->isNull() || this->fullyExplored();
        }
    };

protected:
    static Vec3i getDescriptorKey(const NodeDescriptor& n);
    static Vec3f getDescriptorKeyF(const NodeDescriptor& n);
    static Vec3i getDescriptorSpan3(const NodeDescriptor& n);
    static Vec3f getDescriptorSpan3f(const NodeDescriptor& n);

    NodeDescriptor getRootDescriptor() const;
    NodeDescriptor getChildDescriptor(const NodeDescriptor& n, size_t i) const;
    Box3f getDescriptorBox(const NodeDescriptor& n) const;

protected:
    bool adjustBounds(const Vec3f& min, const Vec3f& max);
    void recursiveExplore(
        Node& node,
        const NodeDescriptor& descriptor,
        const Box3f& zone);

protected:
    Vec3f origin{Vec3f::Zero()};
    IndexT vox_span{0};
    FloatT vox_res{0};

    Node root;
    size_t root_height{0};
};



// --- Implementation ----------------------------------------------------------

template<typename F, typename I>
UEOctree<F, I>::UEOctree(FloatT res) : vox_res{res}
{
}

template<typename F, typename I>
void UEOctree<F, I>::addExploredSpace(const Vec3f& min, const Vec3f& max)
{
    if (this->adjustBounds(min, max))
    {
        if (this->root_height < 1)
        {
            this->root.explored = true;
        }
        else
        {
            this->recursiveExplore(
                this->root,
                this->getRootDescriptor(),
                Box3f{min, max});
        }
    }
}

template<typename F, typename I>
void UEOctree<F, I>::clear()
{
    this->root.clear();
    this->root_height = 0;
    this->vox_span = 0;
}

template<typename F, typename I>
bool UEOctree<F, I>::isExplored(const Vec3f& pt)
{
    const Arr3f aligned_pt = ((pt - this->origin) / this->vox_res).array();
    if ((aligned_pt >= Arr3f::Zero()).all() &&
        (aligned_pt < Arr3f::Constant(this->vox_span)).all())
    {
        if (this->root.fullyExplored())
        {
            return true;
        }

        const Arr3i vox_idx = aligned_pt.floor().template cast<IndexT>();

        Node* node = &(this->root);
        // iterates through [root_height - 1 ... 0], while active node still has children
        for (size_t height = this->root_height;
             height-- > 0 && !node->isNull();)
        {
            const IndexT i =
                (((vox_idx.x() >> height) & 0x1) << 2 |
                 ((vox_idx.y() >> height) & 0x1) << 1 |
                 ((vox_idx.z() >> height) & 0x1));

            node = node->children + i;
            if (node->fullyExplored())
            {
                return true;
            }
        }
    }
    return false;
}


template<typename F, typename I>
void UEOctree<F, I>::Node::init()
{
    if (this->isNull())
    {
        this->children = new Node[8];
    }
}
template<typename F, typename I>
void UEOctree<F, I>::Node::clear()
{
    if (this->children)
    {
        delete[] this->children;
        this->children = nullptr;
    }
}

template<typename F, typename I>
UEOctree<F, I>::Node& UEOctree<F, I>::Node::operator[](size_t i)
{
    // assert(i < 8 && this->children);
    return this->children[i];
}
template<typename F, typename I>
const UEOctree<F, I>::Node& UEOctree<F, I>::Node::operator[](size_t i) const
{
    // assert(i < 8 && this->children);
    return this->children[i];
}


template<typename F, typename I>
UEOctree<F, I>::Vec3i UEOctree<F, I>::getDescriptorKey(const NodeDescriptor& n)
{
    return n.template head<3>();
}
template<typename F, typename I>
UEOctree<F, I>::Vec3f UEOctree<F, I>::getDescriptorKeyF(const NodeDescriptor& n)
{
    return n.template head<3>().template cast<FloatT>();
}
template<typename F, typename I>
UEOctree<F, I>::Vec3i UEOctree<F, I>::getDescriptorSpan3(
    const NodeDescriptor& n)
{
    return Vec3i::Constant(n[3]);
}
template<typename F, typename I>
UEOctree<F, I>::Vec3f UEOctree<F, I>::getDescriptorSpan3f(
    const NodeDescriptor& n)
{
    return Vec3f::Constant(static_cast<FloatT>(n[3]));
}

template<typename F, typename I>
UEOctree<F, I>::NodeDescriptor UEOctree<F, I>::getRootDescriptor() const
{
    return NodeDescriptor{0, 0, 0, 0x1 << this->root_height};
}

template<typename F, typename I>
UEOctree<F, I>::NodeDescriptor UEOctree<F, I>::getChildDescriptor(
    const NodeDescriptor& n,
    size_t i) const
{
    // assert(i < 8);
    return NodeDescriptor{
        n[0] + (n[3] / 2) * (i >> 2 & 0x1),
        n[1] + (n[3] / 2) * (i >> 1 & 0x1),
        n[2] + (n[3] / 2) * (i >> 0 & 0x1),
        n[3] / 2};
}

template<typename F, typename I>
UEOctree<F, I>::Box3f UEOctree<F, I>::getDescriptorBox(
    const NodeDescriptor& n) const
{
    const Vec3f min_corner =
        this->origin + getDescriptorKeyF(n) * this->vox_res;
    return Box3f{
        min_corner,
        min_corner + getDescriptorSpan3f(n) * this->vox_res};
}


template<typename F, typename I>
bool UEOctree<F, I>::adjustBounds(const Vec3f& min, const Vec3f& max)
{
    if (!this->root.anyExplored())
    {
        this->origin = min;
        FloatT max_diff = ((max - min) / this->vox_res).maxCoeff();
        this->root_height =
            static_cast<size_t>(std::ceil(std::log2(std::max(max_diff, 1.f))));
        this->vox_span = (0x1 << std::min(this->root_height, MAX_DEPTH));

        if (this->root_height > MAX_DEPTH)
        {
            this->root_height = MAX_DEPTH;
            return false;
        }
    }
    else
    {
        while ((min.array() < this->origin.array()).any())
        {
            if (this->root_height >= MAX_DEPTH)
            {
                return false;
            }

            uint8_t x_bit = min.x() < this->origin.x() ? 0x1 : 0x0;
            uint8_t y_bit = min.y() < this->origin.y() ? 0x1 : 0x0;
            uint8_t z_bit = min.z() < this->origin.z() ? 0x1 : 0x0;
            this->origin -= Vec3f{x_bit, y_bit, z_bit} * this->vox_res *
                            (0x1 << this->root_height);
            this->vox_span *= 2;
            this->root_height++;
            if (this->root.anyExplored())
            {
                size_t octant = (x_bit << 2) | (y_bit << 1) | (z_bit << 0);
                Node* tmp = new Node[8];
                tmp[octant].children = root.children;
                tmp[octant].explored = root.explored;
                root.children = tmp;
                root.explored = false;
            }
        }
        while (((max - this->origin).array() >=
                Arr3f::Constant(this->vox_span * this->vox_res))
                   .any())
        {
            if (this->root_height >= MAX_DEPTH)
            {
                return false;
            }

            this->vox_span *= 2;
            this->root_height++;
            if (this->root.anyExplored())
            {
                Node* tmp = new Node[8];
                tmp[0].children = root.children;
                tmp[0].explored = root.explored;
                root.children = tmp;
                root.explored = false;
            }
        }
    }

    return true;
}

template<typename F, typename I>
void UEOctree<F, I>::recursiveExplore(
    Node& node,
    const NodeDescriptor& descriptor,
    const Box3f& zone)
{
    node.init();
    bool all_explored = true;
    for (size_t i = 0; i < 8; i++)
    {
        Node& child = node[i];
        NodeDescriptor child_desc = this->getChildDescriptor(descriptor, i);
        Box3f child_span = this->getDescriptorBox(child_desc);

        if (zone.contains(child_span))
        {
            child.explored = true;
            child.clear();
        }
        else if (zone.intersects(child_span))
        {
            if (child_desc[3] > 1)
            {
                this->recursiveExplore(child, child_desc, zone);
            }
            else
            {
                child.explored = true;
                child.clear();
            }
        }

        all_explored &= child.fullyExplored();
    }

    if (all_explored)
    {
        node.explored = true;
        node.clear();
    }
}

};  // namespace perception
};  // namespace csm
