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

#include <cmath>
#include <limits>
#include <numbers>
#include <type_traits>

#include <pcl/point_types.h>


namespace csm
{
namespace perception
{

namespace traversibility
{

/* WeightEval does nothing for non-specialized types. */
template<typename T>
struct WeightEval;

/* Currently specialized evaluators are:
 * float
 * double
 * pcl::PointXYZI (intensity field)
 * pcl::PointXYZINormal (intensity field) */

/* WeightEval specialization for float. Just a wrapper. */
template<>
struct WeightEval<float>
{
    using EvalT = float;
    using WeightT = float;

    inline static WeightT& eval(EvalT& v) { return v; }
    inline static const WeightT& eval(const EvalT& v) { return v; }
};
/* WeightEval specialization for float. Just a wrapper. */
template<>
struct WeightEval<double>
{
    using EvalT = double;
    using WeightT = double;

    inline static WeightT& eval(EvalT& v) { return v; }
    inline static const WeightT& eval(const EvalT& v) { return v; }
};
/* WeightEval specialization for pcl::PointXYZI. Exposes the intensity member. */
template<>
struct WeightEval<pcl::PointXYZI>
{
    using EvalT = pcl::PointXYZI;
    using WeightT = float;

    inline static WeightT& eval(EvalT& v) { return v.intensity; }
    inline static const WeightT& eval(const EvalT& v) { return v.intensity; }
};
/* WeightEval specialization for pcl::PointXYZINormal. Exposes the intensity member. */
template<>
struct WeightEval<pcl::PointXYZINormal>
{
    using EvalT = pcl::PointXYZINormal;
    using WeightT = float;

    inline static WeightT& eval(EvalT& v) { return v.intensity; }
    inline static const WeightT& eval(const EvalT& v) { return v.intensity; }
};


/* Traits struct to test if WeightEval is specialized for a given type and
 * the eval() function is defined correctly. */
template<typename, typename = void>
struct has_weight_eval : std::false_type
{
};
template<typename T>
struct has_weight_eval<
    T,
    std::void_t<
        typename WeightEval<T>::WeightT,
        decltype(WeightEval<T>::eval(std::declval<T&>()))>> : std::true_type
{
};

/* Weighting-type mapping shortcut for valid WeightEval<> specializations. */
template<typename T>
using weight_t = typename WeightEval<T>::WeightT;

/* Checks if a weight evaluator is defined, the mapped weight-type is the same as the
 * input type, and if this type is trivial (can be constexpr evaluated) */
template<typename T>
inline constexpr bool can_constexpr_eval_weight =
    has_weight_eval<T>::value && std::is_same_v<T, weight_t<T>> &&
    std::is_trivially_constructible_v<T> && std::is_trivially_copyable_v<T>;


/* Shortcut to unwrap the weight of any type for which an evaluator is defined. */
template<typename T>
inline weight_t<T>& weight(T& x)
{
    static_assert(
        has_weight_eval<T>::value,
        "Type does not support traversibility weight evaluation");

    return WeightEval<T>::eval(x);
}
/* Shortcut to unwrap the weight of any type for which an evaluator is defined. */
template<typename T>
inline const weight_t<T>& weight(const T& x)
{
    static_assert(
        has_weight_eval<T>::value,
        "Type does not support traversibility weight evaluation");

    return WeightEval<T>::eval(x);
}

/* Shortcuts wrapper-like evaluators so that constexpr evaluation can be preserved.
 * (Assumes wrapper-like eval API's are indeed wrappers, which is currently the
 * case. Be careful - this assumption may not always be true!) */
template<typename T>
inline constexpr weight_t<T> constexprWeight(const T& x)
{
    if constexpr (can_constexpr_eval_weight<T>)
    {
        return x;
    }
    else
    {
        return weight(x);
    }
}



/* Traversibility/Pathplanning definition:
 * Nominal range is the always traversible - lower is more traversible, higher is less
 * Extended range is possibly traversible - higher is less
 * Anything higher than extended range bound is a definite obstacle
 * Anything lower than nominal range min signals a frontier node
 * NaN signals unknown traversibility */

/* Traversibility weight nominal range minimum bound (inclusive).
 * Value type is that which is mapped by the weight eval specialization
 * given the template param. */
template<typename T>
inline constexpr weight_t<T> NOMINAL_MIN_WEIGHT = []
{
    static_assert(has_weight_eval<T>::value);
    return weight_t<T>(0);
}();
/* Traversibility weight nominal range maximum bound (inclusive).
 * [This is the same value as the extended range minimum bound (exclusive)]
 * Value type is that which is mapped by the weight eval specialization
 * given the template param. */
template<typename T>
inline constexpr weight_t<T> NOMINAL_MAX_WEIGHT = []
{
    static_assert(has_weight_eval<T>::value);
    return weight_t<T>(1);
}();
/* Traversibility weight extended range maximum bound (inclusive).
 * Value type is that which is mapped by the weight eval specialization
 * given the template param. */
template<typename T>
inline constexpr weight_t<T> EXTENDED_MAX_WEIGHT = []
{
    static_assert(has_weight_eval<T>::value);
    return weight_t<T>(10);
}();


/* Marker value which denotes the traversibility weight of a "definite obstacle"
 * Value type is that which is mapped by the weight eval specialization
 * given the template param. */
template<typename T>
inline constexpr weight_t<T> OBSTACLE_MARKER_VAL = []
{
    static_assert(
        has_weight_eval<T>::value &&
        std::numeric_limits<weight_t<T>>::has_infinity);
    return std::numeric_limits<weight_t<T>>::infinity();
}();
/* Marker value which denotes the traversibility weight of a frontier node (path-planning only)
 * Value type is that which is mapped by the weight eval specialization
 * given the template param. */
template<typename T>
inline constexpr weight_t<T> FRONTIER_MARKER_VAL = []
{
    static_assert(
        has_weight_eval<T>::value &&
        std::numeric_limits<weight_t<T>>::has_infinity);
    return -std::numeric_limits<weight_t<T>>::infinity();
}();
/* Marker value which denotes an unknown traversibility weight
 * Value type is that which is mapped by the weight eval specialization
 * given the template param. */
template<typename T>
inline constexpr weight_t<T> UNKNOWN_MARKER_VAL = []
{
    static_assert(
        has_weight_eval<T>::value &&
        std::numeric_limits<weight_t<T>>::has_quiet_NaN);
    return std::numeric_limits<weight_t<T>>::quiet_NaN();
}();


/* Denormalize the input value (range [0, 1]) to the equivalent weight
 * in the range [NOMINAL_MIN_WEIGHT, NOMINAL_MAX_WEIGHT].
 * Input value must be floating point, and return value is of the type
 * mapped by the equivalent weight evaluator given the template parameter T.
 * (Set T to the container type, not the weight type!) */
template<typename T, typename Fp>
inline constexpr weight_t<T> nominalWeight(Fp normalized_val)
{
    static_assert(std::is_floating_point<Fp>::value);

    constexpr weight_t<T> NOMINAL_RANGE =
        (NOMINAL_MAX_WEIGHT<T> - NOMINAL_MIN_WEIGHT<T>);

    return static_cast<weight_t<T>>(normalized_val) * NOMINAL_RANGE +
           NOMINAL_MIN_WEIGHT<T>;
}
/* Denormalize the input value (range [0, 1]) to the equivalent weight
 * in the range [NOMINAL_MAX_WEIGHT, EXTENDED_MAX_WEIGHT].
 * Input value must be floating point, and return value is of the type
 * mapped by the equivalent weight evaluator given the template parameter T.
 * (Set T to the container type, not the weight type!)*/
template<typename T, typename Fp>
inline constexpr weight_t<T> extendedWeight(Fp normalized_val)
{
    static_assert(std::is_floating_point<Fp>::value);

    constexpr weight_t<T> EXTENDED_RANGE =
        (EXTENDED_MAX_WEIGHT<T> - NOMINAL_MAX_WEIGHT<T>);

    return static_cast<weight_t<T>>(normalized_val) * EXTENDED_RANGE +
           NOMINAL_MAX_WEIGHT<T>;
}


/* Test if a traversibility weight is in the nominal range - that is -
 * [NOMINAL_MIN_WEIGHT, NOMINAL_MAX_WEIGHT].
 * Weight evaluators are automatically applied to "unbox" the weight value. */
template<typename T>
inline constexpr bool isNominal(const T& val)
{
    weight_t<T> w = constexprWeight<T>(val);
    return w >= NOMINAL_MIN_WEIGHT<T> && w <= NOMINAL_MAX_WEIGHT<T>;
}
/* Test if a traversibility weight is in the extended range - that is -
 * (NOMINAL_MAX_WEIGHT, EXTENDED_MAX_WEIGHT].
 * Weight evaluators are automatically applied to "unbox" the weight value. */
template<typename T>
inline constexpr bool isExtended(const T& val)
{
    weight_t<T> w = constexprWeight<T>(val);
    return w > NOMINAL_MAX_WEIGHT<T> && w <= EXTENDED_MAX_WEIGHT<T>;
}
/* Test if a traversibility weight is in the nominal or extended range - ie.
 * it't weight value is meaningful and not just a marker.
 * Weight evaluators are automatically applied to "unbox" the weight value. */
template<typename T>
inline constexpr bool isWeighted(const T& val)
{
    weight_t<T> w = constexprWeight<T>(val);
    return w >= NOMINAL_MIN_WEIGHT<T> && w <= EXTENDED_MAX_WEIGHT<T>;
}
/* Test if a traversibility weight denotes a definite obstacle.
 * Weight evaluators are automatically applied to "unbox" the weight value. */
template<typename T>
inline constexpr bool isObstacle(const T& val)
{
    return constexprWeight<T>(val) > EXTENDED_MAX_WEIGHT<T>;
}
/* Test if a traversibility weight denotes a frontier node.
 * Weight evaluators are automatically applied to "unbox" the weight value. */
template<typename T>
inline constexpr bool isFrontier(const T& val)
{
    return constexprWeight<T>(val) < NOMINAL_MIN_WEIGHT<T>;
}
/* Test if a traversibility weight is unknown.
 * Weight evaluators are automatically applied to "unbox" the weight value. */
template<typename T>
inline constexpr bool isUnknown(const T& val)
{
    return std::isnan(constexprWeight<T>(val));
}
/* Test if a traversibility weight should be read as a marker - ie. it's weight
 * value is not meaningful.
 * Weight evaluators are automatically applied to "unbox" the weight value. */
template<typename T>
inline constexpr bool isMarker(const T& val)
{
    return !isWeighted<T>(val);
}

};  // namespace traversibility

};  // namespace perception
};  // namespace csm


namespace util
{
namespace traits
{

template<typename T>
using supports_traversibility =
    csm::perception::traversibility::has_weight_eval<T>;

template<typename T>
using traversibility_weight_t = csm::perception::traversibility::weight_t<T>;

};  // namespace traits
};  // namespace util
