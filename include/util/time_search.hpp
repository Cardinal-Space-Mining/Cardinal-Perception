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

#include <deque>
#include <utility>
#include <stddef.h>
#include <algorithm>
#include <functional>


namespace util
{

// template<typename Stamp_T = double>
// struct TSBase
// {
//     static_assert(std::is_numeric<Stamp_T>::value);

//     using StampT = Stamp_T;

//     template<typename T>
//     using StampedElem = std::pair<StampT, T>;
//     template<typename T, typename AllocT = std::allocator<StampedElem<T>>>
//     using TSDeque = std::deque<StampedElem<T>, AllocT>;
// };


namespace tsq  // TimeStamp "Q" (queue)
{

template<typename T> using TSQ = std::deque<std::pair<double, T>>;
template<typename T> using TSQElem = std::pair<double, T>;

/** Returns the index of the element with timestamp equal to or immediately
  * before (previous in time) - ei. idx - 1 is the element whose timestamp is
  * immediately after. Returns 0 if the target is newer than the newest
  * timestamp, and q.size() if the timestamp is older than the oldest in the
  * queue. This is the same index which should be used if inserting into the
  * queue such that it stays sorted. */
template<typename T>
size_t binarySearchIdx(const TSQ<T>& q, double ts)
{
    if (q.empty())
    {
        return 0;
    }
    if (ts >= q.front().first)
    {
        return 0;
    }
    if (ts <= q.back().first)
    {
        return q.size();
    }

    size_t after = 0, before = q.size();
    while (after < before)
    {
        size_t mid = (after + before) / 2;
        if (ts < q[mid].first)
        {
            after = mid;
        }
        else if (ts > q[mid].first)
        {
            before = mid;
        }
        else
        {
            return mid;
        }

        if (before - after <= 1)
        {
            return before;
        }
    }
    return 0;
}

template<typename T>
inline void trimToStamp(TSQ<T>& q, double min_ts)
{
    q.resize(util::tsq::binarySearchIdx<T>(q, min_ts));
}

template<typename T>
inline bool inWindow(const TSQ<T>& q, double ts, double window)
{
    return q.empty() || ts >= (std::max(q.front().first, ts) - window);
}

template<typename T>
inline double windowMin(const TSQ<T>& q, double new_ts, double window)
{
    return (q.empty() ? new_ts : std::max(q.front().first, new_ts)) - window;
}

template<typename T>
inline double newestStamp(const TSQ<T>& q)
{
    return q.empty() ? 0. : q.front().first;
}

template<typename T>
inline double oldestStamp(const TSQ<T>& q)
{
    return q.empty() ? 0. : q.back().first;
}

template<typename T>
inline bool validLerpIdx(const TSQ<T>& q, size_t idx)
{
    return idx > 0 && idx < q.size();
}

template<typename T>
inline bool extractNormalizedRange(
    TSQ<T>& dest,
    const TSQ<T>& q,
    double t1,
    double t2,
    std::function<T(const T&, const T&, double)> lerp)
{
    size_t oldest = binarySearchIdx<T>(q, t1);
    size_t newest = binarySearchIdx<T>(q, t2);
    if (oldest == q.size() && oldest > 0)
    {
        oldest--;
    }
    if (newest > 0)
    {
        newest--;
    }
    if (newest == oldest)
    {
        return false;  // cannot lerp with only 1 sample
    }

    if (oldest - newest > 1)  // copy the internal samples
    {
        dest.assign(
            q.begin() + (newest > t2 ? newest + 1 : newest),
            q.begin() + (oldest < t1 ? oldest : oldest + 1));

        for (TSQElem<T>& e : dest)
        {
            e.first = (e.first - t1) / (t2 - t1);
        }
    }

    const TSQElem<T>& a = q[oldest];
    const TSQElem<T>& b = q[oldest - 1];
    const TSQElem<T>& c = q[newest + 1];
    const TSQElem<T>& d = q[newest];

    TSQElem<T>& start = dest.emplace_back();
    TSQElem<T>& end = dest.emplace_front();

    start.first = 0.;
    end.first = 1.;
    start.second =
        lerp(a.second, b.second, (t1 - a.first) / (b.first - a.first));
    end.second = lerp(c.second, d.second, (t2 - c.first) / (d.first - c.first));

    return true;
}

};  // namespace tsq
};  // namespace util
