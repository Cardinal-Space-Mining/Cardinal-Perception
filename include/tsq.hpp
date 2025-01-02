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

#include <deque>
#include <utility>
#include <stddef.h>
#include <algorithm>


namespace util
{
namespace tsq   // TimeStamp "Q" (queue)
{

template<typename T> using TSQ = std::deque<std::pair<double, T>>;
template<typename T> using TSQElem = std::pair<double, T>;

/** Returns the index of the element with timestamp equal to or immediately before (previous in time) - ei. idx - 1 is the
 * element whose timestamp is immediately after. Returns 0 if the target is newer than the newest timestamp, and q.size() if
 * the timestamp is older than the oldest in the queue. This is the same index which should be used if inserting into the
 * queue such that it stays sorted. */
template<typename T>
size_t binarySearchIdx(const TSQ<T>& q, double ts)
{
    if(q.empty()) return 0;
    if(ts >= q.front().first) return 0;
    if(ts <= q.back().first) return q.size();

    size_t after = 0, before = q.size();
    for(;after < before;)
    {
        size_t mid = (after + before) / 2;
        if(ts < q[mid].first)
        {
            after = mid;
        }
        else if(ts > q[mid].first)
        {
            before = mid;
        }
        else
        {
            return mid;
        }

        if(before - after <= 1)
        {
            return before;
        }
    }
    return 0;
}

template<typename T>
inline void trimToStamp(TSQ<T>& q, double min_ts)
{
    q.resize( util::tsq::binarySearchIdx<T>(q, min_ts) );
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
inline bool validLerpIdx(const TSQ<T>& q, size_t idx)
{
    return idx > 0 && idx < q.size();
}

};
};
