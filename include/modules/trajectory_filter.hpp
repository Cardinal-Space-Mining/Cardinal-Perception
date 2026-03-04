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
#include <mutex>
#include <atomic>
#include <memory>
#include <vector>
#include <utility>
#include <iterator>
#include <type_traits>

#include <sstream>
#include <iostream>

#include <util/geometry.hpp>
#include <util/time_search.hpp>

// #define TRAJECTORY_FILTER_PRINT_DEBUG 1
#ifndef TRAJECTORY_FILTER_PRINT_DEBUG
    #define TRAJECTORY_FILTER_PRINT_DEBUG 0
#endif

#define TRAJECTORY_FILTER_DEFAULT_SAMPLE_WINDOW_S          0.5
#define TRAJECTORY_FILTER_DEFAULT_FILTER_WINDOW_S          0.3
#define TRAJECTORY_FILTER_DEFAULT_AVG_LINEAR_ERROR_THRESH  2e-2
#define TRAJECTORY_FILTER_DEFAULT_AVG_ANGULAR_ERROR_THRESH 5e-2
#define TRAJECTORY_FILTER_DEFAULT_MAX_LINEAR_ERROR_STDDEV  4e-2
#define TRAJECTORY_FILTER_DEFAULT_MAX_ANGULAR_ERROR_STDDEV 4e-2


namespace csm
{
namespace perception
{

/*  */
template<typename Pose3_T = util::geom::Pose3d>
class TrajectoryFilter
{
    static_assert(
        std::is_same_v<Pose3_T, util::geom::Pose3f> ||
        std::is_same_v<Pose3_T, util::geom::Pose3d>);

public:
    using Pose3 = Pose3_T;

    template<typename F>
    using Pose3_ = util::geom::Pose3<F>;

    template<typename T>
    using TStamped_ = util::tsq::TSQElem<T>;

public:
    /*  */
    struct MeasPair
    {
        Pose3 odometry;
        Pose3 absolute;

        double linear_error{0.};
        double angular_error{0.};

        void setErrorFrom(const MeasPair& prev);
    };
    /*  */
    struct FilterMetrics
    {
        std::atomic<bool> last_filter_status{false};
        std::atomic<size_t> last_filter_mask{0};

        std::atomic<size_t> odom_q_len{0};
        std::atomic<size_t> abs_q_len{0};
        std::atomic<size_t> meas_q_len{0};

        std::atomic<double> last_search_window{0.};
        std::atomic<double> last_avg_linear_err{0.};
        std::atomic<double> last_avg_angular_err{0.};
        std::atomic<double> last_linear_err_stddev{0.};
        std::atomic<double> last_angular_err_stddev{0.};

        std::atomic<double> newest_meas_linear_err{0.};
        std::atomic<double> newest_meas_angular_err{0.};
    };

public:
    TrajectoryFilter() = default;
    TrajectoryFilter(const TrajectoryFilter& ref);

    ~TrajectoryFilter() = default;

public:
    /*  */
    void applyParams(
        double sample_window_s,
        double filter_window_s,
        double avg_linear_error_thresh,
        double avg_angular_error_thresh,
        double max_linear_error_dev,
        double max_angular_error_dev);

    /* Add timestamped odometry measurement and iterate the filter. */
    template<typename F>
    void addOdom(const Pose3_<F>& pose, double ts);
    /* Add timestamped absolute measurement and iterate the filter. */
    template<typename F>
    void addAbsolute(const Pose3_<F>& pose, double ts);

    /*  */
    TStamped_<MeasPair> getFiltered() const;
    /*  */
    void getFiltered(TStamped_<MeasPair>& out) const;

    /*  */
    const FilterMetrics& getStatus() const;

protected:
    void processQueue();
    void updateFilter();
    void printQueues();

private:
    // deque of "source" odometry spanning our time window
    std::deque<TStamped_<Pose3>> odom_queue;
    // temporary queue for measurements that need to be processed
    std::deque<TStamped_<Pose3>> absolute_queue;
    // active trajectory queue
    std::deque<TStamped_<MeasPair>> trajectory;

    std::mutex mtx;
    mutable std::mutex result_mtx;

    MeasPair last_filtered_meas;
    double last_fitered_stamp{0.};

    FilterMetrics metrics;

    double sample_window_s{TRAJECTORY_FITLER_DEFAULT_SAMPLE_WINDOW_S};
    double filter_window_s{TRAJECTORY_FILTER_DEFAULT_FILTER_WINDOW_S};
    double max_avg_linear_err{
        TRAJECTORY_FILTER_DEFAULT_AVG_LINEAR_ERROR_THRESH};
    double max_avg_angular_err{
        TRAJECTORY_FILTER_DEFAULT_AVG_ANGULAR_ERROR_THRESH};
    double max_linear_err_stddev{
        TRAJECTORY_FILTER_DEFAULT_MAX_LINEAR_ERROR_STDDEV};
    double max_angular_err_stddev{
        TRAJECTORY_FILTER_DEFAULT_MAX_ANGULAR_ERROR_STDDEV};
};



// --- Implementation ----------------------------------------------------------

#ifndef TRAJECTORY_FILTER_PRECOMPILED

    #if TRAJECTORY_FILTER_PRINT_DEBUG
        #include <sstream>
        #include <iostream>
    #endif


template<typename P>
TrajectoryFilter<P>::TrajectoryFilter(const TrajectoryFilter& ref) :
    sample_window_s{ref.sample_window_s},
    filter_window_s{ref.filter_window_s},
    max_avg_linear_err{ref.max_avg_linear_err},
    max_avg_angular_err{ref.max_avg_angular_err},
    max_linear_err_stddev{max_linear_err_stddev},
    max_angular_err_stddev{max_angular_err_stddev}
{
}


template<typename P>
void TrajectoryFilter<P>::applyParams(
    double sample_window_s,
    double filter_window_s,
    double avg_linear_error_thresh,
    double avg_angular_error_thresh,
    double max_linear_error_dev,
    double max_angular_error_dev)
{
    std::unique_lock lock{this->mtx};

    this->sample_window_s = sample_window_s;
    this->filter_window_s = filter_window_s;
    this->max_avg_linear_err = avg_linear_error_thresh;
    this->max_avg_angular_err = avg_angular_error_thresh;
    this->max_linear_err_stddev = max_linear_error_dev;
    this->max_angular_err_stddev = max_angular_error_dev;
}

template<typename P>
template<typename F>
void TrajectoryFilter<P>::addOdom<F>(const Pose3_<F>& pose, double ts)
{
    using namespace util::geom::cvt::ops;
    using namespace util::tsq;

    std::unique_lock lock{this->mtx};

    // Only insert if the stamp is newer than the window min
    if (inWindow(this->trajectory, ts, this->sample_window_s))
    {
        // Obtain the insertion idx
        const size_t idx = binarySearchIdx(this->odom_queue, ts);

        // If the idx is past the end or the stamp is unique, create a new entry
        if (idx >= this->odom_queue.size() ||
            std::abs(tstamp(this->odom_queue[idx]) - ts) > 1e-7)
        {
            this->odom_queue.emplace(
                this->odom_queue.begin() + idx,
                ts,
                Pose3{});
        }

        // Assign the target entry's value
        value(this->odom_queue[idx]) << pose;

        this->processQueue();
    }
}

template<typename P>
template<typename F>
void TrajectoryFilter<P>::addAbsolute<F>(const Pose3_<F>& pose, double ts)
{
    using namespace util::geom::cvt::ops;
    using namespace util::tsq;

    std::unique_lock lock{this->mtx};

    // Only insert if the stamp is newer than the window min
    if (inWindow(this->trajectory, ts, this->sample_window_s))
    {
        // Obtain the insertion idx
        const size_t idx = binarySearchIdx(this->absolute_queue, ts);

        // If the idx is past the end or the stamp is unique, create a new entry
        if (idx >= this->absolute_queue.size() ||
            std::abs(tstamp(this->absolute_queue[idx]) - ts) > 1e-7)
        {
            this->absolute_queue.emplace(
                this->absolute_queue.begin() + idx,
                ts,
                Pose3{});
        }

        // Assign the target entry's value
        value(this->absolute_queue) << pose;

    #if TRAJECTORY_FILTER_PRINT_DEBUG
        std::cout << "[TRAJECTORY FILTER]: Added measurement (" << ts
                  << ") at index " << idx << std::endl;
    #endif

        this->processQueue();
    }
}


template<typename P>
typename TrajectoryFilter<P>::template TStamped_<
    typename TrajectoryFilter<P>::MeasPair>
    TrajectoryFilter<P>::getFiltered() const
{
    std::unique_lock lock{this->result_mtx};

    return {this->latest_filtered_stamp, this->latest_filtered};
}

template<typename P>
void TrajectoryFilter<P>::getFiltered(TStamped_<MeasPair>& out) const
{
    std::unique_lock lock{this->result_mtx};

    out.first = this->latest_filtered_stamp;
    out.second = this->latest_filtered;
}


template<typename P>
const TrajectoryFilter<P>::FilterMetrics& TrajectoryFilter<P>::getStatus() const
{
    return this->metrics;
}


template<typename P>
void TrajectoryFilter<P>::processQueue()
{
    #if TRAJECTORY_FILTER_PRINT_DEBUG
    std::cout << "[TRAJECTORY FILTER]: Pre-processed queues >>\n";
    this->printQueues();
    #endif

    using namespace util::tsq;

    // 1. Update window min stamp by backtracking from newest sample among
    //    all three queues
    const double window_min =
        (std::max(
             {newestStamp(this->odom_queue),
              newestStamp(this->absolute_queue),
              newestStamp(this->trajectory)}) -
         this->sample_window_s);

    // 2. Remove raw samples that are older than the updated window
    trimToStamp(this->odom_queue, window_min);
    trimToStamp(this->absolute_queue, window_min);

    // 3. Processing depends on unused/new absolute measurements
    if (this->absolute_queue.empty())
    {
        return;
    }

    // this->metrics.last_filter_status = false;

    // 4. Create temporary queue for measurements that can't be matched -
    //    these get reinsterted afterwards
    std::deque<TStamped_<Pose3>> readd_queue;

    // 5. Obtain the index of the absolute measurement that matches the newest
    //    odom measurement
    const size_t abs_meas_i =
        binarySearchIdx(this->absolute_queue, newestStamp(this->odom_queue));

    // 6. Loop through all absolute measurements older than and including that
    //    matched to the newest odom measurement (oldest to newest)
    for (size_t i = this->absolute_queue.size(); i-- >= (abs_meas_i + 1);)
    {
        const TStamped_<Pose3>& abs_meas = this->absolute_queue[i];

        // Get the matched odometry idx for this sample
        const size_t odom_meas_i =
            binarySearchIdx(this->odom_queue, tstamp(abs_meas));
        // Get the sample itself
        const TStamped_<Pose3>& odom_meas = this->odom_queue[odom_meas_i];

        const bool is_exact_match =
            (std::abs(tstamp(abs_meas) - tstamp(odom_meas)) < 1e-7);
        const bool can_lerp = validLerpIdx(this->odom_queue, odom_meas_i);

        // If absolute measurement timestamp exactly matches that of the odom
        // sample, or the odometry queue can be sampled, continue
        if (is_exact_match || can_lerp)
        {
            // MeasPair holds the matched [possibly interpolated] measurements
            MeasPair meas_pair{};
            // Copy the absolute measurement since we are using it's timestamp directly
            meas_pair.absolute = value(abs_meas);

            // If timestamps matched exactly, directly assign odom measurement
            if (is_exact_match)
            {
                meas_pair.odometry = odom_meas;
            }
            // Otherwise, interpolate using the odometry queue
            else
            {
                const TStamped_<Pose3>& next_odom_meas =
                    this->odom_queue[odom_meas_i - 1];
                const double interp_ts =
                    (tstamp(abs_meas) - tstamp(odom_meas)) /
                    (tstamp(next_odom_meas) - tstamp(odom_meas));

                // Linear interpolation between samples to get result
                Pose3 manifold, interp_off;
                util::geom::relativeDiff(
                    manifold,
                    value(odom_meas),
                    value(next_odom_meas));
                util::geom::lerpSimple(interp_off, manifold, interp_ts);
                util::geom::compose(
                    meas_pair.odometry,
                    value(odom_meas),
                    interp_off);
            }

            // Use the absolute measurement's stamp to seed the matched data
            // insertion into the output queue
            const size_t traj_i =
                util::tsq::binarySearchIdx(this->trajectory, tstamp(abs_meas));

            // Emplace matched measurements
            this->trajectory.emplace(
                this->trajectory.begin() + traj_i,
                tstamp(abs_meas),
                std::move(meas_pair));

            // Get newly inserted measurements buffer
            MeasPair& curr_meas = value(this->trajectory[traj_i]);

            // Compute error from last sample, if it exists
            if (traj_i + 1 < this->trajectory.size())
            {
                const MeasPair& prev_meas = value(this->trajectory[traj_i + 1]);
                curr_meas.setErrorFrom(prev_meas);
            }
            // Update next sample's error based on the current sample, it it exists
            if (traj_i > 0)
            {
                MeasPair& next_meas = value(this->trajectory[traj_i - 1]);
                next_meas.setErrorFrom(curr_meas);
            }
        }
        // Otherwise, save measurement for later
        else
        {
            readd_queue.push_front(this->absolute_queue[i]);
        }
    }

    if (abs_meas_i < this->absolute_queue.size())
    {
        // If any absolute measurements were iterated over, remove them
        // from the queue
        this->absolute_queue.erase(
            this->absolute_queue.begin() + abs_meas_i,
            this->absolute_queue.end());

        // Reinsert any unused measurement so they can be tried again later
        if (readd_queue.size() > 0)
        {
            this->absolute_queue.insert(
                this->absolute_queue.end(),
                readd_queue.begin(),
                readd_queue.end());
        }
    }

    this->updateFilter();

    #if TRAJECTORY_FILTER_PRINT_DEBUG
    std::cout << "[TRAJECTORY FILTER]: Post-processed queues >>\n";
    this->printQueues();
    #endif
}

template<typename P>
void TrajectoryFilter<P>::updateFilter()
{
    using namespace util::tsq;

    // Ensure matches queue is the right size
    trimToStamp(
        this->trajectory,
        newestStamp(this->trajectory) - this->sample_window_s);
    // No processing to be done if there aren't any samples
    if (this->trajectory.empty())
    {
        return;
    }

    // TODO: if restart_thresh < filter_window, iterate through nodes and
    // remove all after first place where (delta > restart_thresh)

    const double dt =
        (tstamp(this->trajectory.front()) - tstamp(this->trajectory.back()));

    // This is ok since we already checked if trajectory queue was empty
    // Don't use the last sample since it could have uninitialized error metrics
    const size_t n_samples = (this->trajectory.size() - 1);

    // Compute average linear and angular errors
    double sum_linear_err = 0.;
    double sum_angular_err = 0.;
    for (size_t i = 0; i < n_samples; i++)
    {
        const MeasPair& x = value(this->trajectory[i]);

        sum_linear_err += x.linear_error;
        sum_angular_err += x.angular_error;
    }
    const double avg_linear_err = (sum_linear_err / n_samples);
    const double avg_angular_err = (sum_angular_err / n_samples);

    // If > 1 sample, compute error standard deviations
    double linear_err_stddev = 0.;
    double angular_err_stddev = 0.;
    if (n_samples > 1)
    {
        double sum_sq_linear_err = 0.;
        double sum_sq_angular_err = 0.;
        for (size_t i = 0; i < n_samples; i++)
        {
            const MeasPair& x = value(this->trajectory[i]);

            const double linear_diff = x.linear_error - avg_linear_err;
            const double angular_diff = x.angular_error - avg_angular_err;

            sum_sq_linear_err += (linear_diff * linear_diff);
            sum_sq_angular_err += (angular_diff * angular_diff);
        }

        linear_err_stddev = std::sqrt(sum_sq_linear_err / (n_samples - 1));
        angular_err_stddev = std::sqrt(sum_sq_angular_err / (n_samples - 1));

        // Determine if metrics are within parameterized thresholds
        const bool window_req = (dt >= this->filter_window_s);
        const bool linear_err_req =
            (avg_linear_err <= this->max_avg_linear_err);
        const bool angular_err_req =
            (avg_angular_err <= this->max_avg_angular_err);
        const bool linear_stddev_req =
            (linear_err_stddev <= this->max_linear_err_stddev);
        const bool angular_stddev_req =
            (angular_err_stddev <= this->max_angular_err_stddev);

        this->metrics.last_filter_status =
            (window_req &&         //
             linear_err_req &&     //
             angular_err_req &&    //
             linear_stddev_req &&  //
             angular_stddev_req);
        this->metrics.last_filter_mask =
            ((window_req << 0) +         //
             (linear_err_req << 1) +     //
             (angular_err_req << 2) +    //
             (linear_stddev_req << 3) +  //
             (angular_stddev_req << 4));
    }
    else
    {
        this->metrics.last_filter_status = false;
        this->metrics.last_filter_mask = 0;
    }

    if (this->metrics.last_filter_status)
    {
        std::unique_lock lock{this->result_mtx};

        this->last_filtered_meas = value(this->trajectory.front());
        this->last_filtered_stamp = tstamp(this->trajectory.front());
    }

    this->metrics.odom_q_len = this->odom_queue.size();
    this->metrics.abs_q_len = this->absolute_queue.size();
    this->metrics.meas_q_len = this->trajectory.size();

    this->metrics.last_search_window = dt;

    this->metrics.last_avg_linear_err = avg_linear_err;
    this->metrics.last_avg_angular_err = avg_angular_err;
    this->metrics.last_linear_err_stddev = linear_err_stddev;
    this->metrics.last_angular_err_stddev = angular_err_stddev;

    this->metrics.newest_meas_linear_err =
        value(this->trajectory.front()).linear_error;
    this->metrics.newest_meas_angular_err =
        value(this->trajectory.front()).angular_error;
}

template<typename P>
void TrajectoryFilter<P>::printQueues()
{
    using namespace util::tsq;

    std::ostringstream fmt;
    fmt << "\tOdom Q >> " << this->odom_queue.size() << ", [ ";
    for (const auto& elem : this->odom_queue)
    {
        fmt << tstamp(elem) << ", ";
    }
    fmt << "]\n\tAbsl Q >> " << this->absolute_queue.size() << ", [ ";
    for (const auto& elem : this->absolute_queue)
    {
        fmt << tstamp(elem) << ", ";
    }
    fmt << "]\n\tTraj Q >> " << this->trajectory.size() << ", [ ";
    for (const auto& elem : this->trajectory)
    {
        fmt << tstamp(elem) << ", ";
    }
    fmt << "]\n";
    std::cout << fmt.str();
    std::cout.flush();
}


template<typename P>
void TrajectoryFilter<P>::MeasPair::setErrorFrom(const MeasPair& prev)
{
    Pose3 odom_diff, meas_diff;
    util::geom::relativeDiff(odom_diff, prev.odometry, this->odometry);
    util::geom::relativeDiff(meas_diff, prev.measurement, this->measurement);

    this->linear_error = (odom_diff.vec - meas_diff.vec).norm();
    this->angular_error = odom_diff.quat.angularDistance(meas_diff.quat);
}

#endif

};  // namespace perception
};  // namespace csm
