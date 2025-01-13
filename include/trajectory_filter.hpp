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
#include <vector>
#include <mutex>
#include <utility>
#include <iterator>
#include <memory>
#include <type_traits>
#include <atomic>

#include <iostream>
#include <sstream>

#include "tsq.hpp"
#include "util.hpp"
#include "geometry.hpp"


namespace csm
{
namespace perception
{

template<
    typename Meas_T,
    typename Float_T = double>
class TrajectoryFilter
{
    static_assert(std::is_convertible_v<Meas_T, util::geom::Pose3<Float_T>&>);
public:
    template<typename T>
    using Timestamped_ = std::pair<double, T>;

    using Meas_Ptr = std::shared_ptr<Meas_T>;
    using MultiMeas = std::vector<Meas_Ptr>;
    using Pose3 = util::geom::Pose3<Float_T>;

public:
    struct KeyPose
    {
        std::shared_ptr<Meas_T> measurement;
        Pose3 odometry;

        double linear_error{ 0. };
        double angular_error{ 0. };

        void computeError(const KeyPose& prev);
    };

public:
    TrajectoryFilter() = default;
    TrajectoryFilter(const TrajectoryFilter& ref) :
        sample_window_s{ ref.sample_window_s },
        filter_window_s{ ref.filter_window_s },
        avg_linear_error_thresh{ ref.avg_linear_error_thresh },
        avg_angular_error_thresh{ ref.avg_angular_error_thresh },
        max_linear_error_dev{ max_linear_error_dev },
        max_angular_error_dev{ max_angular_error_dev } {}

    ~TrajectoryFilter() = default;

    void applyParams(
        double sample_window_s = 0.5,
        double filter_window_s = 0.3,
        double avg_linear_error_thresh = 2e-2,
        double avg_angular_error_thresh = 2e-2,
        double max_linear_error_dev = 4e-2,
        double max_angular_error_dev = 4e-2 );

    void addOdom(const Pose3& pose, double ts = 0.);
    void addMeasurement(const Meas_Ptr& meas, double ts = 0.);
    void addMeasurement(MultiMeas& meas, double ts = 0.);

    void getFiltered(Timestamped_<KeyPose>& out) const;
    Timestamped_<KeyPose> getFiltered() const;

    inline bool lastFilterStatus() const { return this->metrics.last_filter_status.load(); }
    inline size_t lastFilterMask() const { return this->metrics.last_filter_mask.load(); }
    inline size_t odomQueueSize() const { return this->metrics.odom_q_len.load(); }
    inline size_t measurementQueueSize() const { return this->metrics.meas_q_len.load(); }
    inline size_t trajectoryQueueSize() const { return this->metrics.traj_len.load(); }
    inline double lastFilterWindow() const { return this->metrics.last_traj_window.load(); }
    inline double lastAvgLinearError() const { return this->metrics.last_avg_linear_err.load(); }
    inline double lastAvgAngularError() const { return this->metrics.last_avg_angular_err.load(); }
    inline double lastLinearDeviation() const { return this->metrics.last_linear_dev.load(); }
    inline double lastAngularDeviation() const { return this->metrics.last_angular_dev.load(); }
    inline double lastLinearDelta() const { return this->metrics.last_linear_delta.load(); }
    inline double lastAngularDelta() const { return this->metrics.last_angular_delta.load(); }

protected:
    void processQueue();
    void updateFilter();
    void printQueues();

private:
    // deque of "source" odometry spanning our time window
    std::deque<Timestamped_<Pose3>> odom_queue;
    // temporary queue for measurements that need to be processed
    std::deque<Timestamped_<MultiMeas>> measurements_queue, meas_temp_queue;
    // active trajectory queue
    std::deque<Timestamped_<KeyPose>> trajectory;

    std::mutex mtx;
    mutable std::mutex result_mtx;

    KeyPose latest_filtered;
    double latest_filtered_stamp{ 0. };

    struct
    {
        std::atomic<bool>
            last_filter_status{ false };
        std::atomic<size_t>
            last_filter_mask{ 0 },
            odom_q_len{ 0 },
            meas_q_len{ 0 },
            traj_len{ 0 };
        std::atomic<double>
            last_traj_window{ 0. },
            last_avg_linear_err{ 0. },
            last_avg_angular_err{ 0. },
            last_linear_dev{ 0. },
            last_angular_dev{ 0. },
            last_linear_delta{ 0. },
            last_angular_delta{ 0. };
    }
    metrics;

    double sample_window_s{ 0.4 };
    double filter_window_s{ 0.3 };
    double avg_linear_error_thresh{ 2e-2 };
    double avg_angular_error_thresh{ 5e-2 };
    double max_linear_error_dev{ 4e-2 };
    double max_angular_error_dev{ 4e-2 };

};


template<typename M, typename fT>
void TrajectoryFilter<M, fT>::applyParams(
    double sample_window_s,
    double filter_window_s,
    double avg_linear_error_thresh,
    double avg_angular_error_thresh,
    double max_linear_error_dev,
    double max_angular_error_dev)
{
    std::unique_lock _lock{ this->mtx };
    this->sample_window_s = sample_window_s;
    this->filter_window_s = filter_window_s;
    this->avg_linear_error_thresh = avg_linear_error_thresh;
    this->avg_angular_error_thresh = avg_angular_error_thresh;
    this->max_linear_error_dev = max_linear_error_dev;
    this->max_angular_error_dev = max_angular_error_dev;
}

template<typename M, typename fT>
void TrajectoryFilter<M, fT>::addOdom(const Pose3& pose, double ts)
{
    std::unique_lock _lock{ this->mtx };
    if(util::tsq::inWindow(this->trajectory, ts, this->sample_window_s))
    {
        const size_t idx = util::tsq::binarySearchIdx(this->odom_queue, ts);
        this->odom_queue.emplace(this->odom_queue.begin() + idx, ts, pose);

        this->processQueue();
    }
}

template<typename M, typename fT>
void TrajectoryFilter<M, fT>::addMeasurement(const Meas_Ptr& meas, double ts)
{
    std::unique_lock _lock{ this->mtx };
    if(util::tsq::inWindow(this->trajectory, ts, this->sample_window_s))
    {
        const size_t idx = util::tsq::binarySearchIdx(this->measurements_queue, ts);
        if(idx >= this->measurements_queue.size() || ts != this->measurements_queue[idx].first)
        {
            this->measurements_queue.insert(this->measurements_queue.begin() + idx, { ts, {} });
        }
        this->measurements_queue[idx].second.push_back(meas);

        // std::cout << "[TRAJECTORY FILTER]: Added measurement (" << ts << ") at index " << idx << std::endl;

        this->processQueue();
    }
}

template<typename M, typename fT>
void TrajectoryFilter<M, fT>::addMeasurement(MultiMeas& meas, double ts)
{
    std::unique_lock _lock{ this->mtx };
    if(util::tsq::inWindow(this->trajectory, ts, this->sample_window_s))
    {
        const size_t idx = util::tsq::binarySearchIdx(this->measurements_queue, ts);
        if(idx >= this->measurements_queue.size() || ts != this->measurements_queue[idx].first)
        {
            this->measurements_queue.emplace(this->measurements_queue.begin() + idx, ts, {});
            std::swap(this->measurements_queue[idx].second, meas);
        }
        else
        {
            auto& _vec = this->measurements_queue[idx].second;
            _vec.insert(_vec.end(), meas.begin(), meas.end());
        }

        this->processQueue();
    }
}

template<typename M, typename fT>
void TrajectoryFilter<M, fT>::getFiltered(Timestamped_<KeyPose>& out) const
{
    std::unique_lock _lock{ this->result_mtx };
    out.first = this->latest_filtered_stamp;
    out.second = this->latest_filtered;
}

template<typename M, typename fT>
typename TrajectoryFilter<M, fT>:: template Timestamped_<typename TrajectoryFilter<M, fT>::KeyPose> TrajectoryFilter<M, fT>::getFiltered() const
{
    std::unique_lock _lock{ this->result_mtx };
    return { this->latest_filtered_stamp, this->latest_filtered };
}


template<typename M, typename fT>
void TrajectoryFilter<M, fT>::processQueue()
{
    // std::cout << "[TRAJECTORY FILTER]: Pre-processed queues >>\n";
    // this->printQueues();

    const double window_min =
        std::max( {
            util::tsq::newestStamp(this->odom_queue),
            util::tsq::newestStamp(this->measurements_queue),
            util::tsq::newestStamp(this->trajectory)            } ) - this->sample_window_s;

    util::tsq::trimToStamp(this->odom_queue, window_min);
    util::tsq::trimToStamp(this->measurements_queue, window_min);

    if(this->measurements_queue.empty()) return;

    this->meas_temp_queue.clear();
    const size_t idx = util::tsq::binarySearchIdx(this->measurements_queue, util::tsq::newestStamp(this->odom_queue));
    for(int64_t i = this->measurements_queue.size() - 1; i >= (int64_t)idx; i--)     // TODO: fix this mess
    {
        Timestamped_<MultiMeas>& meas = this->measurements_queue[i];
        if(meas.second.size() == 0) continue;

        // find corresponding odometry to lerp between
        const size_t odom_prev = util::tsq::binarySearchIdx(this->odom_queue, meas.first);
        if(util::tsq::validLerpIdx(this->odom_queue, odom_prev))
        {
            const Timestamped_<Pose3>&
                before = this->odom_queue[odom_prev],
                after = this->odom_queue[odom_prev - 1];

            const double interp_ts = (meas.first - before.first) / (after.first - before.first);

            Pose3 manifold, interp_off;
            util::geom::relative_diff(manifold, before.second, after.second);   // get the relative transform (pose form)
            // TODO: shortcut if close enough to either endpoint
            util::geom::lerpSimple(interp_off, manifold, interp_ts); // interpolate assuming constant curvature within the manifold

            const size_t traj_idx = util::tsq::binarySearchIdx(this->trajectory, meas.first);
            this->trajectory.insert(this->trajectory.begin() + traj_idx, { meas.first, KeyPose{} });

            KeyPose& node = this->trajectory[traj_idx].second;
            const KeyPose& prev_node = this->trajectory[traj_idx + 1].second;
            KeyPose* next_node = (traj_idx > 0) ? &this->trajectory[traj_idx - 1].second : nullptr;

            util::geom::compose(node.odometry, before.second, interp_off);

            const bool has_prev_node = (traj_idx + 1 < this->trajectory.size());
            if(!has_prev_node && !next_node)
            {
                node.measurement = meas.second[0];
            }
            else
            {
                node.measurement = nullptr;

                double best_lerr = 0., best_aerr = 0., best_post_lerr = 0., best_post_aerr = 0.;
                Meas_Ptr best_meas = nullptr;
                for(size_t i = 0; i < meas.second.size(); i++)
                {
                    node.measurement = meas.second[i];
                    double post_lerr = 0., post_aerr = 0.;
                    if(has_prev_node)
                    {
                        node.computeError(prev_node);
                    }
                    if(next_node)
                    {
                        next_node->computeError(node);
                        post_lerr = next_node->linear_error;
                        post_aerr = next_node->angular_error;
                    }

                    const double score = post_lerr + post_aerr + node.linear_error + node.angular_error;
                    if(best_meas == nullptr || score < (best_lerr + best_aerr + best_post_lerr + best_post_aerr))
                    {
                        best_meas = node.measurement;
                        best_lerr = node.linear_error;
                        best_aerr = node.angular_error;
                        best_post_lerr = post_lerr;
                        best_post_aerr = post_aerr;
                    }
                }
                node.measurement = best_meas;
                node.linear_error = best_lerr;
                node.angular_error = best_aerr;
                if(next_node)    // reapply
                {
                    next_node->linear_error = best_post_lerr;
                    next_node->angular_error = best_post_aerr;
                }
            }
        }
        else
        {
            this->meas_temp_queue.push_front(this->measurements_queue[i]);
        }
    }
    std::swap(this->measurements_queue, this->meas_temp_queue);

    this->updateFilter();

    // std::cout << "[TRAJECTORY FILTER]: Post-processed queues >>\n";
    // this->printQueues();
}

template<typename M, typename fT>
void TrajectoryFilter<M, fT>::updateFilter()
{
    util::tsq::trimToStamp(this->trajectory, util::tsq::newestStamp(this->trajectory) - this->sample_window_s);
    if(this->trajectory.empty()) return;

    // TODO: if restart_thresh < filter_window, iterate through nodes and remove all after first place where (delta > restart_thresh)

    const double _dt = this->trajectory.front().first - this->trajectory.back().first;
    // if(_dt < this->filter_window_s) return;

    double _linear = 0., _angular = 0.;
    const size_t n_samples = this->trajectory.size() - 1;
    for(size_t i = 0; i < n_samples; i++)
    {
        auto& n = this->trajectory[i].second;

        _linear += n.linear_error;
        _angular += n.angular_error;
    }
    const double norm_linear = _linear / _dt;
    const double norm_angular = _angular / _dt;
    double stddev_linear = 0., stddev_angular = 0.;
    if(n_samples > 1)
    {
        _linear /= n_samples;
        _angular /= n_samples;
        for(size_t i = 0; i < n_samples; i++)
        {
            auto& n = this->trajectory[i].second;

            const double _ld = n.linear_error - _linear;
            const double _ad = n.angular_error - _angular;
            stddev_linear += _ld * _ld;
            stddev_angular += _ad * _ad;
        }
        stddev_linear /= (n_samples - 1);
        stddev_angular /= (n_samples - 1);
        stddev_linear = std::sqrt(stddev_linear);
        stddev_angular = std::sqrt(stddev_angular);

        const bool
            window_req = _dt >= this->filter_window_s,
            linear_req = norm_linear <= this->avg_linear_error_thresh,
            angular_req = norm_angular <= this->avg_angular_error_thresh,
            linear_var_req = stddev_linear <= this->max_linear_error_dev,
            angular_var_req = stddev_angular <= this->max_angular_error_dev;

        // std::cout << "\nLINEAR: " << norm_linear << " vs. " << this->avg_linear_error_thresh << "\nANGULAR: " << norm_angular << " vs. " << this->avg_angular_error_thresh << std::endl;

        this->metrics.last_filter_status =
            window_req &&
            linear_req &&
            angular_req &&
            linear_var_req &&
            angular_var_req;
        this->metrics.last_filter_mask =
            (window_req << 0) +
            (linear_req << 1) +
            (angular_req << 2) +
            (linear_var_req << 3) +
            (angular_var_req << 4);
    }
    else
    {
        this->metrics.last_filter_status = false;
        this->metrics.last_filter_mask = 0;
    }

    if(this->metrics.last_filter_status)
    {
        this->result_mtx.lock();
        KeyPose& _front = this->trajectory.front().second;
        this->latest_filtered.measurement = _front.measurement;
        this->latest_filtered.odometry = _front.odometry;
        this->latest_filtered.linear_error = norm_linear;
        this->latest_filtered.angular_error = norm_angular;
        this->latest_filtered_stamp = this->trajectory.front().first;
        this->result_mtx.unlock();
    }

    this->metrics.odom_q_len = this->odom_queue.size();
    this->metrics.meas_q_len = this->measurements_queue.size();
    this->metrics.traj_len = this->trajectory.size();
    this->metrics.last_traj_window = _dt;
    this->metrics.last_avg_linear_err = norm_linear;
    this->metrics.last_avg_angular_err = norm_angular;
    this->metrics.last_linear_dev = stddev_linear;
    this->metrics.last_angular_dev = stddev_angular;
    this->metrics.last_linear_delta = this->trajectory.front().second.linear_error;
    this->metrics.last_angular_delta = this->trajectory.front().second.angular_error;
}

template<typename M, typename fT>
void TrajectoryFilter<M, fT>::printQueues()
{
    std::ostringstream fmt;
    fmt << "\tOdom Q >> " << this->odom_queue.size() << ", [ ";
    for(const auto& elem : this->odom_queue)
    {
        fmt << elem.first << ", ";
    }
    fmt << "]\n\tMeas Q >> " << this->measurements_queue.size() << ", [ ";
    for(const auto& elem : this->measurements_queue)
    {
        fmt << elem.first << ", ";
    }
    fmt << "]\n\tTraj Q >> " << this->trajectory.size() << ", [ ";
    for(const auto& elem : this->trajectory)
    {
        fmt << elem.first << ", ";
    }
    fmt << "]\n";
    std::cout << fmt.str();
    std::cout.flush();
}


template<typename M, typename fT>
void TrajectoryFilter<M, fT>::KeyPose::computeError(const KeyPose& prev)
{
    Pose3 odom_diff, meas_diff;
    util::geom::relative_diff(odom_diff, prev.odometry, this->odometry);
    util::geom::relative_diff(meas_diff, static_cast<const Pose3&>(*prev.measurement), static_cast<const Pose3&>(*this->measurement));

    this->linear_error = (odom_diff.vec - meas_diff.vec).norm();
    this->angular_error = odom_diff.quat.angularDistance(meas_diff.quat);
}

};
};
