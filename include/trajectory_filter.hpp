#pragma once

#include <deque>
#include <vector>
#include <mutex>
#include <utility>
#include <iterator>
#include <memory>
#include <type_traits>
#include <atomic>

#include "util.hpp"
#include "geometry.hpp"


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
    TrajectoryFilter(
        double filter_window_s = 0.5,
        double min_sample_time_s = 0.3,
        double linear_error_per_s_thresh = 2e-2,
        double angular_error_per_s_thresh = 2e-2
    ) :
        filter_window_s{ filter_window_s },
        min_sample_time_s{ min_sample_time_s },
        linear_error_per_s_thresh{ linear_error_per_s_thresh },
        angular_error_per_s_thresh{ angular_error_per_s_thresh } {}

    TrajectoryFilter(const TrajectoryFilter& ref) :
        filter_window_s{ ref.filter_window_s },
        min_sample_time_s{ ref.min_sample_time_s },
        linear_error_per_s_thresh{ ref.linear_error_per_s_thresh },
        angular_error_per_s_thresh{ ref.angular_error_per_s_thresh } {}

    ~TrajectoryFilter() = default;

    void setFilterWindow(double s);
    void setRestartThresh(double s);

    void addOdom(const Pose3& pose, double ts = 0.);
    void addMeasurement(const Meas_Ptr& meas, double ts = 0.);
    void addMeasurement(MultiMeas& meas, double ts = 0.);

    void getFiltered(Timestamped_<KeyPose>& out) const;
    Timestamped_<KeyPose> getFiltered() const;

    inline bool lastFilterStatus() const { return this->metrics.last_filter_status.load(); }
    inline size_t odomQueueSize() const { return this->metrics.odom_q_len.load(); }
    inline size_t measurementQueueSize() const { return this->metrics.meas_q_len.load(); }
    inline size_t trajectoryQueueSize() const { return this->metrics.traj_len.load(); }
    inline double lastFilterDelta() const { return this->metrics.last_traj_delta.load(); }
    inline double lastFilterLinear() const { return this->metrics.last_linear_err.load(); }
    inline double lastFilterAngular() const { return this->metrics.last_angular_err.load(); }

protected:
    void processQueue();
    void updateFilter();

private:
    // deque of "source" odometry spanning our time window
    std::deque<Timestamped_<Pose3>> odom_queue;
    // temporary queue for measurements that need to be processed
    std::deque<Timestamped_<MultiMeas>> measurements_queue;
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
            odom_q_len{ 0 },
            meas_q_len{ 0 },
            traj_len{ 0 };
        std::atomic<double>
            last_traj_delta{ 0. },
            last_linear_err{ 0. },
            last_angular_err{ 0. };
    }
    metrics;

    const double filter_window_s{ 0.4 };
    const double min_sample_time_s{ 0.3 };
    const double linear_error_per_s_thresh{ 2e-2 };
    const double angular_error_per_s_thresh{ 5e-2 };

};



namespace util
{
    namespace tsq   // TimeStamp "Q" (queue)
    {
        template<typename T> using TSQ = std::deque<std::pair<double, T>>;

        /** Returns the index of the element with timestamp just greater than that provided - ei. idx - 1 is the element whose timestamp is just less.
         * This is the same index which should be used if inserting into the queue such that it stays sorted. */
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


template<typename M, typename fT>
void TrajectoryFilter<M, fT>::addOdom(const Pose3& pose, double ts)
{
    std::unique_lock _lock{ this->mtx };
    if(util::tsq::inWindow(this->trajectory, ts, this->filter_window_s))
    {
        const size_t idx = util::tsq::binarySearchIdx(this->odom_queue, ts);
        this->odom_queue.insert(this->odom_queue.begin() + idx, { ts, pose });

        this->processQueue();
    }
}

template<typename M, typename fT>
void TrajectoryFilter<M, fT>::addMeasurement(const Meas_Ptr& meas, double ts)
{
    std::unique_lock _lock{ this->mtx };
    if(util::tsq::inWindow(this->trajectory, ts, this->filter_window_s))
    {
        const size_t idx = util::tsq::binarySearchIdx(this->measurements_queue, ts);
        if(idx >= this->measurements_queue.size() || ts != this->measurements_queue[idx].first)
        {
            this->measurements_queue.insert(this->measurements_queue.begin() + idx, { ts, {} });
        }
        this->measurements_queue[idx].second.push_back(meas);

        this->processQueue();
    }
}

template<typename M, typename fT>
void TrajectoryFilter<M, fT>::addMeasurement(MultiMeas& meas, double ts)
{
    std::unique_lock _lock{ this->mtx };
    if(util::tsq::inWindow(this->trajectory, ts, this->filter_window_s))
    {
        const size_t idx = util::tsq::binarySearchIdx(this->measurements_queue, ts);
        if(idx >= this->measurements_queue.size() || ts != this->measurements_queue[idx].first)
        {
            this->measurements_queue.insert(this->measurements_queue.begin() + idx, { ts, {} });
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
TrajectoryFilter<M, fT>::Timestamped_<typename TrajectoryFilter<M, fT>::KeyPose> TrajectoryFilter<M, fT>::getFiltered() const
{
    std::unique_lock _lock{ this->result_mtx };
    return { this->latest_filtered_stamp, this->latest_filtered };
}


template<typename M, typename fT>
void TrajectoryFilter<M, fT>::processQueue()
{
    const double window_min =
        std::max( {
            util::tsq::newestStamp(this->odom_queue),
            util::tsq::newestStamp(this->measurements_queue),
            util::tsq::newestStamp(this->trajectory)            } ) - this->filter_window_s;

    util::tsq::trimToStamp(this->odom_queue, window_min);
    util::tsq::trimToStamp(this->measurements_queue, window_min);

    if(this->measurements_queue.empty()) return;

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
    }
    this->measurements_queue.resize(idx);   // this may not be ideal?

    this->updateFilter();
}

template<typename M, typename fT>
void TrajectoryFilter<M, fT>::updateFilter()
{
    util::tsq::trimToStamp(this->trajectory, util::tsq::newestStamp(this->trajectory) - this->filter_window_s);
    if(this->trajectory.empty()) return;

    // TODO: if restart_thresh < filter_window, iterate through nodes and remove all after first place where (delta > restart_thresh)

    const double _dt = this->trajectory.front().first - this->trajectory.back().first;
    if(_dt < this->min_sample_time_s) return;

    double _linear = 0., _angular = 0.;
    for(size_t i = 0; i < this->trajectory.size() - 1; i++)
    {
        auto& n = this->trajectory[i].second;

        _linear += n.linear_error;
        _angular += n.angular_error;
    }
    _linear /= _dt;
    _angular /= _dt;

    this->metrics.last_filter_status = (_linear <= this->linear_error_per_s_thresh && _angular <= this->angular_error_per_s_thresh);
    if(this->metrics.last_filter_status)
    {
        this->result_mtx.lock();
        KeyPose& _front = this->trajectory.front().second;
        this->latest_filtered.measurement = _front.measurement;
        this->latest_filtered.odometry = _front.odometry;
        this->latest_filtered.linear_error = _linear;
        this->latest_filtered.angular_error = _angular;
        this->latest_filtered_stamp = this->trajectory.front().first;
        this->result_mtx.unlock();
    }

    this->metrics.odom_q_len = this->odom_queue.size();
    this->metrics.meas_q_len = this->measurements_queue.size();
    this->metrics.traj_len = this->trajectory.size();
    this->metrics.last_traj_delta = _dt;
    this->metrics.last_linear_err = _linear;
    this->metrics.last_angular_err = _angular;
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
