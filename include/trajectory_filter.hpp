#pragma once

#include <deque>
#include <vector>
#include <mutex>
#include <utility>
#include <iterator>
#include <iostream>

#include "util.hpp"
#include "geometry.hpp"


template<typename Float_T = double>
class TrajectoryFilter
{
public:
    template<typename T>
    using Timestamped_ = std::pair<double, T>;

    using Pose3 = util::geom::Pose3<Float_T>;
    using ExportResult = std::tuple<double, Pose3, Pose3, double, double>;
public:
    TrajectoryFilter() = default;
    TrajectoryFilter(const TrajectoryFilter& ref) :
        filter_window_s{ ref.filter_window_s },
        restart_thresh_s{ ref.restart_thresh_s },
        min_sample_time_s{ ref.min_sample_time_s },
        linear_error_per_s_thresh{ ref.linear_error_per_s_thresh },
        angular_error_per_s_thresh{ ref.angular_error_per_s_thresh } {}
    ~TrajectoryFilter() = default;

    void setFilterWindow(double s);
    void setRestartThresh(double s);

    void addOdom(const Pose3& pose, double ts = 0.);
    void addMeasurement(const Pose3& meas, double ts = 0.);
    bool getFiltered(ExportResult& x);

    inline size_t odomQueueSize() const { std::unique_lock _lock{ this->mtx }; return this->source_odom.size(); }
    inline size_t measurementQueueSize() const { std::unique_lock _lock{ this->mtx }; return this->measurements_queue.size(); }
    inline size_t trajectoryQueueSize() const { std::unique_lock _lock{ this->mtx }; return this->trajectory.size(); }

protected:
    void processQueue();
    void updateFilter();

    struct TrajectoryNode
    {
        Pose3 measured;
        Pose3 reference;

        double linear_error{ 0. };
        double angular_error{ 0. };

        bool filter_state{ false };

        void computeError(const TrajectoryNode& prev);
    };

private:
    // deque of "source" odometry spanning our time window
    std::deque<Timestamped_<Pose3>> source_odom;
    // deque of "filtered" measurements that need to be processed (temporary holding)
    std::deque<Timestamped_<Pose3>> measurements_queue;

    std::deque<Timestamped_<TrajectoryNode>> trajectory;
    ExportResult latest;

    mutable std::mutex mtx;

    double filter_window_s{ 0.4 };
    double restart_thresh_s{ 0.5 };
    double min_sample_time_s{ 0.3 };

    double linear_error_per_s_thresh{ 2e-2 };
    double angular_error_per_s_thresh{ 5e-2 };

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


template<typename fT>
void TrajectoryFilter<fT>::addOdom(const Pose3& pose, double ts)
{
    std::unique_lock _lock{ this->mtx };
    if(util::tsq::inWindow(this->trajectory, ts, this->filter_window_s))
    {
        const size_t idx = util::tsq::binarySearchIdx(this->source_odom, ts);
        this->source_odom.insert(this->source_odom.begin() + idx, { ts, pose });

        this->processQueue();
    }
}

template<typename fT>
void TrajectoryFilter<fT>::addMeasurement(const Pose3& meas, double ts)
{
    std::unique_lock _lock{ this->mtx };
    if(util::tsq::inWindow(this->trajectory, ts, this->filter_window_s))
    {
        const size_t idx = util::tsq::binarySearchIdx(this->measurements_queue, ts);
        this->measurements_queue.insert(this->measurements_queue.begin() + idx, { ts, meas });

        this->processQueue();
    }
}

template<typename fT>
bool TrajectoryFilter<fT>::getFiltered(ExportResult& x)
{
    std::unique_lock _lock{ this->mtx };
    if(this->trajectory.empty()) return false;

    x = this->latest;
    return this->trajectory.front().second.filter_state;
}


template<typename fT>
void TrajectoryFilter<fT>::processQueue()
{
    const double window_min =
        std::max( {
            util::tsq::newestStamp(this->source_odom),
            util::tsq::newestStamp(this->measurements_queue),
            util::tsq::newestStamp(this->trajectory)            } ) - this->filter_window_s;

    util::tsq::trimToStamp(this->source_odom, window_min);
    util::tsq::trimToStamp(this->measurements_queue, window_min);

    if(this->measurements_queue.empty()) return;

    const int64_t idx = util::tsq::binarySearchIdx(this->measurements_queue, util::tsq::newestStamp(this->source_odom));
    for(int64_t i = this->measurements_queue.size() - 1; i >= idx; i--)     // TODO: fix this mess
    {
        auto& meas = this->measurements_queue[i];

        const size_t odom_prev = util::tsq::binarySearchIdx(this->source_odom, meas.first);
        if(util::tsq::validLerpIdx(this->source_odom, odom_prev))
        {
            const Timestamped_<Pose3>&
                before = this->source_odom[odom_prev],
                after = this->source_odom[odom_prev - 1];

            const double ts_interp = (meas.first - before.first) / (after.first - before.first);

            Pose3 manifold, interp_off;
            util::geom::relative_diff(manifold, before.second, after.second);

            // shortcut if close enough to either endpoint

            util::geom::lerpCurvature(interp_off, manifold, ts_interp);

            const size_t traj_idx = util::tsq::binarySearchIdx(this->trajectory, meas.first);
            this->trajectory.insert(this->trajectory.begin() + traj_idx, { meas.first, TrajectoryNode{} });

            TrajectoryNode& node = this->trajectory[traj_idx].second;

            node.reference.vec = before.second.vec + before.second.quat._transformVector(interp_off.vec);
            node.reference.quat = before.second.quat * interp_off.quat;
            node.measured = meas.second;

            if(traj_idx + 1 < this->trajectory.size())
            {
                node.computeError(this->trajectory[traj_idx + 1].second);
            }
            if(traj_idx > 0)
            {
                this->trajectory[traj_idx - 1].second.computeError(node);
            }
        }
    }
    this->measurements_queue.resize(idx);   // this may not be ideal?

    this->updateFilter();
}

template<typename fT>
void TrajectoryFilter<fT>::updateFilter()
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

        n.filter_state = false;
    }
    _linear /= _dt;
    _angular /= _dt;

    this->trajectory.front().second.filter_state = (
        _linear <= this->linear_error_per_s_thresh &&
        _angular <= this->angular_error_per_s_thresh );

    const auto& _node = this->trajectory.front();
    std::get<0>(this->latest) = _node.first;
    std::get<1>(this->latest) = _node.second.reference;
    std::get<2>(this->latest) = _node.second.measured;
    std::get<3>(this->latest) = _linear;
    std::get<4>(this->latest) = _angular;
}


template<typename fT>
void TrajectoryFilter<fT>::TrajectoryNode::computeError(const TrajectoryNode& prev)
{
    Pose3 odom_diff, meas_diff;
    util::geom::relative_diff(odom_diff, prev.reference, this->reference);
    util::geom::relative_diff(meas_diff, prev.measured, this->measured);

    this->linear_error = (odom_diff.vec - meas_diff.vec).norm();
    this->angular_error = odom_diff.quat.angularDistance(meas_diff.quat);
}
