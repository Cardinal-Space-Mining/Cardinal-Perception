#pragma once

#include <deque>
#include <vector>
#include <mutex>
#include <utility>
#include <iterator>
#include <memory>
#include <type_traits>
#include <atomic>

#include "tsq.hpp"
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
    TrajectoryFilter() = default;
    TrajectoryFilter(const TrajectoryFilter& ref) :
        filter_window_s{ ref.filter_window_s },
        min_sample_time_s{ ref.min_sample_time_s },
        avg_linear_error_thresh{ ref.avg_linear_error_thresh },
        avg_angular_error_thresh{ ref.avg_angular_error_thresh },
        max_linear_error_variance{ max_linear_error_variance },
        max_angular_error_variance{ max_angular_error_variance } {}

    ~TrajectoryFilter() = default;

    void applyParams(
        double filter_window_s = 0.5,
        double min_sample_time_s = 0.3,
        double avg_linear_error_thresh = 2e-2,
        double avg_angular_error_thresh = 2e-2,
        double max_linear_error_variance = 1e-5,
        double max_angular_error_variance = 1e-5 );

    void addOdom(const Pose3& pose, double ts = 0.);
    void addMeasurement(const Meas_Ptr& meas, double ts = 0.);
    void addMeasurement(MultiMeas& meas, double ts = 0.);

    void getFiltered(Timestamped_<KeyPose>& out) const;
    Timestamped_<KeyPose> getFiltered() const;

    inline bool lastFilterStatus() const { return this->metrics.last_filter_status.load(); }
    inline size_t odomQueueSize() const { return this->metrics.odom_q_len.load(); }
    inline size_t measurementQueueSize() const { return this->metrics.meas_q_len.load(); }
    inline size_t trajectoryQueueSize() const { return this->metrics.traj_len.load(); }
    inline double lastFilterWindow() const { return this->metrics.last_traj_window.load(); }
    inline double lastAvgLinearError() const { return this->metrics.last_avg_linear_err.load(); }
    inline double lastAvgAngularError() const { return this->metrics.last_avg_angular_err.load(); }
    inline double lastLinearVariance() const { return this->metrics.last_linear_variance.load(); }
    inline double lastAngularVariance() const { return this->metrics.last_angular_variance.load(); }
    inline double lastLinearDelta() const { return this->metrics.last_linear_delta.load(); }
    inline double lastAngularDelta() const { return this->metrics.last_angular_delta.load(); }

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
            last_traj_window{ 0. },
            last_avg_linear_err{ 0. },
            last_avg_angular_err{ 0. },
            last_linear_variance{ 0. },
            last_angular_variance{ 0. },
            last_linear_delta{ 0. },
            last_angular_delta{ 0. };
    }
    metrics;

    double filter_window_s{ 0.4 };
    double min_sample_time_s{ 0.3 };
    double avg_linear_error_thresh{ 2e-2 };
    double avg_angular_error_thresh{ 5e-2 };
    double max_linear_error_variance{ 1e-5 };
    double max_angular_error_variance{ 1e-5 };

};


template<typename M, typename fT>
void TrajectoryFilter<M, fT>::applyParams(
    double filter_window_s,
    double min_sample_time_s,
    double avg_linear_error_thresh,
    double avg_angular_error_thresh,
    double max_linear_error_variance,
    double max_angular_error_variance)
{
    std::unique_lock _lock{ this->mtx };
    this->filter_window_s = filter_window_s;
    this->min_sample_time_s = min_sample_time_s;
    this->avg_linear_error_thresh = avg_linear_error_thresh;
    this->avg_angular_error_thresh = avg_angular_error_thresh;
    this->max_linear_error_variance = max_linear_error_variance;
    this->max_angular_error_variance = max_angular_error_variance;
}

template<typename M, typename fT>
void TrajectoryFilter<M, fT>::addOdom(const Pose3& pose, double ts)
{
    std::unique_lock _lock{ this->mtx };
    if(util::tsq::inWindow(this->trajectory, ts, this->filter_window_s))
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
    const size_t n_samples = this->trajectory.size() - 1;
    for(size_t i = 0; i < n_samples; i++)
    {
        auto& n = this->trajectory[i].second;

        _linear += n.linear_error;
        _angular += n.angular_error;
    }
    const double norm_linear = _linear / _dt;
    const double norm_angular = _angular / _dt;
    double var_linear = 0., var_angular = 0.;
    if(n_samples > 1)
    {
        _linear /= n_samples;
        _angular /= n_samples;
        for(size_t i = 0; i < n_samples; i++)
        {
            auto& n = this->trajectory[i].second;

            const double _ld = n.linear_error - _linear;
            const double _ad = n.angular_error - _angular;
            var_linear += _ld * _ld;
            var_angular += _ad * _ad;
        }
        var_linear /= (n_samples - 1);
        var_angular /= (n_samples - 1);

        this->metrics.last_filter_status = 
            norm_linear <= this->avg_linear_error_thresh &&
            norm_angular <= this->avg_angular_error_thresh &&
            var_linear <= this->max_linear_error_variance &&
            var_angular <= this->max_angular_error_variance;
    }
    else
    {
        this->metrics.last_filter_status = false;
    }

    if(this->metrics.last_filter_status)
    {
        this->result_mtx.lock();
        KeyPose& _front = this->trajectory.front().second;
        this->latest_filtered.measurement = _front.measurement;
        this->latest_filtered.odometry = _front.odometry;
        this->latest_filtered.linear_error = norm_linear;
        this->latest_filtered.angular_error = norm_linear;
        this->latest_filtered_stamp = this->trajectory.front().first;
        this->result_mtx.unlock();
    }

    this->metrics.odom_q_len = this->odom_queue.size();
    this->metrics.meas_q_len = this->measurements_queue.size();
    this->metrics.traj_len = this->trajectory.size();
    this->metrics.last_traj_window = _dt;
    this->metrics.last_avg_linear_err = norm_linear;
    this->metrics.last_avg_angular_err = norm_angular;
    this->metrics.last_linear_variance = var_linear;
    this->metrics.last_angular_variance = var_angular;
    this->metrics.last_linear_delta = this->trajectory.front().second.linear_error;
    this->metrics.last_angular_delta = this->trajectory.front().second.angular_error;
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
