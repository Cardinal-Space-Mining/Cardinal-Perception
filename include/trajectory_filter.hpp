#pragma once

#include <deque>
#include <vector>
#include <mutex>
#include <utility>

#include "util.hpp"
#include "geometry.hpp"


template<typename Float_T>
class TrajectoryFilter_
{
    template<typename T>
    using Timestamped_ = std::pair<double, T>;

    using Pose3 = util::geom::Pose3<Float_T>;
public:
    TrajectoryFilter() = default;
    ~TrajectoryFilter() = default;

    void setFilterWindow(double s);
    void setRestartThresh(double s);

    void addOdom(const Pose3& pose, double ts = 0.);
    void addMeasurement(const Pose3& meas, double ts = 0.);

protected:
    struct Measurement
    {
        Pose3 measured;
        Pose3 reference;

        double linear_error;
        double angular_error;
        double stamp;
    };

private:
    // deque of "source" odometry spanning our time window
    std::deque<Timestamped_<Pose3>> source_odom;
    // deque of "filtered" measurements that need to be processed (temporary holding)
    std::deque<Timestamped_<Pose3>> measurements_queue;

    std::deque<Measurement> trajectory;
    std::mutex mtx;

    double filter_window_s{ 1. };
    double restart_thresh_s{ 0.5 };

};



namespace util
{
    /** Returns the index of the element with timestamp just greater than that provided - ei. idx - 1 is just less.
     * This is the same index which should be used if inserting into the queue such that it stays sorted. */
    template<typename T>
    size_t binarySearchStamp(const std::deque<std::pair<double, T>>& q, double ts)
    {
        if(q.empty()) return 0;
        if(ts >= q.front().first) return 0;
        if(ts <= q.back().first) return q.size();

        size_t after = 0, before = q.size() - 1;
        for(;after <= before;)
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

            if(after - before <= 1)
            {
                return before;
            }
        }
        return 0;
    }

    template<typename T>
    void rel_diff(
        util::geom::Pose3<T>& rel,
        const util::geom::Pose3<T>& from,
        const util::geom::Pose3<T>& to)
    {
        util::geom::component_diff(rel, from, to);
        rel.vec = from.quat.inverse()._transformVector(rel.vec);
    }
};


template<typename fT>
void TrajectoryFilter_<fT>::addOdom(const Pose3& pose, double ts)
{
    if(!this->source_odom.empty())
    {
        const double curr_ts = std::max(ts, this->source_odom.front().first);
        const double min_ts = curr_ts - this->filter_window_s;

        for(;!this->source_odom.empty();)
        {
            if(this->source_odom.back().first < min_ts) this->source_odom.pop_back();
        }
    }

    if(this->source_odom.empty())
    {
        this->source_odom.push_front({ ts, pose });
    }
    else
    {
        const double curr_newest_ts = this->source_odom.front().first;

        if(ts > curr_newest_ts) this->source_odom.push_front({ ts, pose });
        else if(ts > curr_newest_ts - this->filter_window_s)
        {

        }
    }
}

template<typename fT>
void TrajectoryFilter_<fT>::addMeasurement(const Pose3& meas, double ts)
{
    // calc bounds
    // remove old
    // insert new
    // update filter*
}
