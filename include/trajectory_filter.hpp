#pragma once

#include <deque>
#include <vector>
#include <mutex>
#include <utility>
#include <iterator>

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
    std::mutex odom_mtx, meas_mtx;

    double filter_window_s{ 1. };
    double restart_thresh_s{ 0.5 };

};



namespace util
{
    namespace tsq   // TimeStamp "Q" (queue)
    {
        /** Returns the index of the element with timestamp just greater than that provided - ei. idx - 1 is just less.
         * This is the same index which should be used if inserting into the queue such that it stays sorted. */
        template<typename T>
        size_t binarySearchIdx(const std::deque<std::pair<double, T>>& q, double ts)
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
    };

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
    this->odom_mtx.lock();

    const size_t idx = util::tsq::binarySearchIdx(this->source_odom, ts);
    this->source_odom.insert(this->source_odom.begin() + idx, { ts, pose });

    this->source_odom.resize(
        util::tsq::binarySearchIdx(this->source_odom, this->source_odom.front().first - this->filter_window_s) );

    // check to use queued measurements

    this->odom_mtx.unlock();
}

template<typename fT>
void TrajectoryFilter_<fT>::addMeasurement(const Pose3& meas, double ts)
{
    this->odom_mtx.lock();
    this->meas_mtx.lock();

    if(ts > this->source_odom.front().first)
    {
        const size_t idx = util::tsq::binarySearchIdx(this->measurements_queue, ts);
        this->measurements_queue.insert(this->measurements_queue.begin() + idx, { ts, meas });
    }


    // calc bounds
    // remove old
    // insert new
    // update filter*
}
