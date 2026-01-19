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

#include <mutex>
#include <atomic>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sensor_msgs/msg/imu.hpp>

#include <util/geometry.hpp>
#include <util/time_cvt.hpp>
#include <util/time_search.hpp>


namespace csm
{
namespace perception
{

/* Manages IMU sensor samples, providing convenience functions for looking
 * up the delta rotation between timestamps, applying gyro/acceleration biases,
 * and computing the gravity vector. */
template<typename Float_T = double>
class ImuIntegrator
{
public:
    using FloatT = Float_T;
    using TimeFloatT = double;
    using Vec3 = Eigen::Vector<FloatT, 3>;
    using Quat = Eigen::Quaternion<FloatT>;

    using ImuMsg = sensor_msgs::msg::Imu;

public:
    inline ImuIntegrator(
        bool use_orientation = true,
        TimeFloatT calib_time = 1.) :
        use_orientation{use_orientation},
        calib_time{calib_time}
    {
    }
    ~ImuIntegrator() = default;

public:
    void addSample(const ImuMsg& imu);
    void trimSamples(TimeFloatT trim_ts);
    void setAutoTrimWindow(TimeFloatT window_s = 0.f);
    bool recalibrate(TimeFloatT dt, bool force = false);

    Vec3 estimateGravity(
        TimeFloatT dt,
        FloatT* stddev = nullptr,
        FloatT* dr = nullptr) const;
    Quat getDelta(TimeFloatT start, TimeFloatT end) const;
    bool getNormalizedOffsets(
        util::tsq::TSQ<Quat>& dest,
        TimeFloatT t1,
        TimeFloatT t2) const;

    inline bool hasSamples() const { return !this->raw_buffer.empty(); }
    inline bool isCalibrated() const { return this->is_calibrated; }
    inline bool usingOrientation() const { return this->use_orientation; }
    inline const Vec3& gyroBias() const { return this->calib_bias.ang_vel; }
    inline const Vec3& accelBias() const { return this->calib_bias.lin_accel; }

protected:
    void recalibrateRange(size_t begin, size_t end);

    struct ImuMeas
    {
        Vec3 ang_vel{Vec3::Zero()};
        Vec3 lin_accel{Vec3::Zero()};
    };

protected:
    util::tsq::TSQ<Quat> orient_buffer;
    util::tsq::TSQ<ImuMeas> raw_buffer;
    ImuMeas calib_bias;

    mutable std::mutex mtx;
    std::atomic<bool> is_calibrated{false};
    std::atomic<bool> use_orientation;

    const double calib_time;
    double trim_window{600.0};
};





// --- Implementation ----------------------------------------------------------

template<typename F>
void ImuIntegrator<F>::addSample(const ImuMsg& imu)
{
    using namespace util::geom::cvt::ops;

    std::unique_lock imu_lock{this->mtx};

    if (this->use_orientation && imu.orientation_covariance[0] == -1.)
    {
        // message topic source doesn't support orientation
        this->use_orientation = false;
    }

    const TimeFloatT stamp = util::toFloatSeconds(imu.header.stamp);

    if (this->use_orientation)
    {
        Quat q;
        q << imu.orientation;

        const size_t idx =
            util::tsq::binarySearchIdx(this->orient_buffer, stamp);
        this->orient_buffer.emplace(
            this->orient_buffer.begin() + idx,
            stamp,
            q);

        if (this->trim_window > 0)
        {
            util::tsq::trimToStamp(
                this->orient_buffer,
                (util::tsq::newestStamp(this->orient_buffer) -
                 this->trim_window));
        }
    }

    {
        const size_t idx = util::tsq::binarySearchIdx(this->raw_buffer, stamp);
        auto meas = this->raw_buffer.emplace(this->raw_buffer.begin() + idx);

        meas->first = stamp;
        (meas->second.ang_vel << imu.angular_velocity) -=
            this->calib_bias.ang_vel;
        (meas->second.lin_accel << imu.linear_acceleration) -=
            this->calib_bias.lin_accel;

        if (this->calib_time > 0. && !this->is_calibrated &&
            this->calib_time <= (util::tsq::newestStamp(this->raw_buffer) -
                                 util::tsq::oldestStamp(this->raw_buffer)))
        {
            this->recalibrateRange(0, this->raw_buffer.size());
        }

        if (this->trim_window > 0)
        {
            util::tsq::trimToStamp(
                this->raw_buffer,
                (util::tsq::newestStamp(this->raw_buffer) - this->trim_window));
        }
    }
}

template<typename F>
void ImuIntegrator<F>::trimSamples(TimeFloatT trim_ts)
{
    std::unique_lock imu_lock{this->mtx};

    util::tsq::trimToStamp(this->orient_buffer, trim_ts);
    util::tsq::trimToStamp(this->raw_buffer, trim_ts);
}

template<typename F>
void ImuIntegrator<F>::setAutoTrimWindow(TimeFloatT window_s)
{
    std::unique_lock imu_lock{this->mtx};

    this->trim_window = window_s;
}

template<typename F>
bool ImuIntegrator<F>::recalibrate(TimeFloatT dt, bool force)
{
    std::unique_lock imu_lock{this->mtx};

    const TimeFloatT newest_t = util::tsq::newestStamp(this->raw_buffer);
    if (force || dt <= (newest_t - util::tsq::oldestStamp(this->raw_buffer)))
    {
        this->recalibrateRange(
            0,
            util::tsq::binarySearchIdx(this->raw_buffer, newest_t - dt));
        return true;
    }
    return false;
}

template<typename F>
typename ImuIntegrator<F>::Vec3 ImuIntegrator<F>::estimateGravity(
    TimeFloatT dt,
    FloatT* stddev,
    FloatT* dr) const
{
    std::unique_lock imu_lock{this->mtx};

    const TimeFloatT newest_t = util::tsq::newestStamp(this->raw_buffer);
    const size_t max_idx =
        util::tsq::binarySearchIdx(this->raw_buffer, newest_t - dt);
    Vec3 avg = Vec3::Zero();

    for (size_t i = 0; i < max_idx; i++)
    {
        avg += this->raw_buffer[i].second.lin_accel;
    }
    avg /= max_idx;

    if (stddev)
    {
        if (max_idx > 1)
        {
            Vec3 var3 = Vec3::Zero();
            for (size_t i = 0; i < max_idx; i++)
            {
                Vec3 d = this->raw_buffer[i].second.lin_accel - avg;
                var3 += d.cwiseProduct(d);
            }
            *stddev = std::sqrt(var3.sum());
        }
        else
        {
            *stddev = 0.;
        }
    }
    if (dr)
    {
        *dr = 0.;
        for (size_t i = 1; i < max_idx; i++)
        {
            *dr += this->orient_buffer[i - 1].second.angularDistance(
                this->orient_buffer[i].second);
        }
    }

    return (avg += this->accelBias());
}

template<typename F>
typename ImuIntegrator<F>::Quat ImuIntegrator<F>::getDelta(
    TimeFloatT start,
    TimeFloatT end) const
{
    using namespace util::geom::cvt::ops;

    Quat q = Quat::Identity();
    std::unique_lock imu_lock{this->mtx};

    if (this->use_orientation)
    {
        size_t oldest = util::tsq::binarySearchIdx(this->orient_buffer, start);
        size_t newest = util::tsq::binarySearchIdx(this->orient_buffer, end);

        if (oldest == this->orient_buffer.size() && oldest > 0)
        {
            oldest--;
        }
        if (newest > 0)
        {
            newest--;
        }

        if (newest < oldest)
        {
            const auto& a = this->orient_buffer[oldest];
            const auto& b = this->orient_buffer[oldest - 1];
            const auto& c = this->orient_buffer[newest + 1];
            const auto& d = this->orient_buffer[newest];

            const Quat prev = a.second.slerp(
                (start - a.first) / (b.first - a.first),
                b.second);
            const Quat curr =
                c.second.slerp((end - c.first) / (d.first - c.first), d.second);

            q = prev.inverse() * curr;
        }
    }
    else
    {
        size_t oldest = util::tsq::binarySearchIdx(this->raw_buffer, start);
        size_t newest = util::tsq::binarySearchIdx(this->raw_buffer, end);

        if (oldest == this->raw_buffer.size() && oldest > 0)
        {
            oldest--;
        }
        if (newest > 0)
        {
            newest--;
        }

        if (newest < oldest)
        {
            const auto& a = this->raw_buffer[oldest];
            const auto& b = this->raw_buffer[oldest - 1];
            const auto& c = this->raw_buffer[newest + 1];
            const auto& d = this->raw_buffer[newest];

            const Vec3 head =
                a.second.ang_vel + (b.second.ang_vel - a.second.ang_vel) *
                                       (start - a.first) / (b.first - a.first);
            const Vec3 tail =
                c.second.ang_vel + (d.second.ang_vel - c.second.ang_vel) *
                                       (end - c.first) / (d.first - c.first);

            for (size_t i = oldest; i > newest; i--)
            {
                const Vec3 *from, *to;
                double from_t, to_t;
                if (i == oldest)
                {
                    from = &head;
                    from_t = start;
                }
                else
                {
                    from = &(this->raw_buffer[i].second.ang_vel);
                    from_t = this->raw_buffer[i].first;
                }

                if (i - 1 == newest)
                {
                    to = &tail;
                    to_t = end;
                }
                else
                {
                    to = &(this->raw_buffer[i - 1].second.ang_vel);
                    to_t = this->raw_buffer[i - 1].first;
                }

                const Vec3 avg_w = (*from + *to) * 0.5f;
                const FloatT dt = static_cast<FloatT>(to_t - from_t);
                const Vec3 theta = avg_w * dt;
                const FloatT angle = theta.norm();

                if (angle > 1e-8)
                {
                    const Vec3 axis = theta / angle;
                    q = (q * Quat{Eigen::AngleAxis<FloatT>(angle, axis)})
                            .normalized();
                }
            }
        }
    }

    return q;
}

template<typename F>
bool ImuIntegrator<F>::getNormalizedOffsets(
    util::tsq::TSQ<Quat>& dest,
    TimeFloatT t1,
    TimeFloatT t2) const
{
    std::unique_lock imu_lock{this->mtx};

    size_t oldest = util::tsq::binarySearchIdx(this->orient_buffer, t1);
    size_t newest = util::tsq::binarySearchIdx(this->orient_buffer, t2);
    if (oldest == this->orient_buffer.size() && oldest > 0)
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
            this->orient_buffer.begin() + (newest > t2 ? newest + 1 : newest),
            this->orient_buffer.begin() + (oldest < t1 ? oldest : oldest + 1));

        for (auto& e : dest)
        {
            e.first = (e.first - t1) / (t2 - t1);
        }
    }

    const auto& a = this->orient_buffer[oldest];
    const auto& b = this->orient_buffer[oldest - 1];
    const auto& c = this->orient_buffer[newest + 1];
    const auto& d = this->orient_buffer[newest];

    dest.emplace_back();
    dest.emplace_front();
    auto& start = dest.back();
    auto& end = dest.front();

    start.first = 0.;
    end.first = 1.;
    start.second =
        a.second.slerp((t1 - a.first) / (b.first - a.first), b.second)
            .normalized();
    end.second = c.second.slerp((t2 - c.first) / (d.first - c.first), d.second)
                     .normalized();

    Quat inv_ref = dest.back().second.conjugate();
    dest.back().second = Quat::Identity();

    for (size_t i = 0; i < dest.size() - 1; i++)
    {
        (dest[i].second *= inv_ref).normalize();
    }

    return true;
}

template<typename F>
void ImuIntegrator<F>::recalibrateRange(size_t begin, size_t end)
{
    for (size_t i = begin; i < end; i++)
    {
        this->calib_bias.ang_vel += this->raw_buffer[i].second.ang_vel;
        this->calib_bias.lin_accel += this->raw_buffer[i].second.lin_accel;
    }

    this->calib_bias.ang_vel /= this->raw_buffer.size();
    this->calib_bias.lin_accel /= this->raw_buffer.size();

    // retroactively apply the newly calculated bias
    for (auto& m : this->raw_buffer)
    {
        m.second.ang_vel -= this->calib_bias.ang_vel;
        m.second.lin_accel -= this->calib_bias.lin_accel;
    }

    this->is_calibrated = true;
}

};  // namespace perception
};  // namespace csm
