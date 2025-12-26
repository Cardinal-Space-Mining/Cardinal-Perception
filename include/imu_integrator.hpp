/*******************************************************************************
*   Copyright (C) 2024-2025 Cardinal Space Mining Club                         *
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

#include "util/geometry.hpp"
#include "util/time_cvt.hpp"
#include "util/time_search.hpp"


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

        // 10 min max, orientation is likely still valid but at this point we probably have worse problems
        util::tsq::trimToStamp(
            this->orient_buffer,
            (util::tsq::newestStamp(this->orient_buffer) - 600.));
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

        // 5 min max, integration is definitely deviated after this for most imus
        util::tsq::trimToStamp(
            this->raw_buffer,
            (util::tsq::newestStamp(this->raw_buffer) - 300.));
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

        if (newest != oldest)
        {
            const auto& a = this->orient_buffer[oldest];
            const auto& b = this->orient_buffer[oldest - 1];
            const auto& c = this->orient_buffer[newest + 1];
            const auto& d = this->orient_buffer[newest];

            Quat prev = a.second.slerp(
                (start - a.first) / (b.first - a.first),
                b.second);
            Quat curr =
                c.second.slerp((end - c.first) / (d.first - c.first), d.second);

            q = prev.inverse() * curr;
        }
    }
    else
    {
        assert(!"IMU angular velocity integration is not implemented!");
#if 0  // TODO
        const size_t
            init_idx = util::tsq::binarySearchIdx(this->imu_buffer, start),
            end_idx = util::tsq::binarySearchIdx(this->imu_buffer, end);

        // Relative IMU integration of gyro and accelerometer
        double curr_imu_stamp = 0.;
        double prev_imu_stamp = 0.;
        double dt;

        for(size_t i = init_idx; i >= end_idx; i--)
        {
            const auto& imu_sample = this->raw_buffer[i];

            if(prev_imu_stamp == 0.)
            {
                prev_imu_stamp = imu_sample.first;
                continue;
            }

            // Calculate difference in imu measurement times IN SECONDS
            curr_imu_stamp = imu_sample.first;
            dt = curr_imu_stamp - prev_imu_stamp;
            prev_imu_stamp = curr_imu_stamp;

            // Relative gyro propagation quaternion dynamics
            Quatd qq = q;
            q.w() -= 0.5 * dt *
                (qq.x() * imu_sample.second.ang_vel.x()
                    + qq.y() * imu_sample.second.ang_vel.y()
                    + qq.z() * imu_sample.second.ang_vel.z() );
            q.x() += 0.5 * dt *
                (qq.w() * imu_sample.second.ang_vel.x()
                    - qq.z() * imu_sample.second.ang_vel.y()
                    + qq.y() * imu_sample.second.ang_vel.z() );
            q.y() += 0.5 * dt *
                (qq.z() * imu_sample.second.ang_vel.x()
                    + qq.w() * imu_sample.second.ang_vel.y()
                    - qq.x() * imu_sample.second.ang_vel.z() );
            q.z() += 0.5 * dt *
                (qq.x() * imu_sample.second.ang_vel.y()
                    - qq.y() * imu_sample.second.ang_vel.x()
                    + qq.w() * imu_sample.second.ang_vel.z() );
        }

        q.normalize();
#endif
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
