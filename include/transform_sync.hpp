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
#include <string>
#include <string_view>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "util.hpp"
#include "geometry.hpp"
#include "trajectory_filter.hpp"

#ifndef TRANSFORM_SYNC_PRINT_DEBUG
    #define TRANSFORM_SYNC_PRINT_DEBUG 0
#endif


namespace csm
{
namespace perception
{

/** Utility container for synchronizing the publishing of transform updates
 *  given an [assumed singular] odometry thread and [assumed singular] global
 *  measurement thread. It is also assumed that the odometry thread "seeds"
 *  all measurement instances. */
template<typename MeasPose_T = util::geom::Pose3d, typename Float_T = double>
class TransformSynchronizer
{
    using TrajectoryFilterT =
        csm::perception::TrajectoryFilter<MeasPose_T, Float_T>;
    using MeasT = MeasPose_T;
    using MeasPtrT = typename TrajectoryFilterT::MeasPtr;
    using Pose3 = typename TrajectoryFilterT::Pose3;
    using PoseTf3 = util::geom::PoseTf3<Float_T>;

public:
    using SyncToken = uint32_t;

public:
    inline TransformSynchronizer(
        tf2_ros::TransformBroadcaster& tf_broadcaster,
        tf2_ros::Buffer* tf_buffer = nullptr,
        std::string_view map_frame_id = "map",
        std::string_view odom_frame_id = "odom",
        std::string_view base_frame_id = "base_link") :
        tf_broadcaster{tf_broadcaster},
        tf_buffer{tf_buffer},
        map_frame{map_frame_id},
        odom_frame{odom_frame_id},
        base_frame{base_frame_id}
    {
        this->odom_tf.pose.quat.setIdentity();
        this->odom_tf.pose.vec.setZero();
        this->odom_tf.tf.setIdentity();
        this->map_tf.pose.quat.setIdentity();
        this->map_tf.pose.vec.setZero();
        this->map_tf.tf.setIdentity();
    }
    ~TransformSynchronizer() = default;

public:
    void setFrameIds(
        std::string_view map_frame_id,
        std::string_view odom_frame_id,
        std::string_view base_frame_id);

    SyncToken beginOdometryIteration();
    void beginMeasurementIteration(SyncToken);

    template<typename Flt>
    void endOdometryIterationSuccess(
        const util::geom::PoseTf3<Flt>& tf,
        double ts);
    void endOdometryIterationFailure();

    void endMeasurementIterationSuccess(const MeasT& meas, double ts);
    void endMeasurementIterationSuccess(const MeasPtrT& meas, double ts);
    void endMeasurementIterationFailure();

    TrajectoryFilterT& trajectoryFilter() { return this->trajectory_filter; }
    const TrajectoryFilterT& trajectoryFilter() const
    {
        return this->trajectory_filter;
    }

    PoseTf3 getOdomTf() const;
    PoseTf3 getMapTf() const;

    template<typename Flt>
    double getOdomTf(util::geom::PoseTf3<Flt>& tf) const;
    template<typename Flt>
    double getMapTf(util::geom::PoseTf3<Flt>& tf) const;

protected:
    tf2_ros::TransformBroadcaster& tf_broadcaster;
    tf2_ros::Buffer* tf_buffer{nullptr};
    TrajectoryFilterT trajectory_filter;

    std::string map_frame;
    std::string odom_frame;
    std::string base_frame;

    SyncToken odom_beg_counter{1};
    SyncToken meas_beg_counter{1};
    SyncToken odom_end_counter{1};
    SyncToken meas_end_counter{1};

    std::mutex mtx;
    mutable std::mutex tf_mtx;

    PoseTf3 map_tf, odom_tf;
    double map_stamp{0.}, odom_stamp{0.};

private:
    inline bool resolveOdomSucceeded() const
    {
        return (this->odom_beg_counter == this->odom_end_counter);
    }
    inline bool resolveOdomFailed() const
    {
        return (this->odom_end_counter == 0);
    }
    inline bool resolveOdomFinished() const
    {
        return (this->resolveOdomSucceeded() || this->resolveOdomFailed());
    }

    inline bool resolveMeasSucceeded() const
    {
        return (this->meas_beg_counter == this->meas_end_counter);
    }
    inline bool resolveMeasFailed() const
    {
        return (this->meas_end_counter == 0);
    }
    inline bool resolveMeasFinished() const
    {
        return (this->resolveMeasSucceeded() || this->resolveMeasFailed());
    }

    inline bool resolveHasDesynced() const
    {
        return (this->meas_beg_counter < this->odom_beg_counter);
    }

    inline bool resolveOdomHasPriority() const
    {
        return (this->resolveMeasFinished() || this->resolveHasDesynced());
    }
    inline bool resolveMeasHasPriority() const
    {
        return (this->resolveOdomFinished() && !this->resolveHasDesynced());
    }

    void updateMap();
    void publishMap();
    void publishOdom();
};





#if TRANSFORM_SYNC_PRINT_DEBUG
    #include <iostream>
#endif

template<typename MP, typename F>
void TransformSynchronizer<MP, F>::setFrameIds(
    std::string_view map_frame_id,
    std::string_view odom_frame_id,
    std::string_view base_frame_id)
{
    this->map_frame = map_frame_id;
    this->odom_frame = odom_frame_id;
    this->base_frame = base_frame_id;
}

template<typename MP, typename F>
typename TransformSynchronizer<MP, F>::SyncToken
    TransformSynchronizer<MP, F>::beginOdometryIteration()
{
    SyncToken tk;
    this->mtx.lock();
    {
        this->odom_end_counter = this->odom_beg_counter;
        tk = ++this->odom_beg_counter;
    }
    this->mtx.unlock();

#if TRANSFORM_SYNC_PRINT_DEBUG
    std::cout << "[TFSYNC]: BEGIN ODOMETRY -- " << tk << std::endl;
#endif

    return tk;
}

template<typename MP, typename F>
void TransformSynchronizer<MP, F>::beginMeasurementIteration(SyncToken x)
{
    this->mtx.lock();
    {
        this->meas_beg_counter = x;
        this->meas_end_counter = x - 1;
    }
    this->mtx.unlock();

#if TRANSFORM_SYNC_PRINT_DEBUG
    std::cout << "[TFSYNC]: BEGIN MEASUREMENT -- " << x << std::endl;
#endif
}

template<typename MP, typename F>
template<typename Flt>
void TransformSynchronizer<MP, F>::endOdometryIterationSuccess(
    const util::geom::PoseTf3<Flt>& tf,
    double ts)
{
    using namespace util::geom::cvt::ops;

    this->mtx.lock();
    {
        this->odom_end_counter = this->odom_beg_counter;
        this->tf_mtx.lock();
        {
            this->odom_tf << tf;
            this->odom_stamp = ts;
        }
        this->tf_mtx.unlock();
        this->trajectory_filter.addOdom(this->odom_tf.pose, this->odom_stamp);

        if (this->resolveOdomHasPriority())
        {
            if (this->trajectory_filter.lastFilterStatus())
            {
                this->updateMap();
                this->publishMap();
            }
            this->publishOdom();
        }

#if TRANSFORM_SYNC_PRINT_DEBUG
        std::cout << "[TFSYNC]: END ODOMETRY SUCCESS -- "
                  << this->resolveOdomHasPriority() << " -- "
                  << this->trajectory_filter.lastFilterStatus() << std::endl;
#endif
    }

    this->mtx.unlock();
}

template<typename MP, typename F>
void TransformSynchronizer<MP, F>::endOdometryIterationFailure()
{
    this->mtx.lock();
    {
        this->odom_end_counter = 0;

        if (this->resolveOdomHasPriority() &&
            this->trajectory_filter.lastFilterStatus())
        {
            this->updateMap();
            this->publishMap();
        }

#if TRANSFORM_SYNC_PRINT_DEBUG
        std::cout << "[TFSYNC]: END ODOMETRY FAILURE -- "
                  << this->resolveOdomHasPriority() << " -- "
                  << this->trajectory_filter.lastFilterStatus() << std::endl;
#endif
    }

    this->mtx.unlock();
}

template<typename MP, typename F>
void TransformSynchronizer<MP, F>::endMeasurementIterationSuccess(
    const MeasT& meas,
    double ts)
{
    return this->endMeasurementIterationSuccess(
        std::make_shared<MeasT>(meas),
        ts);
}

template<typename MP, typename F>
void TransformSynchronizer<MP, F>::endMeasurementIterationSuccess(
    const MeasPtrT& meas,
    double ts)
{
    this->mtx.lock();
    {
        this->meas_end_counter = this->meas_beg_counter;
        this->trajectory_filter.addMeasurement(meas, ts);

        const bool filter_success = this->trajectory_filter.lastFilterStatus();
        if (filter_success)
        {
            this->updateMap();
        }

        if (this->resolveMeasHasPriority())
        {
            if (filter_success)
            {
                this->publishMap();
            }
            if (this->resolveOdomSucceeded())
            {
                this->publishOdom();
            }
        }

#if TRANSFORM_SYNC_PRINT_DEBUG
        std::cout << "[TFSYNC]: END MEASUREMENT SUCCESS -- "
                  << this->resolveMeasHasPriority() << " -- " << filter_success
                  << " -- " << this->resolveOdomSucceeded() << std::endl;
#endif
    }

    this->mtx.unlock();
}

template<typename MP, typename F>
void TransformSynchronizer<MP, F>::endMeasurementIterationFailure()
{
    this->mtx.lock();
    {
        this->meas_end_counter = 0;

        if (this->resolveMeasHasPriority() && this->resolveOdomSucceeded())
        {
            this->publishOdom();
        }

#if TRANSFORM_SYNC_PRINT_DEBUG
        std::cout << "[TFSYNC]: END MEASUREMENT FAILURE -- "
                  << this->resolveMeasHasPriority() << " -- "
                  << this->resolveOdomSucceeded() << std::endl;
#endif
    }

    this->mtx.unlock();
}


template<typename MP, typename F>
typename TransformSynchronizer<MP, F>::PoseTf3
    TransformSynchronizer<MP, F>::getOdomTf() const
{
    std::unique_lock lock{this->tf_mtx};
    return this->odom_tf;
}

template<typename MP, typename F>
typename TransformSynchronizer<MP, F>::PoseTf3
    TransformSynchronizer<MP, F>::getMapTf() const
{
    std::unique_lock lock{this->tf_mtx};
    return this->map_tf;
}

template<typename MP, typename F>
template<typename Flt>
double TransformSynchronizer<MP, F>::getOdomTf(
    util::geom::PoseTf3<Flt>& tf) const
{
    using namespace util::geom::cvt::ops;

    std::unique_lock lock{this->tf_mtx};
    tf << this->odom_tf;
    return this->odom_stamp;
}

template<typename MP, typename F>
template<typename Flt>
double TransformSynchronizer<MP, F>::getMapTf(
    util::geom::PoseTf3<Flt>& tf) const
{
    using namespace util::geom::cvt::ops;

    std::unique_lock lock{this->tf_mtx};
    tf << this->map_tf;
    return this->map_stamp;
}


template<typename MP, typename F>
void TransformSynchronizer<MP, F>::updateMap()
{
    using namespace util::geom::cvt::ops;

    auto keypose = this->trajectory_filter.getFiltered();

    const Pose3& absolute =
        static_cast<const Pose3&>(*keypose.second.measurement);
    const Pose3& match = keypose.second.odometry;

    typename PoseTf3::Tf_T absolute_tf, match_tf;
    absolute_tf << absolute;
    match_tf << match;

    this->tf_mtx.lock();
    {
        this->map_tf.tf = (absolute_tf * match_tf.inverse()).template cast<F>();
        this->map_tf.pose << this->map_tf.tf;
        this->map_stamp = keypose.first;
    }
    this->tf_mtx.unlock();
}

template<typename MP, typename F>
void TransformSynchronizer<MP, F>::publishMap()
{
    using namespace util::geom::cvt::ops;

    geometry_msgs::msg::TransformStamped tf_;

    tf_.header.stamp = util::toTimeStamp(this->map_stamp);
    tf_.header.frame_id = this->map_frame;
    tf_.child_frame_id = this->odom_frame;
    tf_.transform << this->map_tf.pose;

    this->tf_broadcaster.sendTransform(tf_);
    if(this->tf_buffer)
    {
        this->tf_buffer->setTransform(tf_, "cardinal_perception");
    }
}

template<typename MP, typename F>
void TransformSynchronizer<MP, F>::publishOdom()
{
    using namespace util::geom::cvt::ops;

    geometry_msgs::msg::TransformStamped tf_;

    tf_.header.stamp = util::toTimeStamp(this->odom_stamp);
    tf_.header.frame_id = this->odom_frame;
    tf_.child_frame_id = this->base_frame;
    tf_.transform << this->odom_tf.pose;

    this->tf_broadcaster.sendTransform(tf_);
    if(this->tf_buffer)
    {
        this->tf_buffer->setTransform(tf_, "cardinal_perception");
    }
}

};  // namespace perception
};  // namespace csm
