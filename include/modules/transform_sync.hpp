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
#include <string>
#include <string_view>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <util/geometry.hpp>
#include <util/time_cvt.hpp>

#include "trajectory_filter.hpp"

#define TRANSFORM_SYNC_PRINT_DEBUG 1
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
template<typename Pose3_T = util::geom::Pose3d>
class TransformSynchronizer
{
    static_assert(
        std::is_same_v<Pose3_T, util::geom::Pose3f> ||
        std::is_same_v<Pose3_T, util::geom::Pose3d>);

    template<typename F>
    using Pose3_ = util::geom::Pose3<F>;
    template<typename F>
    using PoseTf3_ = util::geom::PoseTf3<F>;

    using Pose3 = Pose3_T;
    using PoseTf3 = PoseTf3_<typename Pose3::Scalar_T>;

    using TrajectoryFilterT = csm::perception::TrajectoryFilter<Pose3>;

public:
    using SyncToken = uint32_t;

public:
    TransformSynchronizer(
        tf2_ros::TransformBroadcaster& tf_broadcaster,
        tf2_ros::Buffer* tf_buffer = nullptr,
        std::string_view map_frame_id = "map",
        std::string_view odom_frame_id = "odom",
        std::string_view base_frame_id = "base_link");
    ~TransformSynchronizer() = default;

public:
    void setFrameIds(
        std::string_view map_frame_id,
        std::string_view odom_frame_id,
        std::string_view base_frame_id);

    SyncToken beginOdometryIteration();
    template<typename F>
    void endOdometryIterationSuccess(const Pose3_<F>& tf, double ts);
    template<typename F>
    void endOdometryIterationSuccess(const PoseTf3_<F>& tf, double ts);
    void endOdometryIterationFailure();

    void beginMeasurementIteration(SyncToken);
    template<typename F>
    void endMeasurementIterationSuccess(const Pose3_<F>& tf, double ts);
    void endMeasurementIterationFailure();

    TrajectoryFilterT& getFilter();
    const TrajectoryFilterT& getFilter() const;

    PoseTf3 getOdomTf() const;
    PoseTf3 getMapTf() const;

    template<typename F>
    double getOdomTf(PoseTf3_<F>& tf) const;
    template<typename F>
    double getMapTf(PoseTf3_<F>& tf) const;

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

    PoseTf3 odom_tf;
    PoseTf3 map_tf;
    double odom_stamp{0.};
    double map_stamp{0.};

private:
    bool resolveOdomSucceeded() const;
    bool resolveOdomFailed() const;
    bool resolveOdomFinished() const;

    bool resolveMeasSucceeded() const;
    bool resolveMeasFailed() const;
    bool resolveMeasFinished() const;

    bool resolveHasDesynced() const;

    bool resolveOdomHasPriority() const;
    bool resolveMeasHasPriority() const;

    void updateMap();
    void publishMap();
    void publishOdom();
};



// --- Implementation ----------------------------------------------------------

#if TRANSFORM_SYNC_PRINT_DEBUG
    #include <iostream>
#endif


template<typename P>
TransformSynchronizer<P>::TransformSynchronizer(
    tf2_ros::TransformBroadcaster& tf_broadcaster,
    tf2_ros::Buffer* tf_buffer,
    std::string_view map_frame_id,
    std::string_view odom_frame_id,
    std::string_view base_frame_id) :
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


template<typename P>
void TransformSynchronizer<P>::setFrameIds(
    std::string_view map_frame_id,
    std::string_view odom_frame_id,
    std::string_view base_frame_id)
{
    this->map_frame = map_frame_id;
    this->odom_frame = odom_frame_id;
    this->base_frame = base_frame_id;
}


template<typename P>
typename TransformSynchronizer<P>::SyncToken
    TransformSynchronizer<P>::beginOdometryIteration()
{
    SyncToken tk;
    {
        std::unique_lock lock{this->mtx};

        this->odom_end_counter = this->odom_beg_counter;
        tk = ++this->odom_beg_counter;
    }

#if TRANSFORM_SYNC_PRINT_DEBUG
    std::cout << "[TFSYNC]: BEGIN ODOMETRY (" << tk << ")" << std::endl;
#endif

    return tk;
}

template<typename P>
template<typename F>
void TransformSynchronizer<P>::endOdometryIterationSuccess(
    const Pose3_<F>& tf,
    double ts)
{
    using namespace util::geom::cvt::ops;

    PoseTf3 tmp;
    tmp.pose << tf;
    tmp.tf << tf;
    this->endOdometryIterationSuccess(tmp, ts);
}

template<typename P>
template<typename F>
void TransformSynchronizer<P>::endOdometryIterationSuccess(
    const PoseTf3_<F>& tf,
    double ts)
{
    using namespace util::geom::cvt::ops;

    std::unique_lock lock{this->mtx};

    this->odom_end_counter = this->odom_beg_counter;
    {
        std::unique_lock tf_lock{this->tf_mtx};

        this->odom_tf << tf;
        this->odom_stamp = ts;
    }
    this->trajectory_filter.addOdom(this->odom_tf.pose, this->odom_stamp);

    if (this->resolveOdomHasPriority())
    {
        if (this->trajectory_filter.getStatus().last_filter_status)
        {
            this->updateMap();
            this->publishMap();
        }
        this->publishOdom();
    }

#if TRANSFORM_SYNC_PRINT_DEBUG
    std::cout << "[TFSYNC]: END ODOMETRY SUCCESS (" << this->odom_beg_counter
              << ") <PRIO: " << this->resolveOdomHasPriority() << ", FLTR: "
              << this->trajectory_filter.getStatus().last_filter_status << ">"
              << std::endl;
#endif
}

template<typename P>
void TransformSynchronizer<P>::endOdometryIterationFailure()
{
    std::unique_lock lock{this->mtx};

    this->odom_end_counter = 0;

    if (this->resolveOdomHasPriority() &&
        this->trajectory_filter.getStatus().last_filter_status)
    {
        this->updateMap();
        this->publishMap();
    }

#if TRANSFORM_SYNC_PRINT_DEBUG
    std::cout << "[TFSYNC]: END ODOMETRY FAILURE (" << this->odom_beg_counter
              << ") <PRIO: " << this->resolveOdomHasPriority() << ", FLTR: "
              << this->trajectory_filter.getStatus().last_filter_status << ">"
              << std::endl;
#endif
}


template<typename P>
void TransformSynchronizer<P>::beginMeasurementIteration(SyncToken x)
{
    {
        std::unique_lock lock{this->mtx};

        this->meas_beg_counter = x;
        this->meas_end_counter = x - 1;
    }

#if TRANSFORM_SYNC_PRINT_DEBUG
    std::cout << "[TFSYNC]: BEGIN MEASUREMENT (" << x << ")" << std::endl;
#endif
}

template<typename P>
template<typename F>
void TransformSynchronizer<P>::endMeasurementIterationSuccess(
    const Pose3_<F>& tf,
    double ts)
{
    std::unique_lock lock{this->mtx};

    this->meas_end_counter = this->meas_beg_counter;
    this->trajectory_filter.addAbsolute(tf, ts);

    const bool filter_success =
        this->trajectory_filter.getStatus().last_filter_status;
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
    std::cout << "[TFSYNC]: END MEASUREMENT SUCCESS (" << this->meas_beg_counter
              << ") <PRIO: " << this->resolveMeasHasPriority()
              << ", FLTR: " << filter_success
              << ", ODMS: " << this->resolveOdomSucceeded() << ">" << std::endl;
#endif
}

template<typename P>
void TransformSynchronizer<P>::endMeasurementIterationFailure()
{
    std::unique_lock lock{this->mtx};

    this->meas_end_counter = 0;

    if (this->resolveMeasHasPriority() && this->resolveOdomSucceeded())
    {
        this->publishOdom();
    }

#if TRANSFORM_SYNC_PRINT_DEBUG
    std::cout << "[TFSYNC]: END MEASUREMENT FAILURE (" << this->meas_beg_counter
              << ") <PRIO: " << this->resolveMeasHasPriority()
              << ", ODMS: " << this->resolveOdomSucceeded() << ">" << std::endl;
#endif
}


template<typename P>
TransformSynchronizer<P>::TrajectoryFilterT&
    TransformSynchronizer<P>::getFilter()
{
    return this->trajectory_filter;
}

template<typename P>
const TransformSynchronizer<P>::TrajectoryFilterT&
    TransformSynchronizer<P>::getFilter() const
{
    return this->trajectory_filter;
}


template<typename P>
typename TransformSynchronizer<P>::PoseTf3 TransformSynchronizer<P>::getOdomTf()
    const
{
    std::unique_lock lock{this->tf_mtx};

    return this->odom_tf;
}

template<typename P>
typename TransformSynchronizer<P>::PoseTf3 TransformSynchronizer<P>::getMapTf()
    const
{
    std::unique_lock lock{this->tf_mtx};

    return this->map_tf;
}

template<typename P>
template<typename F>
double TransformSynchronizer<P>::getOdomTf(util::geom::PoseTf3<F>& tf) const
{
    using namespace util::geom::cvt::ops;

    std::unique_lock lock{this->tf_mtx};

    tf << this->odom_tf;
    return this->odom_stamp;
}

template<typename P>
template<typename F>
double TransformSynchronizer<P>::getMapTf(util::geom::PoseTf3<F>& tf) const
{
    using namespace util::geom::cvt::ops;

    std::unique_lock lock{this->tf_mtx};

    tf << this->map_tf;
    return this->map_stamp;
}


template<typename P>
bool TransformSynchronizer<P>::resolveOdomSucceeded() const
{
    return (this->odom_beg_counter == this->odom_end_counter);
}
template<typename P>
bool TransformSynchronizer<P>::resolveOdomFailed() const
{
    return (this->odom_end_counter == 0);
}
template<typename P>
bool TransformSynchronizer<P>::resolveOdomFinished() const
{
    return (this->resolveOdomSucceeded() || this->resolveOdomFailed());
}

template<typename P>
bool TransformSynchronizer<P>::resolveMeasSucceeded() const
{
    return (this->meas_beg_counter == this->meas_end_counter);
}
template<typename P>
bool TransformSynchronizer<P>::resolveMeasFailed() const
{
    return (this->meas_end_counter == 0);
}
template<typename P>
bool TransformSynchronizer<P>::resolveMeasFinished() const
{
    return (this->resolveMeasSucceeded() || this->resolveMeasFailed());
}

template<typename P>
bool TransformSynchronizer<P>::resolveHasDesynced() const
{
    return (this->meas_beg_counter < this->odom_beg_counter);
}

template<typename P>
bool TransformSynchronizer<P>::resolveOdomHasPriority() const
{
    return (this->resolveMeasFinished() || this->resolveHasDesynced());
}
template<typename P>
bool TransformSynchronizer<P>::resolveMeasHasPriority() const
{
    return (this->resolveOdomFinished() && !this->resolveHasDesynced());
}


template<typename P>
void TransformSynchronizer<P>::updateMap()
{
    using namespace util::geom::cvt::ops;
    using namespace util::tsq;

    typename TrajectoryFilterT::TStamped_<typename TrajectoryFilterT::MeasPair>
        filtered_meas = this->trajectory_filter.getFiltered();

    typename PoseTf3::Tf_T absolute_tf, match_tf;
    absolute_tf << value(filtered_meas).absolute;
    match_tf << value(filtered_meas).odometry;

    {
        std::unique_lock lock{this->tf_mtx};

        this->map_tf.tf = (absolute_tf * match_tf.inverse());
        this->map_tf.pose << this->map_tf.tf;
        this->map_stamp = tstamp(filtered_meas);
    }
}

template<typename P>
void TransformSynchronizer<P>::publishMap()
{
    using namespace util::geom::cvt::ops;

    geometry_msgs::msg::TransformStamped tf_msg;

    tf_msg.header.stamp = util::toTimeMsg(this->map_stamp);
    tf_msg.header.frame_id = this->map_frame;
    tf_msg.child_frame_id = this->odom_frame;
    tf_msg.transform << this->map_tf.pose;

    this->tf_broadcaster.sendTransform(tf_msg);
    if (this->tf_buffer)
    {
        this->tf_buffer->setTransform(tf_msg, "cardinal_perception");
    }

#if TRANSFORM_SYNC_PRINT_DEBUG
    std::cout << "[TFSYNC]: PUBLISH MAP TF @ " << this->map_stamp << std::endl;
#endif
}

template<typename P>
void TransformSynchronizer<P>::publishOdom()
{
    using namespace util::geom::cvt::ops;

    geometry_msgs::msg::TransformStamped tf_msg;

    tf_msg.header.stamp = util::toTimeMsg(this->odom_stamp);
    tf_msg.header.frame_id = this->odom_frame;
    tf_msg.child_frame_id = this->base_frame;
    tf_msg.transform << this->odom_tf.pose;

    this->tf_broadcaster.sendTransform(tf_msg);
    if (this->tf_buffer)
    {
        this->tf_buffer->setTransform(tf_msg, "cardinal_perception");
    }

#if TRANSFORM_SYNC_PRINT_DEBUG
    std::cout << "[TFSYNC]: PUBLISH ODOM TF @ " << this->odom_stamp
              << std::endl;
#endif
}

};  // namespace perception
};  // namespace csm
