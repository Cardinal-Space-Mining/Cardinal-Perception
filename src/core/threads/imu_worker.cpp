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

#include "imu_worker.hpp"

#include <Eigen/Core>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <csm_metrics/profiling.hpp>

#include <util.hpp>
#include <geometry.hpp>
#include <imu_transform.hpp>


using namespace util::geom::cvt::ops;

using Vec3d = Eigen::Vector3d;
using Quatd = Eigen::Quaterniond;
using PoseStampedMsg = geometry_msgs::msg::PoseStamped;


namespace csm
{
namespace perception
{

ImuWorker::ImuWorker(RclNode& node, const Tf2Buffer& tf_buffer) :
    node{node},
    tf_buffer{tf_buffer},
    pub_map{node, PERCEPTION_TOPIC(""), PERCEPTION_PUBSUB_QOS}
{
}

void ImuWorker::configure(const std::string& base_frame)
{
    this->base_frame = base_frame;
}

void ImuWorker::accept(const ImuMsg& msg)
{
    PROFILING_SYNC();
    PROFILING_NOTIFY_ALWAYS(imu);

    try
    {
        auto tf = this->tf_buffer.lookupTransform(
            this->base_frame,
            msg.header.frame_id,
            util::toTf2TimePoint(msg.header.stamp));

        ImuMsg tf_imu;
        tf2::doTransform(msg, tf_imu, tf);

        this->imu_sampler.addSample(tf_imu);
    }
    catch (const std::exception& e)
    {
        RCLCPP_INFO(
            this->node.get_logger(),
            "[IMU CALLBACK]: Failed to process imu measurment.\n\twhat(): %s",
            e.what());
    }

#if PERCEPTION_PUBLISH_GRAV_ESTIMATION > 0
    double stddev, delta_r;
    Vec3d grav_vec = this->imu_sampler.estimateGravity(1., &stddev, &delta_r);

    PoseStampedMsg grav_pub;
    grav_pub.header.stamp = msg.header.stamp;
    grav_pub.header.frame_id = this->base_frame;
    grav_pub.pose.orientation
        << Quatd::FromTwoVectors(Vec3d{1., 0., 0.}, grav_vec);
    this->pub_map.publish("poses/gravity_estimation", grav_pub);
    this->pub_map.publish<std_msgs::msg::Float64>(
        "metrics/gravity_estimation/acc_stddev",
        stddev);
    this->pub_map.publish<std_msgs::msg::Float64>(
        "metrics/gravity_estimation/delta_rotation",
        delta_r);
#endif

    PROFILING_NOTIFY_ALWAYS(imu);
}

};  // namespace perception
};  // namespace csm
