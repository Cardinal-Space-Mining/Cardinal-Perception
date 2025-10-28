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

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/point_stamped.hpp>

#include <cardinal_perception/srv/update_path_planning_mode.hpp>

#define FG_CLICKED_POINT_TOPIC "/clicked_point"
#define PATH_SERVER_SERVICE    "/cardinal_perception/update_path_planning"


class FgPathServer : public rclcpp::Node
{
    using PointStampedMsg = geometry_msgs::msg::PointStamped;
    using UpdatePathPlanSrv = cardinal_perception::srv::UpdatePathPlanningMode;

public:
    FgPathServer();

protected:
    void handleClickedPoint(const PointStampedMsg& msg);

protected:
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    rclcpp::Subscription<PointStampedMsg>::SharedPtr target_sub;
    rclcpp::Client<UpdatePathPlanSrv>::SharedPtr path_plan_client;
//
};


FgPathServer::FgPathServer() :
    Node("fg_path_server"),
    tf_buffer{std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)},
    tf_listener{tf_buffer},
    target_sub{this->create_subscription<PointStampedMsg>(
        FG_CLICKED_POINT_TOPIC,
        rclcpp::SensorDataQoS{},
        [this](const PointStampedMsg& msg) { this->handleClickedPoint(msg); })},
    path_plan_client{
        this->create_client<UpdatePathPlanSrv>(PATH_SERVER_SERVICE)}
{
}

void FgPathServer::handleClickedPoint(const PointStampedMsg& msg)
{
    if (!this->path_plan_client->service_is_ready())
    {
        return;
    }

    this->path_plan_client->prune_pending_requests();

    auto req = std::make_shared<UpdatePathPlanSrv::Request>();
    req->target.header = msg.header;
    req->target.pose.position = msg.point;
    req->completed = false;

    this->path_plan_client->async_send_request(
        req,
        [this](rclcpp::Client<UpdatePathPlanSrv>::SharedFuture) {});
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FgPathServer>());
    rclcpp::shutdown();
}
