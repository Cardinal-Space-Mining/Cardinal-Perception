#include "./engine/localization.hpp"


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<PerceptionNode> node = std::make_shared<PerceptionNode>();

    // rclcpp::executors::MultiThreadedExecutor exec;
    // exec.add_node(node);
    // exec.spin();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
