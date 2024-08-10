#include "./engine/perception.hpp"


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(std::make_shared<PerceptionNode>());
    exec.spin();

    rclcpp::shutdown();

    return 0;
}
