#include "engine/tag_detection.hpp"


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<TagDetector> node = std::make_shared<TagDetector>();

    // rclcpp::executors::MultiThreadedExecutor exec;
    // exec.add_node(node);
    // exec.spin();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
