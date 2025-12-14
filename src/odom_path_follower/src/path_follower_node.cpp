// ROS Header
#include "rclcpp/rclcpp.hpp"

// Algorithm Header
#include "path_follower.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PathFollower>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}