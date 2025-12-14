// ROS Header
#include "rclcpp/rclcpp.hpp"

// Algorithm Header
#include "ekf_odom.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<EKFOdom>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}