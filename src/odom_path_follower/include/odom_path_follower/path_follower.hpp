#ifndef __PATH_FOLLOWER_HPP__
#define __PATH_FOLLOWER_HPP__

// STD Header
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>
#include <functional>
#include <limits>
#include <sstream>
#include <stdexcept>

// ROS Header
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// ROS Message Header
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"

// Algorithm
#include "ament_index_cpp/get_package_share_directory.hpp"


struct PathPoint
{
    double x;
    double y;
    double yaw;
};

class PathFollower : public rclcpp::Node
{
public:
    PathFollower();

    void init(const rclcpp::Time& curret_time);
    void run(const rclcpp::Time& curret_time);
    void publish(const rclcpp::Time& current_time);

private:
    // Callback functions
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
        std::lock_guard<std::mutex> lock(mutex_odom_data_);
        odom_data_ = *msg;
        has_odom_data_ = true;
    }

    // Algorithm functions
    bool loadParameters();
    bool loadPath(const std::string& relative_path);
    
    bool computeControl(const nav_msgs::msg::Odometry& odom,
                        double& linear_x,
                        double& angular_z);
    void updateLookaheadIndex(const geometry_msgs::msg::Pose& pose,
                              double lookahead_distance);
    double computeLookaheadDistance(double current_speed) const;
    double clampWithStep(double current_value,
                         double target_value,
                         double step_size,
                         double limit) const;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr s_odom_data_;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr p_path_viz_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr p_vehicle_command_;

    // Timer
    rclcpp::TimerBase::SharedPtr run_timer_;

    // Inputs
    nav_msgs::msg::Odometry odom_data_;

    // Mutex
    std::mutex mutex_odom_data_;

    // Outputs
    nav_msgs::msg::Path path_viz_msg_;
    geometry_msgs::msg::Twist vehicle_command_;

    // Parameters
    double loop_rate_;

    double max_lin_vel_;
    double max_ang_vel_;
    double lin_vel_step_size_;
    double ang_vel_step_size_;

    std::string path_relative_file_;
    double target_linear_velocity_;
    double base_lookahead_distance_;
    double lookahead_gain_;
    double min_lookahead_distance_;
    double max_lookahead_distance_;
    double goal_tolerance_;

    // Variables
    bool has_odom_data_;
    std::vector<PathPoint> path_points_;
    bool path_loaded_;
    std::size_t current_path_index_;
    bool goal_reached_;
};

#endif  // __PATH_FOLLOWER_HPP__