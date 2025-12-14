#ifndef __EKF_ODOM_HPP__
#define __EKF_ODOM_HPP__

// STD Header
#include <array>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// ROS Header
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// ROS Message Header
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// Algoorithm Header
#include <Eigen/Dense>


class EKFOdom : public rclcpp::Node
{
public:
    EKFOdom();

    void init(const rclcpp::Time& current_time);
    void run(const rclcpp::Time& current_time);
    void publish(const rclcpp::Time& current_time);

    private:
    // Callback functions
    void jointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_joint_state_);
        joint_state_ = *msg;
        has_joint_state_ = true;
    }
    void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_imu_data_);
        imu_data_ = *msg;
        has_imu_data_ = true;
    }

    // Algorithm functions
    bool loadParameters();

    void predict(sensor_msgs::msg::JointState& joint_state);
    void update(sensor_msgs::msg::Imu& imu_data);

    void normalize_angle(double& angle);

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr s_joint_state_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr s_imu_data_;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr p_odom_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Timer
    rclcpp::TimerBase::SharedPtr run_timer_;

    // Inputs
    sensor_msgs::msg::JointState joint_state_;
    sensor_msgs::msg::Imu imu_data_;

    // Mutex
    std::mutex mutex_joint_state_;
    std::mutex mutex_imu_data_;

    // Outputs
    nav_msgs::msg::Odometry odom_;

    // Parameters
    double loop_rate_;
    bool use_sim_time_;
    
    double wheel_radius_;
    double wheel_separation_;
    double imu_yaw_offset_;
    
    bool publish_tf_;
    std::string frame_id_;
    std::string child_frame_id_;

    std::array<double, 5> initial_covariance_diag_{};
    double q_x_{};
    double q_y_{};
    double q_theta_{};
    double q_v_{};
    double q_w_{};
    double r_theta_{};
    double r_w_{};

    // Variable
    bool has_joint_state_;
    bool has_imu_data_;

    double last_timestamp_;

    Eigen::VectorXd x_; // 상태 벡터: [x, y, theta, v, w]
    Eigen::MatrixXd P_; // 상태 공분산 행렬 (5x5)
    Eigen::MatrixXd Q_; // 프로세스 노이즈 행렬 (5x5)

    Eigen::MatrixXd R_; // 관측 노이즈 행렬 (IMU 센서의 신뢰도)
    Eigen::MatrixXd H_; // 관측 행렬 (상태 변수와 측정값의 관계)
};

#endif  // __EKF_ODOM_HPP__