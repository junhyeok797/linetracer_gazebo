// Algorithm Header
#include "ekf_odom.hpp"

#include <cmath>
#include <vector>

EKFOdom::EKFOdom() : Node("ekf_odom")
{
    RCLCPP_WARN(this->get_logger(), "Initialize node...");

    // QoS init
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // Parameters
    if (!loadParameters()) return;

    // Subscribes
    s_joint_state_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", qos_profile,
        std::bind(&EKFOdom::jointStateCallback, this, std::placeholders::_1));
    s_imu_data_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu", qos_profile,
        std::bind(&EKFOdom::imuCallback, this, std::placeholders::_1));

    // Publishers
    p_odom_ = create_publisher<nav_msgs::msg::Odometry>("ekf_odom", qos_profile);

    // Initialize
    init(this->now());

    // Timer init
    run_timer_ = create_wall_timer(
        std::chrono::milliseconds((int64_t)(1000 / loop_rate_)),
        [this](){ this->run(this->now()); });
}

void EKFOdom::init(const rclcpp::Time& current_time)
{
    has_joint_state_ = false;
    has_imu_data_ = false;

    x_ = Eigen::VectorXd::Zero(5);

    P_ = Eigen::MatrixXd::Zero(5, 5);
    for (size_t i = 0; i < initial_covariance_diag_.size(); ++i)
    {
        const auto index = static_cast<Eigen::Index>(i);
        P_(index, index) = initial_covariance_diag_[i];
    }

    Q_ = Eigen::MatrixXd::Zero(5, 5);
    Q_(0, 0) = q_x_;
    Q_(1, 1) = q_y_;
    Q_(2, 2) = q_theta_;
    Q_(3, 3) = q_v_;
    Q_(4, 4) = q_w_;

    R_ = Eigen::MatrixXd::Zero(2, 2);
    R_(0, 0) = r_theta_;
    R_(1, 1) = r_w_;

    last_timestamp_ = current_time.seconds();
}

void EKFOdom::run(const rclcpp::Time& current_time)
{
    // Get subscribe variables
    sensor_msgs::msg::JointState joint_state;
    {
        std::lock_guard<std::mutex> lock(mutex_joint_state_);
        if (!has_joint_state_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Waiting for joint_state message before commanding");
            return;
        }
        joint_state = joint_state_;
    }
    sensor_msgs::msg::Imu imu_data;
    {
        std::lock_guard<std::mutex> lock(mutex_imu_data_);
        if (!has_imu_data_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Waiting for imu message before commanding");
            return;
        }
        imu_data = imu_data_;
    }

    // Output variables

    // Algorithm
    predict(joint_state);
    update(imu_data);

    // Update output
    odom_.header.stamp = current_time;
    odom_.header.frame_id = frame_id_;
    odom_.child_frame_id = child_frame_id_;

    odom_.pose.pose.position.x = x_(0);
    odom_.pose.pose.position.y = x_(1);
    odom_.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
	q.setRPY(0.0, 0.0, x_(2));
	odom_.pose.pose.orientation.x = q.x();
	odom_.pose.pose.orientation.y = q.y();
	odom_.pose.pose.orientation.z = q.z();
	odom_.pose.pose.orientation.w = q.w();

    odom_.twist.twist.linear.x = x_(3);
    odom_.twist.twist.linear.y = 0.0;
    odom_.twist.twist.linear.z = 0.0;
    odom_.twist.twist.angular.x = 0.0;
    odom_.twist.twist.angular.y = 0.0;
    odom_.twist.twist.angular.z = x_(4);

    odom_.pose.covariance.fill(0.0);
    odom_.pose.covariance[0] = P_(0, 0);
    odom_.pose.covariance[1] = P_(0, 1);
    odom_.pose.covariance[5] = P_(0, 2);
    odom_.pose.covariance[6] = P_(1, 0);
    odom_.pose.covariance[7] = P_(1, 1);
    odom_.pose.covariance[11] = P_(1, 2);
    odom_.pose.covariance[30] = P_(2, 0);
    odom_.pose.covariance[31] = P_(2, 1);
    odom_.pose.covariance[35] = P_(2, 2);

    odom_.twist.covariance.fill(0.0);
    odom_.twist.covariance[0] = P_(3, 3);
    odom_.twist.covariance[35] = P_(4, 4);

    // Publish
    publish(current_time);
}

void EKFOdom::publish(const rclcpp::Time& current_time)
{
    p_odom_->publish(odom_);

    if (publish_tf_ && tf_broadcaster_) {
		geometry_msgs::msg::TransformStamped transform;
		transform.header.stamp = current_time;
		transform.header.frame_id = frame_id_;
		transform.child_frame_id = child_frame_id_;
		transform.transform.translation.x = x_(0);
		transform.transform.translation.y = x_(1);
		transform.transform.translation.z = 0.0;
		transform.transform.rotation = odom_.pose.pose.orientation;

		tf_broadcaster_->sendTransform(transform);
	}
}

bool EKFOdom::loadParameters()
{
    this->declare_parameter<double>("system.loop_rate", 10.0);
    if (!this->get_parameter("system.loop_rate", loop_rate_))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get loop_rate"); return false;
    }
    this->declare_parameter<bool>("system.use_sim_time", false);
    if (!this->get_parameter("system.use_sim_time", use_sim_time_))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get use_sim_time"); return false;
    }

    this->declare_parameter<double>("model.wheel_radius", 0.033);
    if (!this->get_parameter("model.wheel_radius", wheel_radius_))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get wheel_radius"); return false;
    }
    this->declare_parameter<double>("model.wheel_separation", 0.287);
    if (!this->get_parameter("model.wheel_separation", wheel_separation_))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get wheel_separation"); return false;
    }
    this->declare_parameter<double>("model.imu_yaw_offset", 0.0);
    if (!this->get_parameter("model.imu_yaw_offset", imu_yaw_offset_))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get imu_yaw_offset"); return false;
    }

    this->declare_parameter<bool>("tf.publish_tf", false);
    if (!this->get_parameter("tf.publish_tf", publish_tf_))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get publish_tf"); return false;
    }
    this->declare_parameter<std::string>("tf.frame_id", "odom");
    if (!this->get_parameter("tf.frame_id", frame_id_))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get frame_id"); return false;
    }
    this->declare_parameter<std::string>("tf.child_frame_id", "base_footprint");
    if (!this->get_parameter("tf.child_frame_id", child_frame_id_))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get child_frame_id"); return false;
    }

    std::vector<double> init_cov_default{0.5, 0.5, 0.05, 0.1, 0.1};
    this->declare_parameter<std::vector<double>>("ekf.initial_covariance.diagonal", init_cov_default);
    std::vector<double> init_covariance;
    if (!this->get_parameter("ekf.initial_covariance.diagonal", init_covariance))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get EKF initial covariance"); return false;
    }
    if (init_covariance.size() != initial_covariance_diag_.size())
    {
        RCLCPP_ERROR(this->get_logger(), "EKF initial covariance must have %zu entries", initial_covariance_diag_.size());
        return false;
    }
    for (size_t i = 0; i < initial_covariance_diag_.size(); ++i)
    {
        initial_covariance_diag_[i] = init_covariance[i];
    }

    this->declare_parameter<double>("ekf.process_noise.q_x", 0.05);
    if (!this->get_parameter("ekf.process_noise.q_x", q_x_))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get ekf.process_noise.q_x"); return false;
    }
    this->declare_parameter<double>("ekf.process_noise.q_y", 0.05);
    if (!this->get_parameter("ekf.process_noise.q_y", q_y_))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get ekf.process_noise.q_y"); return false;
    }
    this->declare_parameter<double>("ekf.process_noise.q_theta", 0.01);
    if (!this->get_parameter("ekf.process_noise.q_theta", q_theta_))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get ekf.process_noise.q_theta"); return false;
    }
    this->declare_parameter<double>("ekf.process_noise.q_v", 0.1);
    if (!this->get_parameter("ekf.process_noise.q_v", q_v_))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get ekf.process_noise.q_v"); return false;
    }
    this->declare_parameter<double>("ekf.process_noise.q_w", 0.05);
    if (!this->get_parameter("ekf.process_noise.q_w", q_w_))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get ekf.process_noise.q_w"); return false;
    }

    this->declare_parameter<double>("ekf.measurement_noise.r_theta", 0.02);
    if (!this->get_parameter("ekf.measurement_noise.r_theta", r_theta_))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get ekf.measurement_noise.r_theta"); return false;
    }
    this->declare_parameter<double>("ekf.measurement_noise.r_w", 0.02);
    if (!this->get_parameter("ekf.measurement_noise.r_w", r_w_))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get ekf.measurement_noise.r_w"); return false;
    }

    RCLCPP_INFO(this->get_logger(), "=== EKFOdom parameters ===");
    RCLCPP_INFO(this->get_logger(), "[System]");
    RCLCPP_INFO(this->get_logger(), "  loop_rate=%.1f", loop_rate_);
    RCLCPP_INFO(this->get_logger(), "  use_sim_time=%s", use_sim_time_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "[Model]");
    RCLCPP_INFO(this->get_logger(), "  wheel_radius=%.3f", wheel_radius_);
    RCLCPP_INFO(this->get_logger(), "  wheel_separation=%.3f", wheel_separation_);
    RCLCPP_INFO(this->get_logger(), "  imu_yaw_offset=%.3f", imu_yaw_offset_);
    RCLCPP_INFO(this->get_logger(), "[tf]");
    RCLCPP_INFO(this->get_logger(), "  publish_tf=%s", publish_tf_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  frame_id=%s", frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "  child_frame_id=%s", child_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "[ekf]");
    RCLCPP_INFO(this->get_logger(), "  initial_covariance=[%.3f, %.3f, %.3f, %.3f, %.3f]",
        initial_covariance_diag_[0], initial_covariance_diag_[1], initial_covariance_diag_[2],
        initial_covariance_diag_[3], initial_covariance_diag_[4]);
    RCLCPP_INFO(this->get_logger(), "  process_noise=[%.3f, %.3f, %.3f, %.3f, %.3f]", q_x_, q_y_, q_theta_, q_v_, q_w_);
    RCLCPP_INFO(this->get_logger(), "  measurement_noise=[%.3f, %.3f]", r_theta_, r_w_);

    return true;
}

void EKFOdom::predict(sensor_msgs::msg::JointState& joint_state)
{
    // 1. 시간 차이(dt) 계산
    double current_timestamp = rclcpp::Time(joint_state.header.stamp).seconds();
    double dt = current_timestamp - last_timestamp_;
    if (dt <= 0.0) return;

    // 2. 인코더로부터 선속도(v)와 각속도(w) 계산 (제어 입력 추출)
    double wheel_l_vel = joint_state.velocity[0];
    double wheel_r_vel = joint_state.velocity[1];

    double v = wheel_radius_ * (wheel_r_vel + wheel_l_vel) / 2.0;
    double w = wheel_radius_ * (wheel_r_vel - wheel_l_vel) / wheel_separation_;

    // 3. 상태 예측
    double theta = x_(2);
    x_(0) += v * std::cos(theta + (w * dt / 2.0)) * dt;  // x
    x_(1) += v * std::sin(theta + (w * dt / 2.0)) * dt;  // y
    x_(2) += w * dt;                                // theta
    x_(3) = v;                                      // v
    x_(4) = w;                                      // w

    // 4. 야코비안 행렬(F) 계산
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(5, 5);
    F(0, 2) = -v * std::sin(theta + (w * dt / 2.0)) * dt;
    F(0, 3) = std::cos(theta + (w * dt / 2.0)) * dt;
    F(1, 2) = v * std::cos(theta + (w * dt / 2.0)) * dt;
    F(1, 3) = std::sin(theta + (w * dt / 2.0)) * dt;
    F(2, 4) = dt;

    // 5. 오차 공분산 예측 (Covariance Prediction: P = F*P*F' + Q)
    P_ = F * P_ * F.transpose() + Q_;

    last_timestamp_ = current_timestamp;
}

void EKFOdom::update(sensor_msgs::msg::Imu& imu_data)
{
    // 1. IMU로부터 측정값(z) 추출
    double a = 2.0 * (imu_data.orientation.w * imu_data.orientation.z + imu_data.orientation.x * imu_data.orientation.y);
    double b = 1.0 - 2.0 * (imu_data.orientation.y * imu_data.orientation.y + imu_data.orientation.z * imu_data.orientation.z);
    double imu_theta = std::atan2(a, b);
    double imu_w = imu_data.angular_velocity.z;

    Eigen::VectorXd z(2);
    z << imu_theta, imu_w;

    // 2. 관측 행렬 H 정의 (상태 벡터에서 theta와 w만 추출)
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 5);
    H(0, 2) = 1.0; // theta 관측
    H(1, 4) = 1.0; // w 관측

    // 3. 혁신(Innovation) 계산: y = z - Hx
    Eigen::VectorXd y = z - H * x_;
    normalize_angle(y(0));

    // 4. 칼만 이득(Kalman Gain) 계산: K = P*H' * inv(H*P*H' + R)
    Eigen::MatrixXd S = H * P_ * H.transpose() + R_;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    // 5. 상태 벡터 업데이트: x = x + Ky
    x_ = x_ + K * y;

    // 6. 오차 공분산 업데이트: P = (I - KH)P
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(5, 5);
    P_ = (I - K * H) * P_;
}

void EKFOdom::normalize_angle(double& angle)
{
	constexpr double pi = 3.14159265358979323846;
	constexpr double two_pi = 2.0 * pi;

	while (angle > pi) {
		angle -= two_pi;
	}
	while (angle < -pi) {
		angle += two_pi;
	}
}