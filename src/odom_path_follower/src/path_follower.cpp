// Algorithm Header
#include "path_follower.hpp"

PathFollower::PathFollower() : Node("path_follower")
{
    RCLCPP_WARN(this->get_logger(), "Initialize node...");

    // Qos init
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // Parameters
    if (!loadParameters())
    {
        RCLCPP_FATAL(this->get_logger(), "Failed to load parameters");
        throw std::runtime_error("path_follower parameter loading failed");
    }

    // Subscribers
    s_odom_data_ = create_subscription<nav_msgs::msg::Odometry>(
        "ekf_odom", qos_profile,
        std::bind(&PathFollower::odomCallback, this, std::placeholders::_1));

    // Publishers
    p_vehicle_command_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos_profile);

    auto path_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    p_path_viz_ = create_publisher<nav_msgs::msg::Path>("target_path", path_qos);

    // Initialize
    init(this->now());

    // Timer init
    run_timer_ = create_wall_timer(
        std::chrono::milliseconds((int64_t)(1000 / loop_rate_)),
        [this](){ this->run(this->now()); });
}

void PathFollower::init(const rclcpp::Time& current_time)
{
    (void)current_time;
    has_odom_data_ = false;
    path_loaded_ = false;    current_path_index_ = 0;
    goal_reached_ = false;
    current_path_index_ = 0U;

    path_loaded_ = loadPath(path_relative_file_);
    if (!path_loaded_)
    {
        RCLCPP_FATAL(this->get_logger(), "Failed to load path: %s", path_relative_file_.c_str());
        throw std::runtime_error("path_follower path loading failed");
    }
}

void PathFollower::run(const rclcpp::Time& current_time)
{
    // Get subscribe variables
    if (!path_loaded_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Path not loaded yet");
        return;
    }

    nav_msgs::msg::Odometry odom;
    {
        std::lock_guard<std::mutex> lock(mutex_odom_data_);
        if (!has_odom_data_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for odometry");
            return;
        }
        odom = odom_data_;
    }

    // Algorithm
    if (goal_reached_)
    {
        vehicle_command_.linear.x = 0.0;
        vehicle_command_.angular.z = 0.0;
        publish(this->now());
        return;
    }

    double linear_x = clampWithStep(vehicle_command_.linear.x,
                                    target_linear_velocity_,
                                    lin_vel_step_size_,
                                    max_lin_vel_);
    double angular_z = vehicle_command_.angular.z;

    if (!computeControl(odom, linear_x, angular_z))
    {
        goal_reached_ = true;
        vehicle_command_.linear.x = 0.0;
        vehicle_command_.angular.z = 0.0;
        publish(this->now());
        return;
    }
    RCLCPP_INFO(this->get_logger(), "linear_x=%.2f, angular_z=%.2f", linear_x, angular_z);

    // Update output
    vehicle_command_.linear.x = linear_x;
    vehicle_command_.angular.z = angular_z;

    // Publish
    publish(current_time);
}

void PathFollower::publish(const rclcpp::Time& current_time)
{
    (void)current_time;
    p_vehicle_command_->publish(vehicle_command_);
}

bool PathFollower::loadParameters()
{
    this->declare_parameter<double>("system.loop_rate", 10.0);
    if (!this->get_parameter("system.loop_rate", loop_rate_))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get loop_rate"); return false;
    }

    this->declare_parameter<double>("model.max_lin_vel", 0.26);
    if (!this->get_parameter("model.max_lin_vel", max_lin_vel_))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get model.max_lin_vel"); return false;
    }
    this->declare_parameter<double>("model.max_ang_vel", 1.82);
    if (!this->get_parameter("model.max_ang_vel", max_ang_vel_))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get model.max_ang_vel"); return false;
    }
    this->declare_parameter<double>("model.lin_vel_step_size", 0.01);
    if(!this->get_parameter("model.lin_vel_step_size", lin_vel_step_size_))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get model.lin_vel_step_size"); return false;
    }
    this->declare_parameter<double>("model.ang_vel_step_size", 0.01);
    if(!this->get_parameter("model.ang_vel_step_size", ang_vel_step_size_))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get model.ang_vel_step_size"); return false;
    }

    this->declare_parameter<std::string>("control.path_relative_file", "path/rbp_path.csv");
    if(!this->get_parameter("control.path_relative_file", path_relative_file_))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get control.path_relative_file"); return false;
    }
    this->declare_parameter<double>("control.target_lin_vel", 0.2);
    if(!this->get_parameter("control.target_lin_vel", target_linear_velocity_))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get control.target_lin_vel"); return false;
    }
    this->declare_parameter<double>("control.base_lookahead_distance", 0.3);
    if(!this->get_parameter("control.base_lookahead_distance", base_lookahead_distance_))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get control.base_lookahead_distance"); return false;
    }
    this->declare_parameter<double>("control.lookahead_gain", 0.2);
    if(!this->get_parameter("control.lookahead_gain", lookahead_gain_))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get control.lookahead_gain"); return false;
    }
    this->declare_parameter<double>("control.min_lookahead_distance", 0.2);
    if(!this->get_parameter("control.min_lookahead_distance", min_lookahead_distance_))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get control.min_lookahead_distance"); return false;
    }
    this->declare_parameter<double>("control.max_lookahead_distance", 1.0);
    if(!this->get_parameter("control.max_lookahead_distance", max_lookahead_distance_))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get control.max_lookahead_distance"); return false;
    }
    this->declare_parameter<double>("control.goal_tolerance", 0.05);
    if(!this->get_parameter("control.goal_tolerance", goal_tolerance_))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get control.goal_tolerance"); return false;
    }

    RCLCPP_INFO(this->get_logger(), "=== PathFollower parameters ===");
    RCLCPP_INFO(this->get_logger(), "[system]");
    RCLCPP_INFO(this->get_logger(), "  loop_rate=%.1f", loop_rate_);
    RCLCPP_INFO(this->get_logger(), "[model]");
    RCLCPP_INFO(this->get_logger(), "  max_lin_vel=%.2f", max_lin_vel_);
    RCLCPP_INFO(this->get_logger(), "  max_ang_vel=%.2f", max_ang_vel_);
    RCLCPP_INFO(this->get_logger(), "  lin_vel_step_size=%.2f", lin_vel_step_size_);
    RCLCPP_INFO(this->get_logger(), "  ang_vel_step_size=%.2f", ang_vel_step_size_);
    RCLCPP_INFO(this->get_logger(), "[control]");
    RCLCPP_INFO(this->get_logger(), "  path_relative_file=%s", path_relative_file_.c_str());
    RCLCPP_INFO(this->get_logger(), "  target_linear_velocity=%.2f", target_linear_velocity_);
    RCLCPP_INFO(this->get_logger(), "  base_lookahead_distance=%.2f", base_lookahead_distance_);
    RCLCPP_INFO(this->get_logger(), "  lookahead_gain=%.2f", lookahead_gain_);
    RCLCPP_INFO(this->get_logger(), "  min_lookahead_distance=%.2f", min_lookahead_distance_);
    RCLCPP_INFO(this->get_logger(), "  max_lookahead_distance=%.2f", max_lookahead_distance_);
    RCLCPP_INFO(this->get_logger(), "  goal_tolerance=%.2f", goal_tolerance_);

    return true;
}

bool PathFollower::loadPath(const std::string& relative_path)
{
    std::string package_share;
    try
    {
        package_share = ament_index_cpp::get_package_share_directory("odom_path_follower");
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to locate package share: %s", e.what());
        return false;
    }

    const std::string full_path = package_share + "/" + relative_path;

    std::ifstream infile(full_path);
    if (!infile.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot open path file: %s", full_path.c_str());
        return false;
    }

    std::string line;
    if (!std::getline(infile, line))
    {
        RCLCPP_ERROR(this->get_logger(), "Path file is empty: %s", full_path.c_str());
        return false;
    }

    std::vector<PathPoint> path;
    while (std::getline(infile, line))
    {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string token;
        std::vector<double> values;
        while (std::getline(ss, token, ','))
        {
            try
            {
                values.emplace_back(std::stod(token));
            }
            catch (const std::exception&)
            {
                values.clear();
                break;
            }
        }

        if (values.size() < 2) continue;

        PathPoint point{};
        point.x = values[0];
        point.y = values[1];
        point.yaw = values.size() >= 3 ? values[2] : 0.0;
        path.emplace_back(point);
    }

    if (path.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "No valid points found in %s", full_path.c_str());
        return false;
    }

    path_points_ = std::move(path);
    current_path_index_ = 0U;

    // --- RViz 시각화 메시지 생성 부분 추가 ---
    path_viz_msg_.header.frame_id = "odom"; // 실제 사용하는 프레임 이름 확인 (map 또는 odom)
    path_viz_msg_.header.stamp = this->now();
    path_viz_msg_.poses.clear();

    for (const auto& pt : path_points_)
    {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = path_viz_msg_.header;
        pose_stamped.pose.position.x = pt.x;
        pose_stamped.pose.position.y = pt.y;
        pose_stamped.pose.position.z = 0.0;
        
        // Yaw 값이 있다면 쿼터니언으로 변환하여 할당
        tf2::Quaternion q;
        q.setRPY(0, 0, pt.yaw);
        pose_stamped.pose.orientation = tf2::toMsg(q);

        path_viz_msg_.poses.push_back(pose_stamped);
    }

    // 경로 토픽 발행
    p_path_viz_->publish(path_viz_msg_);
    // ---------------------------------------

    RCLCPP_INFO(this->get_logger(), "Loaded %zu path points and published to RViz", path_points_.size());
    return true;
}

bool PathFollower::computeControl(const nav_msgs::msg::Odometry& odom,
                                  double& linear_x,
                                  double& angular_z)
{
    if (path_points_.empty()) return false;

    const auto& pose = odom.pose.pose;
    const double robot_x = pose.position.x;
    const double robot_y = pose.position.y;
    const double robot_yaw = tf2::getYaw(pose.orientation);

    const PathPoint& goal = path_points_.back();
    const double goal_dist = std::hypot(goal.x - robot_x, goal.y - robot_y);
    if (goal_dist <= goal_tolerance_)
    {
        linear_x = 0.0;
        angular_z = 0.0;
        return false;
    }

    const double lookahead_distance = computeLookaheadDistance(std::fabs(linear_x));
    updateLookaheadIndex(pose, lookahead_distance);

    const PathPoint& target_point = path_points_[current_path_index_];

    const double dx = target_point.x - robot_x;
    const double dy = target_point.y - robot_y;

    const double cos_yaw = std::cos(robot_yaw);
    const double sin_yaw = std::sin(robot_yaw);

    const double x_local = cos_yaw * dx + sin_yaw * dy;
    const double y_local = -sin_yaw * dx + cos_yaw * dy;

    const double distance_sq = (x_local * x_local) + (y_local * y_local);
    if (distance_sq < 1e-6)
    {
        angular_z = 0.0;
        return true;
    }

    const double curvature = (2.0 * y_local) / distance_sq;
    const double target_ang_vel = std::max(-max_ang_vel_, std::min(curvature * linear_x, max_ang_vel_));
    angular_z = clampWithStep(angular_z, target_ang_vel, ang_vel_step_size_, max_ang_vel_);

    linear_x = clampWithStep(linear_x, target_linear_velocity_, lin_vel_step_size_, max_lin_vel_);
    if (goal_dist < lookahead_distance)
    {
        linear_x = std::min(linear_x, goal_dist);
    }

    return true;
}

void PathFollower::updateLookaheadIndex(const geometry_msgs::msg::Pose& pose,
                                        double lookahead_distance)
{
    if (path_points_.empty()) return;

    const double x = pose.position.x;
    const double y = pose.position.y;

    std::size_t nearest_index = current_path_index_;
    double nearest_distance = std::numeric_limits<double>::max();

    for (std::size_t i = current_path_index_; i < path_points_.size(); ++i)
    {
        const double dist = std::hypot(path_points_[i].x - x, path_points_[i].y - y);
        if (dist < nearest_distance)
        {
            nearest_distance = dist;
            nearest_index = i;
        }
        else if (dist > nearest_distance && i > nearest_index)
        {
            break;
        }
    }

    std::size_t lookahead_index = nearest_index;
    for (std::size_t i = nearest_index; i < path_points_.size(); ++i)
    {
        const double dist = std::hypot(path_points_[i].x - x, path_points_[i].y - y);
        if (dist >= lookahead_distance)
        {
            lookahead_index = i;
            break;
        }
    }

    current_path_index_ = lookahead_index;
}

double PathFollower::computeLookaheadDistance(double current_speed) const
{
    const double candidate = base_lookahead_distance_ + (lookahead_gain_ * current_speed);
    return std::max(min_lookahead_distance_, std::min(candidate, max_lookahead_distance_));
}

double PathFollower::clampWithStep(double current_value,
                                   double target_value,
                                   double step_size,
                                   double limit) const
{
    const double bounded_target = std::max(-limit, std::min(target_value, limit));
    const double diff = bounded_target - current_value;
    if (std::fabs(diff) <= step_size)
    {
        return bounded_target;
    }
    return current_value + std::copysign(step_size, diff);
}