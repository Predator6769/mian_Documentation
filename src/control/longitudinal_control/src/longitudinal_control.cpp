#include "longitudinal_control/longitudinal_control.hpp"
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <fstream>
#include <optional>
#include <ament_index_cpp/get_package_share_directory.hpp>

LongitudinalController::LongitudinalController() : Node("longitudinal_velocity_control"){

    // traj_subs_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>("/planning/scenario_planning/trajectory",10,
    //     std::bind(&LongitudinalController::trajectory_callback, this, std::placeholders::_1));

    look_ahead_distance = this->declare_parameter("look_ahead_distance", 5.0);
    timer_duration_msec = this->declare_parameter("timer_duration_msec", 100.0);

    vehicle_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/localization/kinematic_state",10,
        std::bind(&LongitudinalController::vehicle_state_callback, this, std::placeholders::_1));

    target_vel_sub_ = this->create_subscription<std_msgs::msg::Float64>("/acc/target_vel",10,
        std::bind(&LongitudinalController::set_target_velocity, this, std::placeholders::_1));
    
    updated_traj_publisher_ = this->create_publisher<autoware_auto_planning_msgs::msg::Trajectory>(
        "/planning/scenario_planning/trajectory", 10);

    std::string pkg_path = ament_index_cpp::get_package_share_directory("longitudinal_control");

    std::string pkg_file = this->declare_parameter("csv_file_path", "/data/parking_lot_trajectory_acc.csv");

    csv_file_path_ = pkg_path + pkg_file;

    if (!read_csv_to_trajectory(csv_file_path_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to read trajectory from CSV.");
    }

    updated_traj_publisher_->publish(current_trajectory);

    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int64_t>(timer_duration_msec)),
        [this]() { this->publishUpdatedTrajectory(); });

}

bool LongitudinalController::read_csv_to_trajectory(const std::string & file_path)
  {
    std::ifstream file(file_path);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Unable to open CSV file: %s", file_path.c_str());
      return false;
    }

    std::string line;
    std::getline(file, line);  // Skip header line

    while (std::getline(file, line)) {
      std::stringstream ss(line);
      std::string cell;
      std::vector<double> values;

      while (std::getline(ss, cell, ',')) {
        values.push_back(std::stod(cell));
      }

      if (values.size() < 11) continue;

      autoware_auto_planning_msgs::msg::TrajectoryPoint tp;
      tp.pose.position.x = values[0];
      tp.pose.position.y = values[1];
      tp.pose.position.z = values[2];
      tp.pose.orientation.x = values[3];
      tp.pose.orientation.y = values[4];
      tp.pose.orientation.z = values[5];
      tp.pose.orientation.w = values[6];
      tp.longitudinal_velocity_mps = values[7];
      tp.acceleration_mps2 = values[10];
      tp.lateral_velocity_mps = values[8];
      tp.heading_rate_rps = values[9];
      tp.front_wheel_angle_rad = 0.0;
      tp.rear_wheel_angle_rad = 0.0;
      tp.time_from_start = builtin_interfaces::msg::Duration();

      current_trajectory.points.push_back(tp);
    }

    current_trajectory.header.frame_id = "map";
    return true;
  }


void LongitudinalController::trajectory_callback(const autoware_auto_planning_msgs::msg::Trajectory & traj){
    current_trajectory = traj;
}

void LongitudinalController::vehicle_state_callback(const nav_msgs::msg::Odometry & vehicle_s){
    current_vehicle_state = vehicle_s;
}

void LongitudinalController::set_target_velocity(const std_msgs::msg::Float64::SharedPtr tar_vel){
    target_velocity = tar_vel->data;
}

std::pair<autoware_auto_planning_msgs::msg::TrajectoryPoint, size_t> LongitudinalController::findNearestTrajectoryPoint(const autoware_auto_planning_msgs::msg::Trajectory & traj, const nav_msgs::msg::Odometry vehicle_state){

    size_t nearest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < traj.points.size(); ++i) {
        const double dx = traj.points[i].pose.position.x - vehicle_state.pose.pose.position.x;
        const double dy = traj.points[i].pose.position.y - vehicle_state.pose.pose.position.y;
        const double dist = std::sqrt(dx * dx + dy * dy);

        if (dist < min_dist) {
            min_dist = dist;
            nearest_idx = i;
        }
    }

    if(nearest_idx == traj.points.size() - 1)
    nearest_idx-=1;

    return {traj.points[nearest_idx+1], nearest_idx+1};
}

std::pair<size_t, size_t> LongitudinalController::getIndecesWithinLookAheadDistance(const autoware_auto_planning_msgs::msg::Trajectory & traj, const double lookahead, size_t start_index){

    size_t end_idx = 0;

    double overall_distance = 0;

    for(size_t i = start_index; i < traj.points.size(); ++i){
        if(i == traj.points.size()-1){
            end_idx = traj.points.size()-1;
            break;
        }
        const double dx = traj.points[i].pose.position.x - traj.points[i+1].pose.position.x;
        const double dy = traj.points[i].pose.position.y - traj.points[i+1].pose.position.y;
        const double dist = std::sqrt(dx * dx + dy * dy);
        overall_distance+=dist;
        if(overall_distance > lookahead){
            end_idx = i-1;
            break;
        }
    }

    return {start_index, end_idx};

}


autoware_auto_planning_msgs::msg::Trajectory LongitudinalController::InterpolateVelocities(
    const autoware_auto_planning_msgs::msg::Trajectory & traj_b,
    size_t start_index,
    size_t end_index,
    const double current_vehicle_velocity,
    const double lookahead)
{
    // Make a copy of the input trajectory
    autoware_auto_planning_msgs::msg::Trajectory traj = traj_b;

    for (size_t i = start_index; i < end_index; ++i) {
        const double dx = traj.points[i].pose.position.x - current_vehicle_state.value().pose.pose.position.x;
        const double dy = traj.points[i].pose.position.y - current_vehicle_state.value().pose.pose.position.y;
        const double dist = std::sqrt(dx * dx + dy * dy);

        // Compute ratio along lookahead distance
        const double ratio = std::clamp(dist / lookahead, 0.0, 1.0);

        const double tar_vel = current_vehicle_velocity + ratio * (target_velocity.value() - current_vehicle_velocity);

        const double accel_target = std::clamp(((tar_vel*tar_vel) - (current_vehicle_velocity*current_vehicle_velocity))/(2*dist), -5.0, 3.0);

        // Linear interpolation from current velocity to target velocity
        if(i != traj.points.size()-1){
            traj.points[i].longitudinal_velocity_mps =
                current_vehicle_velocity + ratio * (target_velocity.value() - current_vehicle_velocity);
            traj.points[i].acceleration_mps2 = accel_target;
        }
    }

    return traj;
}

void LongitudinalController::publishUpdatedTrajectory()
{
    // Safety checks
    if (current_trajectory.points.empty()) {
        // updated_traj_publisher_->publish(current_trajectory);
        RCLCPP_WARN(get_logger(), "Current trajectory is empty. Cannot publish updated trajectory.");
        return;
    }

    if(!current_vehicle_state.has_value() || !target_velocity.has_value()){
        RCLCPP_WARN(get_logger(), "no vehicle state or target velocity Publishing existing trajectory");
        updated_traj_publisher_->publish(current_trajectory);
        return;
    }

    // 1. Find nearest trajectory point to vehicle
    auto [nearest_point, nearest_idx] = findNearestTrajectoryPoint(current_trajectory, current_vehicle_state.value());

    // 2. Determine indices within lookahead distance
    auto [start_idx, end_idx] = getIndecesWithinLookAheadDistance(current_trajectory, look_ahead_distance, nearest_idx);

    if (end_idx <= start_idx) {
        // Not enough points within lookahead
        RCLCPP_WARN(get_logger(), "Lookahead distance too small or insufficient points. Skipping update.");
        updated_traj_publisher_->publish(current_trajectory);
        return;
    }

    // 3. Interpolate velocities along those points
    autoware_auto_planning_msgs::msg::Trajectory updated_traj =
        InterpolateVelocities(current_trajectory, start_idx, end_idx, current_vehicle_state.value().twist.twist.linear.x, look_ahead_distance);

    // 4. Publish the updated trajectory
    if (updated_traj_publisher_) {
        updated_traj_publisher_->publish(updated_traj);
        current_trajectory = updated_traj;
        RCLCPP_INFO(get_logger(), "Published updated trajectory with target velocity %.2f", target_velocity.value());
    } else {
        RCLCPP_WARN(get_logger(), "Updated trajectory publisher not initialized.");
    }
}