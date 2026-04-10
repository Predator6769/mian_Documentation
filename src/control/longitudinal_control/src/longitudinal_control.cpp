#include "longitudinal_control/longitudinal_control.hpp"
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <fstream>
#include <optional>
#include <ament_index_cpp/get_package_share_directory.hpp>

LongitudinalController::LongitudinalController() : Node("longitudinal_velocity_control"){

    // traj_subs_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>("/planning/scenario_planning/trajectory",10,
    //     std::bind(&LongitudinalController::trajectory_callback, this, std::placeholders::_1));

    look_ahead_distance = this->declare_parameter("look_ahead_distance", 2.0);
    timer_duration_msec = this->declare_parameter("timer_duration_msec", 100.0);
    control_mode = this->declare_parameter("control_mode", 2);
    use_idm_in_autoware = this->declare_parameter("use_idm_in_autoware", 0);

    vehicle_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/localization/kinematic_state",10,
        std::bind(&LongitudinalController::vehicle_state_callback, this, std::placeholders::_1));

    target_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("/acc/target_vel",10,
        std::bind(&LongitudinalController::set_target_velocity, this, std::placeholders::_1));
    
    if(use_idm_in_autoware == 1){
        traj_subs_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>("/planning/scenario_planning/trajectory",10,
            std::bind(&LongitudinalController::trajectory_callback, this, std::placeholders::_1));
        
        updated_traj_publisher_ = this->create_publisher<autoware_auto_planning_msgs::msg::Trajectory>(
            "/planning/scenario_planning/trajectory_updated", 10);
        
        goal_point_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/planning/mission_planning/goal",10,
        std::bind(&LongitudinalController::new_goal_callback, this, std::placeholders::_1));
    }
    else{
        // std::cout<<"use_idm_in_autoware_disabled";
        RCLCPP_INFO(get_logger(), "autoware_idm_disabled");
        updated_traj_publisher_ = this->create_publisher<autoware_auto_planning_msgs::msg::Trajectory>(
            "/planning/scenario_planning/trajectory", 10);
    }

    // goal_point_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/planning/mission_planning/goal",10,
    //     std::bind(&LongitudinalController::new_goal_callback, this, std::placeholders::_1));



    // std::string home_dir = std::getenv("HOME");
    // csv_file_path_ = home_dir + "/parking_lot_trajectory_acc.csv";

    std::string pkg_path = ament_index_cpp::get_package_share_directory("longitudinal_control");

    std::string pkg_file = this->declare_parameter("csv_file_path", "/data/parking_lot_trajectory_acc.csv");

    acceleration_sub_ =
    this->create_subscription<geometry_msgs::msg::AccelWithCovarianceStamped>(
        "/localization/acceleration",
        10,
        std::bind(&LongitudinalController::acceleration_callback, this, std::placeholders::_1));

    csv_file_path_ = pkg_path + pkg_file;

    std::cout<<csv_file_path_;
    

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
    raw_trajectory = traj;
}

void LongitudinalController::vehicle_state_callback(const nav_msgs::msg::Odometry & vehicle_s){
    current_vehicle_state = vehicle_s;
}

void LongitudinalController::set_target_velocity(const geometry_msgs::msg::TwistStamped::SharedPtr tar_vel){
    target_velocity = tar_vel->twist.linear.x;
    target_acceleration = tar_vel->twist.linear.z;
}

void LongitudinalController::acceleration_callback(const geometry_msgs::msg::AccelWithCovarianceStamped & tar_acc){
    current_acceleration = tar_acc;
}

void LongitudinalController::new_goal_callback(const geometry_msgs::msg::PoseStamped & goal){

    if(use_idm_in_autoware == 1){
        current_trajectory = raw_trajectory;
        updated_traj_publisher_->publish(current_trajectory);
    }
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

    return {traj.points[nearest_idx], nearest_idx};
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

    int j = static_cast<int>(start_index);

    for (size_t i = start_index; i < end_index; ++i,--j) {
        const double dx = traj.points[i].pose.position.x - current_vehicle_state.value().pose.pose.position.x;
        const double dy = traj.points[i].pose.position.y - current_vehicle_state.value().pose.pose.position.y;
        const double dist = std::sqrt(dx * dx + dy * dy);

        // Compute ratio along lookahead distance
        // const double ratio = std::clamp(dist / lookahead, 0.0, 1.0);

        // const double tar_v = current_vehicle_velocity + ratio * (target_velocity.value() - current_vehicle_velocity);

        const double tar_v = target_velocity.value();

        double ds, target_a;

        if(i == 0)
        ds = std::hypot(
            traj.points[i+1].pose.position.x - traj.points[i].pose.position.x,
            traj.points[i+1].pose.position.y - traj.points[i].pose.position.y
        );
        else
        ds = std::hypot(
            traj.points[i].pose.position.x - traj.points[i-1].pose.position.x,
            traj.points[i].pose.position.y - traj.points[i-1].pose.position.y
        );
        
        if(i == start_index)
        target_a = std::clamp(((tar_v*tar_v) - (current_vehicle_velocity*current_vehicle_velocity))/(2*ds), -5.0, 3.0);
        else
        target_a = std::clamp(((tar_v*tar_v) - (traj.points[i-1].longitudinal_velocity_mps*traj.points[i-1].longitudinal_velocity_mps))/(2*ds), -5.0, 3.0);


        // Linear interpolation from current velocity to target velocity
        if(i != traj.points.size()-1){
            traj.points[i].longitudinal_velocity_mps = tar_v;
            
            traj.points[i].acceleration_mps2 = target_a;
        }

        if(j >= 0){
            traj.points[j].longitudinal_velocity_mps = tar_v;
            
            traj.points[j].acceleration_mps2 = target_a;
        }
    }

    return traj;
}

autoware_auto_planning_msgs::msg::Trajectory LongitudinalController::InterpolateAccelerations(
    const autoware_auto_planning_msgs::msg::Trajectory & traj_b,
    size_t start_index,
    size_t end_index,
    const double current_vehicle_acceleration,
    const double current_vehicle_velocity,
    const double lookahead)
{
    // Make a copy of the input trajectory
    autoware_auto_planning_msgs::msg::Trajectory traj = traj_b;

    int j = static_cast<int>(start_index);

    for (size_t i = start_index; i <end_index; ++i,--j) {
        const double dx = traj.points[i].pose.position.x - current_vehicle_state.value().pose.pose.position.x;
        const double dy = traj.points[i].pose.position.y - current_vehicle_state.value().pose.pose.position.y;
        const double dist = std::sqrt(dx * dx + dy * dy);

        // Compute ratio along lookahead distance
        const double ratio = std::clamp(dist / lookahead, 0.0, 1.0);

        // const double target_a = std::clamp(current_vehicle_acceleration + (ratio * (target_acceleration.value() - current_vehicle_acceleration)), -5.0,3.0);
        
        const double target_a = target_acceleration.value();
        
        double tar_v, ds;

        if(i == traj.points.size()-1)
        ds = std::hypot(
            traj.points[i].pose.position.x - traj.points[i-1].pose.position.x,
            traj.points[i].pose.position.y - traj.points[i-1].pose.position.y
        );
        else
        ds = std::hypot(
            traj.points[i+1].pose.position.x - traj.points[i].pose.position.x,
            traj.points[i+1].pose.position.y - traj.points[i].pose.position.y
        );
        
        if(i == start_index)
        tar_v = std::clamp(
            std::sqrt(std::max(0.0, (2 * ds * target_a) + 
                                    (current_vehicle_velocity * current_vehicle_velocity))),
            0.0, 10.0
        );
        else
        tar_v = std::clamp(
            std::sqrt(std::max(0.0, (2 * ds * target_a) + 
                                    (traj.points[i-1].longitudinal_velocity_mps *
                                    traj.points[i-1].longitudinal_velocity_mps))),
            0.0, 10.0
        );


        // Linear interpolation from current velocity to target velocity
        if(i != traj.points.size()-1){
            traj.points[i].longitudinal_velocity_mps = tar_v;
            traj.points[i].acceleration_mps2 = target_a;

            // traj.points[start_index - j].longitudinal_velocity_mps = tar_v;
            
            // traj.points[start_index - j].acceleration_mps2 = target_a;
        }

        // if(j>= 0){
        //     traj.points[j].longitudinal_velocity_mps = tar_v;
            
        //     traj.points[j].acceleration_mps2 = target_a;
        // }
    }

    return traj;
}

autoware_auto_planning_msgs::msg::Trajectory LongitudinalController::InterpolateAccelerationsAndVelocity(
    const autoware_auto_planning_msgs::msg::Trajectory & traj_b,
    size_t start_index,
    size_t end_index,
    const double current_vehicle_acceleration,
    const double current_vehicle_velocity,
    const double lookahead)
{
    // Make a copy of the input trajectory
    autoware_auto_planning_msgs::msg::Trajectory traj = traj_b;

    int j = static_cast<int>(start_index);

    for (size_t i = start_index; i < end_index; ++i,--j) {
        const double dx = traj.points[i].pose.position.x - current_vehicle_state.value().pose.pose.position.x;
        const double dy = traj.points[i].pose.position.y - current_vehicle_state.value().pose.pose.position.y;
        const double dist = std::sqrt(dx * dx + dy * dy);

        // Compute ratio along lookahead distance
        const double ratio = std::clamp(dist / lookahead, 0.0, 1.0);

        const double target_a = std::clamp(target_acceleration.value(), -5.0,3.0);

        double tar_v = std::clamp(target_velocity.value(),0.0,15.0);

        // Linear interpolation from current velocity to target velocity
        if(i != traj.points.size()-1){
            traj.points[i].longitudinal_velocity_mps = tar_v;
            
            traj.points[i].acceleration_mps2 = target_a;
        }

        // if(j >= 0){
        //     traj.points[j].longitudinal_velocity_mps = tar_v;
            
        //     traj.points[j].acceleration_mps2 = target_a;
        // }
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

    if(control_mode == 0){
        if(!current_vehicle_state.has_value() || !target_velocity.has_value()){
            RCLCPP_WARN(get_logger(), "no vehicle state or target velocity Publishing existing trajectory");
            updated_traj_publisher_->publish(current_trajectory);
            return;
        }
    }

    else if(control_mode == 1){
        if(!current_vehicle_state.has_value() || !target_acceleration.has_value()){
            RCLCPP_WARN(get_logger(), "no vehicle state or target velocity Publishing existing trajectory");
            updated_traj_publisher_->publish(current_trajectory);
            return;
        }
    }

    else {
        if(!current_vehicle_state.has_value() || (!target_acceleration.has_value() && !target_velocity.has_value())){
            RCLCPP_WARN(get_logger(), "no vehicle state or target velocity Publishing existing trajectory");
            updated_traj_publisher_->publish(current_trajectory);
            return;
        }
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

    autoware_auto_planning_msgs::msg::Trajectory updated_traj;

    // 3. Interpolate velocities along those points
    if(control_mode == 0){
        updated_traj =
            InterpolateVelocities(current_trajectory, start_idx, end_idx, current_vehicle_state.value().twist.twist.linear.x, look_ahead_distance);
            RCLCPP_INFO(get_logger(), "Published updated trajectory with target velocity %.2f", target_velocity.value());
    }
    else if(control_mode == 1){
        updated_traj =
            InterpolateAccelerations(current_trajectory, start_idx, end_idx, current_acceleration.value().accel.accel.linear.x, current_vehicle_state.value().twist.twist.linear.x, look_ahead_distance);
            RCLCPP_INFO(get_logger(), "Published updated trajectory with target acc %.2f", target_acceleration.value());
            RCLCPP_INFO(get_logger(), "Nearest point %.2f", updated_traj.points[start_idx].longitudinal_velocity_mps);

    }
    else{
        updated_traj =
            InterpolateAccelerationsAndVelocity(current_trajectory, start_idx, end_idx, current_acceleration.value().accel.accel.linear.x, current_vehicle_state.value().twist.twist.linear.x, look_ahead_distance);
            RCLCPP_INFO(get_logger(), "Published updated trajectory with target acc %.2f and velocity %.2f", target_acceleration.value(), target_velocity.value());

    }

    // 4. Publish the updated trajectory
    if (updated_traj_publisher_) {
        updated_traj_publisher_->publish(updated_traj);
        current_trajectory = updated_traj;
        // RCLCPP_INFO(get_logger(), "Published updated trajectory with target velocity %.2f", target_velocity.value());
    } else {
        RCLCPP_WARN(get_logger(), "Updated trajectory publisher not initialized.");
    }
}

