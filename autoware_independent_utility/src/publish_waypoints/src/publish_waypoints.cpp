// #include "rclcpp/rclcpp.hpp"
// #include "autoware_auto_planning_msgs/msg/trajectory.hpp"
// #include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"
// #include "builtin_interfaces/msg/duration.hpp"
// #include "geometry_msgs/msg/pose.hpp"

// #include <fstream>
// #include <sstream>
// #include <string>
// #include <vector>
// // #include <filesystem>

// class CsvToTrajectoryNode : public rclcpp::Node
// {
// public:
//   CsvToTrajectoryNode() : Node("csv_to_trajectory_node"), has_published_(false)
//   {
//     publisher_ = this->create_publisher<autoware_auto_planning_msgs::msg::Trajectory>(
//       "/planning/scenario_planning/trajectory", 10);

//     std::string home_dir = std::getenv("HOME");
//     csv_file_path_ = home_dir + "/trajectory_output_2.csv";

//     if (!read_csv_to_trajectory(csv_file_path_)) {
//       RCLCPP_ERROR(this->get_logger(), "Failed to read trajectory from CSV.");
//     }

//     timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
//                                      std::bind(&CsvToTrajectoryNode::publish_trajectory, this));
//     timer_2 = this->create_wall_timer(std::chrono::milliseconds(100),
//                                      std::bind(&CsvToTrajectoryNode::read_csv_to_trajectory, this));
//   }

// private:
//   bool read_csv_to_trajectory(const std::string & file_path)
//   {
//     std::ifstream file(file_path);
//     if (!file.is_open()) {
//       RCLCPP_ERROR(this->get_logger(), "Unable to open CSV file: %s", file_path.c_str());
//       return false;
//     }

//     std::string line;
//     std::getline(file, line);  // Skip header line

//     while (std::getline(file, line)) {
//       std::stringstream ss(line);
//       std::string cell;
//       std::vector<double> values;

//       while (std::getline(ss, cell, ',')) {
//         values.push_back(std::stod(cell));
//       }

//       if (values.size() < 11) continue;

//       autoware_auto_planning_msgs::msg::TrajectoryPoint tp;
//       tp.pose.position.x = values[0];
//       tp.pose.position.y = values[1];
//       tp.pose.position.z = values[2];
//       tp.pose.orientation.x = values[3];
//       tp.pose.orientation.y = values[4];
//       tp.pose.orientation.z = values[5];
//       tp.pose.orientation.w = values[6];
//       tp.longitudinal_velocity_mps = values[7];
//       tp.acceleration_mps2 = values[10];
//       tp.lateral_velocity_mps = values[8];
//       tp.heading_rate_rps = values[9];
//       tp.front_wheel_angle_rad = 0.0;
//       tp.rear_wheel_angle_rad = 0.0;
//       tp.time_from_start = builtin_interfaces::msg::Duration();

//       trajectory_.points.push_back(tp);
//     }

//     trajectory_.header.frame_id = "map";
//     return true;
//   }

//   void publish_trajectory()
//   {
//     // if (!has_published_) {
//       trajectory_.header.stamp = this->get_clock()->now();
//       publisher_->publish(trajectory_);
//       RCLCPP_INFO(this->get_logger(), "Published trajectory with %ld points.", trajectory_.points.size());
//       has_published_ = true;
//     // }
//   }

//   std::string csv_file_path_;
//   bool has_published_;
//   autoware_auto_planning_msgs::msg::Trajectory trajectory_;
//   rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr publisher_;
//   rclcpp::TimerBase::SharedPtr timer_;
//   rclcpp::TimerBase::SharedPtr timer_2;
// };

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<CsvToTrajectoryNode>());
//   rclcpp::shutdown();
//   return 0;
// }
#include "rclcpp/rclcpp.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <mutex>
#include <optional>
#include <filesystem>  // C++17

namespace fs = std::filesystem;

class CsvToTrajectoryNode : public rclcpp::Node
{
public:
  CsvToTrajectoryNode()
  : Node("csv_to_trajectory_node")
  {
    // Parameters (all dynamic so you can override via CLI/launch)
    csv_file_path_ = this->declare_parameter<std::string>(
      "csv_path", (std::string)std::getenv("HOME") + "/parking_lot_small_loop_record_test.csv");
    topic_ = this->declare_parameter<std::string>(
      "topic", "/planning/scenario_planning/trajectory");
    auto publish_period_ms = this->declare_parameter<int>("publish_period_ms", 100);
    auto reload_period_ms  = this->declare_parameter<int>("reload_period_ms", 200);

    publisher_ = this->create_publisher<autoware_auto_planning_msgs::msg::Trajectory>(topic_, 10);

    // Initial load (best effort)
    maybe_reload_from_disk(/*force=*/true);

    // Timer that always publishes the *current* trajectory
    publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(publish_period_ms),
      [this]() { publish_trajectory(); });

    // Timer that checks whether the CSV changed and reloads if needed
    reload_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(reload_period_ms),
      [this]() { maybe_reload_from_disk(/*force=*/false); });

    RCLCPP_INFO(this->get_logger(), "csv_path: %s", csv_file_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "topic: %s", topic_.c_str());
  }

private:
  // Parse helper: returns a Trajectory if load succeeded
  std::optional<autoware_auto_planning_msgs::msg::Trajectory>
  load_csv_to_trajectory(const std::string & file_path)
  {
    std::ifstream file(file_path);
    if (!file.is_open()) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                            "Unable to open CSV file: %s", file_path.c_str());
      return std::nullopt;
    }

    autoware_auto_planning_msgs::msg::Trajectory traj;
    traj.header.frame_id = "map";
    traj.points.clear();

    std::string line;
    // Skip header if present
    if (std::getline(file, line)) {
      // If the first line is not numeric CSV (e.g., header), that's fine—we'll just skip it
      // Try to detect: if it contains letters, treat as header; otherwise treat it as data
      bool has_alpha = false;
      for (char c : line) { if (std::isalpha(static_cast<unsigned char>(c))) { has_alpha = true; break; } }
      if (!has_alpha) {
        // First line was data; process it as well
        std::stringstream ss(line);
        std::string cell;
        std::vector<double> values;
        while (std::getline(ss, cell, ',')) {
          if (!cell.empty()) values.push_back(std::stod(cell));
        }
        if (values.size() >= 11) {
          autoware_auto_planning_msgs::msg::TrajectoryPoint tp;
          tp.pose.position.x = values[0];
          tp.pose.position.y = values[1];
          tp.pose.position.z = values[2];
          tp.pose.orientation.x = values[3];
          tp.pose.orientation.y = values[4];
          tp.pose.orientation.z = values[5];
          tp.pose.orientation.w = values[6];
          tp.longitudinal_velocity_mps = values[7];
          tp.lateral_velocity_mps      = values[8];
          tp.heading_rate_rps          = values[9];
          tp.acceleration_mps2         = values[10];
          tp.front_wheel_angle_rad = 0.0;
          tp.rear_wheel_angle_rad  = 0.0;
          tp.time_from_start = builtin_interfaces::msg::Duration();
          traj.points.push_back(tp);
        }
      }
    }

    // Read remaining lines
    while (std::getline(file, line)) {
      if (line.empty()) continue;
      std::stringstream ss(line);
      std::string cell;
      std::vector<double> values;
      while (std::getline(ss, cell, ',')) {
        if (!cell.empty()) values.push_back(std::stod(cell));
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
      tp.lateral_velocity_mps      = values[8];
      tp.heading_rate_rps          = values[9];
      tp.acceleration_mps2         = values[10];
      tp.front_wheel_angle_rad = 0.0;
      tp.rear_wheel_angle_rad  = 0.0;
      tp.time_from_start = builtin_interfaces::msg::Duration();

      traj.points.push_back(tp);
    }

    if (traj.points.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                           "CSV loaded but produced 0 trajectory points.");
    }

    return traj;
  }

  void maybe_reload_from_disk(bool force)
  {
    try {
      if (!fs::exists(csv_file_path_)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "CSV path does not exist yet: %s", csv_file_path_.c_str());
        return;
      }
      auto mtime = fs::last_write_time(csv_file_path_);
      if (!force && last_mtime_.has_value() && mtime == *last_mtime_) {
        // No change
        return;
      }

      auto maybe_traj = load_csv_to_trajectory(csv_file_path_);
      if (!maybe_traj.has_value()) {
        return;  // error already logged
      }

      // Swap in the new trajectory under lock
      {
        std::lock_guard<std::mutex> lk(mutex_);
        trajectory_ = std::move(*maybe_traj);
      }
      last_mtime_ = mtime;
      RCLCPP_INFO(this->get_logger(), "Reloaded trajectory from CSV (%zu points).",
                  trajectory_.points.size());

    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Exception checking/reloading CSV: %s", e.what());
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Unknown error checking/reloading CSV.");
    }
  }

  void publish_trajectory()
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (trajectory_.points.empty()) {
      // Nothing to publish yet
      return;
    }
    trajectory_.header.stamp = this->get_clock()->now();
    publisher_->publish(trajectory_);
    // Keep logs light—info at ~1 Hz would be typical; here we don't spam.
    RCLCPP_DEBUG(this->get_logger(), "Published trajectory with %zu points.", trajectory_.points.size());
  }

  // Params/state
  std::string csv_file_path_;
  std::string topic_;
  std::optional<fs::file_time_type> last_mtime_;

  // ROS I/O
  rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr reload_timer_;

  // Shared trajectory (guarded)
  autoware_auto_planning_msgs::msg::Trajectory trajectory_;
  std::mutex mutex_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CsvToTrajectoryNode>());
  rclcpp::shutdown();
  return 0;
}
