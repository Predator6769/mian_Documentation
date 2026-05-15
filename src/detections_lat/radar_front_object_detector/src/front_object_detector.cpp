// #include <rclcpp/rclcpp.hpp>
// #include <radar_msgs/msg/radar_tracks.hpp>
// #include <visualization_msgs/msg/marker_array.hpp>
// #include <visualization_msgs/msg/marker.hpp>
// #include <geometry_msgs/msg/pose_array.hpp>
// #include <geometry_msgs/msg/point.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
// #include <std_msgs/msg/float64.hpp>
// #include <tier4_debug_msgs/msg/float64_stamped.hpp>
// #include <cmath>
// #include <limits>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>
// #include <deque>
// #include <algorithm>

// class FrontObjectDetector : public rclcpp::Node
// {
// public:
//   FrontObjectDetector()
//   : Node("front_object_detector")
//   {
//     sub_ = this->create_subscription<radar_msgs::msg::RadarTracks>(
//       "/radar_1/radar_tracks",
//       10,
//       std::bind(&FrontObjectDetector::tracksCallback, this, std::placeholders::_1)
//     );

//     // vehicle_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
//     //   "/localization/kinematic_state",
//     //   10,
//     //   std::bind(&FrontObjectDetector::Vehicle_state_callback, this, std::placeholders::_1)
//     // );

//     velocity_status_sub_ =
//       this->create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>(
//         "/vehicle/status/velocity_status",
//         10,
//         std::bind(&FrontObjectDetector::velocityStatusCallback, this, std::placeholders::_1)
//       );

//     marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
//       "/radar_1/front_track_marker", 10
//     );

//     closest_point_pub_ =
//       this->create_publisher<geometry_msgs::msg::PoseArray>(
//         "/radar_1/front_track_data", 10);
    
//     gap_pub_ =
//       this->create_publisher<tier4_debug_msgs::msg::Float64Stamped>(
//         "/acc/perception/gap", 10);
//     vel_pub_ =
//       this->create_publisher<tier4_debug_msgs::msg::Float64Stamped>(
//         "/acc/perception/velocity", 10);
//     acc_pub_ =
//       this->create_publisher<std_msgs::msg::Float64>(
//         "/acc/perception/acceleration", 10);
    
//     leader_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
//         "/radar_1/leader_track_marker", 10
// );

//     // Tunable parameters
//     max_angle_rad_ = this->declare_parameter("max_angle_deg", 5.0) * M_PI / 180.0;
//     y_base = this->declare_parameter("vehicle_width_half", 0.9);
//     longitudinal_limit = this->declare_parameter("longitudinal_limit", 50.0);
//     delta_x = this->declare_parameter("delta_x", 0.5);

//     median_window_ = this->declare_parameter("median_window", 5);
//     alpha_gap_     = this->declare_parameter("alpha_gap", 0.25);   // ~10Hz default
//     alpha_vel_     = this->declare_parameter("alpha_vel", 0.20);   // ~10Hz default
//     a_limit_       = this->declare_parameter("leader_acc_limit", 6.0);   // m/s^2
//     v_margin_gap_  = this->declare_parameter("gap_rate_v_margin", 10.0); // m/s

//     RCLCPP_INFO(this->get_logger(), "Front Object Detector initialized");
//   }

// private:

//   template<typename T>
//     static T clampT(T v, T lo, T hi) { return std::min(std::max(v, lo), hi); }

//     static double medianOfDeque(std::deque<double> d)
//     {
//       if (d.empty()) return std::numeric_limits<double>::quiet_NaN();
//       std::vector<double> v(d.begin(), d.end());
//       const size_t n = v.size();
//       std::nth_element(v.begin(), v.begin() + n/2, v.end());
//       double med = v[n/2];
//       if (n % 2 == 0) {
//         std::nth_element(v.begin(), v.begin() + (n/2 - 1), v.end());
//         med = 0.5 * (med + v[n/2 - 1]);
//       }
//       return med;
//     }

//     void pushWindow(std::deque<double>& w, double x)
//     {
//       w.push_back(x);
//       while ((int)w.size() > median_window_) w.pop_front();
//     }

//   double current_vel = 0.0, current_x = 0.0, current_y = 0.0, current_yaw = 0.0, min_y;
//   const double radar_x_e = 4.0, radar_y_e = 0.0, radar_z_e = 0.57, radar_yaw_e = 0.0, object_vel = 0.0;

//   std::deque<double> gap_win_;
//   std::deque<double> vlead_win_;

//   bool filt_initialized_ = false;
//   double gap_filt_ = 0.0;
//   double vlead_filt_ = 0.0;

//   rclcpp::Time last_filt_time_{0, 0, RCL_ROS_TIME};


//   // void Vehicle_state_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
//   // {
//   //   current_vel = msg->twist.twist.linear.x;
//   //   current_x   = msg->pose.pose.position.x;
//   //   current_y   = msg->pose.pose.position.y;

//   //   const auto &q_msg = msg->pose.pose.orientation;
//   //   tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);

//   //   double roll, pitch, yaw;
//   //   tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
//   //   current_yaw = yaw;
//   // }

//   void Vehicle_state_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
//   {
//     // current_vel = msg->twist.twist.linear.x;
//     current_x   = msg->pose.pose.position.x;
//     current_y   = msg->pose.pose.position.y;

//     const auto &q_msg = msg->pose.pose.orientation;
//     tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);

//     double roll, pitch, yaw;
//     tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
//     current_yaw = yaw;
//   }

//   void velocityStatusCallback(const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr msg)
//   {
//     current_vel = msg->longitudinal_velocity;
//   }

//   void tracksCallback(const radar_msgs::msg::RadarTracks::SharedPtr msg)
//   {
//     visualization_msgs::msg::Marker points;
//     points.header.frame_id = "radar_1";
//     points.header.stamp = msg->header.stamp;
//     points.ns = "radar_points";
//     points.id = 0;
//     points.type = visualization_msgs::msg::Marker::SPHERE_LIST;
//     points.action = visualization_msgs::msg::Marker::ADD;

//     points.color.r = 1.0;
//     points.color.g = 0.4;
//     points.color.b = 0.7;
//     points.color.a = 1.0;

//     points.scale.x = 1.0;
//     points.scale.y = 1.0;
//     points.scale.z = 1.0;

//     points.lifetime = rclcpp::Duration::from_seconds(0.005);


//     visualization_msgs::msg::Marker leader;
//     leader.header.frame_id = "radar_1";
//     leader.header.stamp = msg->header.stamp;
//     leader.ns = "leader_point";
//     leader.id = 0;
//     leader.type = visualization_msgs::msg::Marker::SPHERE;
//     leader.action = visualization_msgs::msg::Marker::ADD;

//     // Green
//     leader.color.r = 0.1;
//     leader.color.g = 1.0;
//     leader.color.b = 0.1;
//     leader.color.a = 1.0;

//     leader.scale.x = 1.4;
//     leader.scale.y = 1.4;
//     leader.scale.z = 1.4;

//     leader.lifetime = rclcpp::Duration::from_seconds(0.2);

//     // Optional: text marker for leader (NEW)
//     visualization_msgs::msg::Marker leader_text;
//     leader_text.header.frame_id = "radar_1";
//     leader_text.header.stamp = msg->header.stamp;
//     leader_text.ns = "leader_text";
//     leader_text.id = 1;
//     leader_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
//     leader_text.action = visualization_msgs::msg::Marker::ADD;

//     // White text
//     leader_text.color.r = 1.0;
//     leader_text.color.g = 1.0;
//     leader_text.color.b = 1.0;
//     leader_text.color.a = 1.0;
//     leader_text.scale.z = 1.0;
//     leader_text.lifetime = rclcpp::Duration::from_seconds(0.2);

//     int id = 0;

//     geometry_msgs::msg::PoseArray data;
//     data.header = msg->header;

//     double min_x = std::numeric_limits<double>::max();
//     double min_vel = 0.0, min_acc = 0.0;

//     geometry_msgs::msg::Point object_pose;

//     for (const auto & track : msg->tracks)
//     {
//       const double x = track.position.x;
//       const double y = track.position.y;

//       double const vel = (std::abs((track.velocity.x  + current_vel)) < 0.1) ? 0.0 : (track.velocity.x + current_vel);
//       const double acc = track.acceleration.x;

//       const double y_max = std::max(y_base, (x * std::tan(max_angle_rad_)));
//       double distance = std::sqrt(x * x + y * y);

//       if (x <= -0.3 || x >= longitudinal_limit) continue;
//       if (std::abs(y) > y_max) continue;

//       geometry_msgs::msg::Point p;
//       p.x = x;
//       p.y = y;
//       p.z = track.position.z;

//       geometry_msgs::msg::Pose poi;
//       poi.position.x = x;
//       poi.position.y = y;
//       poi.position.z = distance;
//       poi.orientation.x = vel;
//       poi.orientation.y = acc;

//       if (min_x > x)
//       {
//         min_x = x;
//         min_vel = vel;
//         min_acc = acc;
//         min_y = y;
//       }

//       data.poses.push_back(poi);

//       visualization_msgs::msg::Marker text;
//       text.header.frame_id = "radar_1";
//       text.header.stamp = msg->header.stamp;
//       text.ns = "radar_labels";
//       text.id = id++;
//       text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
//       text.action = visualization_msgs::msg::Marker::ADD;

//       text.pose.position.x = x;
//       text.pose.position.y = y;
//       text.pose.position.z = 0.6;

//       text.scale.z = 1.0;

//       text.color.r = 1.0;
//       text.color.g = 1.0;
//       text.color.b = 1.0;
//       text.color.a = 1.0;

//       text.text = "Vel: " + std::to_string(vel);
//       text.lifetime = rclcpp::Duration::from_seconds(0.2);

//       points.points.push_back(p);

//       marker_pub_->publish(text);
//       marker_pub_->publish(points);
//     }

//     const bool has_leader = (min_x != std::numeric_limits<double>::max());

//     if (has_leader)
//     {
//       // Timestamp + dt
//       rclcpp::Time t(msg->header.stamp);
//       if (last_filt_time_.nanoseconds() == 0) last_filt_time_ = t;

//       double dt = (t - last_filt_time_).seconds();
//       if (dt <= 1e-4) dt = 0.1; // fallback (prevents divide-by-zero / tiny dt)
//       dt = clampT(dt, 0.01, 0.5); // prevent crazy dt spikes

//       // Raw measurements
//       const double gap_raw = min_x;      // (your "gap" = longitudinal x in radar frame)
//       const double vlead_raw = min_vel;  // (your "absolute leader speed" proxy)

//       // --- Stage A: median window ---
//       pushWindow(gap_win_, gap_raw);
//       pushWindow(vlead_win_, vlead_raw);

//       const double gap_med = medianOfDeque(gap_win_);
//       const double vlead_med = medianOfDeque(vlead_win_);


//       const double v_max_step = a_limit_ * dt;
//       const double gap_max_step = (std::abs(current_vel) + v_margin_gap_) * dt;

//       // --- Stage B: LPF ---
//       if (!filt_initialized_)
//       {
//         gap_filt_ = gap_med;
//         vlead_filt_ = vlead_med;
//         filt_initialized_ = true;

//         gap_prev_ = gap_filt_;
//         vlead_prev_ = vlead_filt_;
//         last_filt_time_ = t;

//       }
//       else
//       {
//         gap_filt_   = gap_filt_   + alpha_gap_ * (gap_med   - gap_filt_);
//         vlead_filt_ = vlead_filt_ + alpha_vel_ * (vlead_med - vlead_filt_);
//         gap_filt_ = clampT(gap_filt_, gap_prev_ - gap_max_step, gap_prev_ + gap_max_step);

//         // Leader speed rate limit
//         vlead_filt_ = clampT(vlead_filt_, vlead_prev_ - v_max_step, vlead_prev_ + v_max_step);

//         gap_filt_   = clampT(gap_filt_,   0.0, longitudinal_limit);
//         vlead_filt_ = clampT(vlead_filt_, 0.0, 80.0); // allow slight negative if your convention can do it

//         // Update prev + time
//         gap_prev_ = gap_filt_;
//         vlead_prev_ = vlead_filt_;
//         last_filt_time_ = t;
//       }

//       // rate limiting (gentle) ---
//       // Gap rate limit

//       //clamp to plausible ranges
//       // gap_filt_   = clampT(gap_filt_,   0.0, longitudinal_limit);
//       // vlead_filt_ = clampT(vlead_filt_, 0.0, 80.0); // allow slight negative if your convention can do it

//       // // Update prev + time
//       // gap_prev_ = gap_filt_;
//       // vlead_prev_ = vlead_filt_;
//       // last_filt_time_ = t;

//       // Publish filtered
//       tier4_debug_msgs::msg::Float64Stamped final_vel;
//       final_vel.stamp = msg->header.stamp;
//       final_vel.data = vlead_filt_;

//       tier4_debug_msgs::msg::Float64Stamped final_longitudinal_x;
//       final_longitudinal_x.stamp = msg->header.stamp;
//       final_longitudinal_x.data = gap_filt_;

//       std_msgs::msg::Float64 final_acceleration;
//       final_acceleration.data = min_acc; // (left unfiltered for now)

//       vel_pub_->publish(final_vel);
//       acc_pub_->publish(final_acceleration);
//       gap_pub_->publish(final_longitudinal_x);
//     }
//     else
//     {
//       // No leader: keep your old behavior (don’t publish gap), but you can also reset if you want.
//       // If you prefer to keep filter state through brief dropouts, do nothing here.
//     }

//     // leader visualization
//     if (has_leader)
//     {
//       leader.pose.position.x = min_x;
//       leader.pose.position.y = min_y;
//       leader.pose.position.z = 0.2;

//       leader_text.pose.position.x = min_x;
//       leader_text.pose.position.y = min_y;
//       leader_text.pose.position.z = 1.2;

//       leader_text.scale.z = 1.0;

//       leader_text.color.r = 0.8;
//       leader_text.color.g = 0.8;
//       leader_text.color.b = 0.8;
//       leader_text.color.a = 1.0;

//       leader_text.text = "LEADER v=" + std::to_string(vlead_filt_) + " gap=" + std::to_string(gap_filt_);

//       leader_marker_pub_->publish(leader);
//       leader_marker_pub_->publish(leader_text);
//     }
//     else
//     {
//       // Delete old leader markers so RViz doesn't keep last one
//       leader.action = visualization_msgs::msg::Marker::DELETE;
//       leader_text.action = visualization_msgs::msg::Marker::DELETE;
//       leader_marker_pub_->publish(leader);
//       leader_marker_pub_->publish(leader_text);
//     }

//     closest_point_pub_->publish(data);
//   }


//   rclcpp::Subscription<radar_msgs::msg::RadarTracks>::SharedPtr sub_;
//   rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
//   rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr closest_point_pub_;
//   rclcpp::Publisher<tier4_debug_msgs::msg::Float64Stamped>::SharedPtr gap_pub_;
//   rclcpp::Publisher<tier4_debug_msgs::msg::Float64Stamped>::SharedPtr vel_pub_;
//   rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr acc_pub_;
//   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vehicle_state_sub_;
//   rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_status_sub_;
//   rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr leader_marker_pub_;

//   double max_angle_rad_;
//   double y_base;
//   double longitudinal_limit;
//   double delta_x;
//   double gap_prev_ = 0.0;
//   double vlead_prev_ = 0.0;
//   int median_window_;
//   double alpha_gap_ ;
//   double alpha_vel_ ;
//   double a_limit_;
//   double v_margin_gap_;
// };

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<FrontObjectDetector>());
//   rclcpp::shutdown();
//   return 0;
// }

#include <rclcpp/rclcpp.hpp>
#include <radar_msgs/msg/radar_tracks.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tier4_debug_msgs/msg/float64_stamped.hpp>
#include <cmath>
#include <limits>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <deque>
#include <algorithm>

class FrontObjectDetector : public rclcpp::Node
{
public:
  FrontObjectDetector()
  : Node("front_object_detector")
  {
    sub_ = this->create_subscription<radar_msgs::msg::RadarTracks>(
      "/radar_1/radar_tracks",
      10,
      std::bind(&FrontObjectDetector::tracksCallback, this, std::placeholders::_1)
    );

    // vehicle_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    //   "/localization/kinematic_state",
    //   10,
    //   std::bind(&FrontObjectDetector::Vehicle_state_callback, this, std::placeholders::_1)
    // );

    velocity_status_sub_ =
      this->create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>(
        "/vehicle/status/velocity_status",
        10,
        std::bind(&FrontObjectDetector::velocityStatusCallback, this, std::placeholders::_1)
      );

    accel_sub_ = 
      this->create_subscription<geometry_msgs::msg::AccelWithCovarianceStamped>(
        "/localization/acceleration",
        10,
        std::bind(&FrontObjectDetector::accelCallback, this, std::placeholders::_1)
      );

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/radar_1/front_track_marker", 10
    );

    closest_point_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseArray>(
        "/radar_1/front_track_data", 10);
    
    gap_pub_ =
      this->create_publisher<tier4_debug_msgs::msg::Float64Stamped>(
        "/acc/perception/gap", 10);
    vel_pub_ =
      this->create_publisher<tier4_debug_msgs::msg::Float64Stamped>(
        "/acc/perception/velocity", 10);
    acc_pub_ =
      this->create_publisher<tier4_debug_msgs::msg::Float64Stamped>(
        "/acc/perception/acceleration", 10);
    
    leader_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/radar_1/leader_track_marker", 10
    );

    // Tunable parameters
    max_angle_rad_ = this->declare_parameter("max_angle_deg", 5.0) * M_PI / 180.0;
    y_base = this->declare_parameter("vehicle_width_half", 0.9);
    longitudinal_limit = this->declare_parameter("longitudinal_limit", 50.0);
    delta_x = this->declare_parameter("delta_x", 0.5);

    median_window_ = this->declare_parameter("median_window", 5);
    alpha_gap_     = this->declare_parameter("alpha_gap", 0.25);   // ~10Hz default
    alpha_vel_     = this->declare_parameter("alpha_vel", 0.20);   // ~10Hz default
    alpha_acc_     = this->declare_parameter("alpha_acc", 0.15);   // ~10Hz default
    
    a_limit_       = this->declare_parameter("leader_acc_limit", 6.0);   // m/s^2 (for velocity rate limit)
    jerk_limit_    = this->declare_parameter("leader_jerk_limit", 10.0); // m/s^3 (for acceleration rate limit)
    v_margin_gap_  = this->declare_parameter("gap_rate_v_margin", 10.0); // m/s

    RCLCPP_INFO(this->get_logger(), "Front Object Detector initialized");
  }

private:

  template<typename T>
    static T clampT(T v, T lo, T hi) { return std::min(std::max(v, lo), hi); }

    static double medianOfDeque(std::deque<double> d)
    {
      if (d.empty()) return std::numeric_limits<double>::quiet_NaN();
      std::vector<double> v(d.begin(), d.end());
      const size_t n = v.size();
      std::nth_element(v.begin(), v.begin() + n/2, v.end());
      double med = v[n/2];
      if (n % 2 == 0) {
        std::nth_element(v.begin(), v.begin() + (n/2 - 1), v.end());
        med = 0.5 * (med + v[n/2 - 1]);
      }
      return med;
    }

    void pushWindow(std::deque<double>& w, double x)
    {
      w.push_back(x);
      while ((int)w.size() > median_window_) w.pop_front();
    }

  double current_vel = 0.0, current_accel = 0.0, current_x = 0.0, current_y = 0.0, current_yaw = 0.0, min_y;
  const double radar_x_e = 4.0, radar_y_e = 0.0, radar_z_e = 0.57, radar_yaw_e = 0.0, object_vel = 0.0;

  std::deque<double> gap_win_;
  std::deque<double> vlead_win_;
  std::deque<double> alead_win_;

  bool filt_initialized_ = false;
  double gap_filt_ = 0.0;
  double vlead_filt_ = 0.0;
  double alead_filt_ = 0.0;

  rclcpp::Time last_filt_time_{0, 0, RCL_ROS_TIME};

  void Vehicle_state_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // current_vel = msg->twist.twist.linear.x;
    current_x   = msg->pose.pose.position.x;
    current_y   = msg->pose.pose.position.y;

    const auto &q_msg = msg->pose.pose.orientation;
    tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    current_yaw = yaw;
  }

  void velocityStatusCallback(const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr msg)
  {
    current_vel = msg->longitudinal_velocity;
  }

  void accelCallback(const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr msg)
  {
    current_accel = msg->accel.accel.linear.x;
  }

  void tracksCallback(const radar_msgs::msg::RadarTracks::SharedPtr msg)
  {
    visualization_msgs::msg::Marker points;
    points.header.frame_id = "radar_1";
    points.header.stamp = msg->header.stamp;
    points.ns = "radar_points";
    points.id = 0;
    points.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    points.action = visualization_msgs::msg::Marker::ADD;

    points.color.r = 1.0;
    points.color.g = 0.4;
    points.color.b = 0.7;
    points.color.a = 1.0;

    points.scale.x = 1.0;
    points.scale.y = 1.0;
    points.scale.z = 1.0;

    points.lifetime = rclcpp::Duration::from_seconds(0.005);


    visualization_msgs::msg::Marker leader;
    leader.header.frame_id = "radar_1";
    leader.header.stamp = msg->header.stamp;
    leader.ns = "leader_point";
    leader.id = 0;
    leader.type = visualization_msgs::msg::Marker::SPHERE;
    leader.action = visualization_msgs::msg::Marker::ADD;

    // Green
    leader.color.r = 0.1;
    leader.color.g = 1.0;
    leader.color.b = 0.1;
    leader.color.a = 1.0;

    leader.scale.x = 1.4;
    leader.scale.y = 1.4;
    leader.scale.z = 1.4;

    leader.lifetime = rclcpp::Duration::from_seconds(0.2);

    // Optional: text marker for leader (NEW)
    visualization_msgs::msg::Marker leader_text;
    leader_text.header.frame_id = "radar_1";
    leader_text.header.stamp = msg->header.stamp;
    leader_text.ns = "leader_text";
    leader_text.id = 1;
    leader_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    leader_text.action = visualization_msgs::msg::Marker::ADD;

    // White text
    leader_text.color.r = 1.0;
    leader_text.color.g = 1.0;
    leader_text.color.b = 1.0;
    leader_text.color.a = 1.0;
    leader_text.scale.z = 1.0;
    leader_text.lifetime = rclcpp::Duration::from_seconds(0.2);

    int id = 0;

    geometry_msgs::msg::PoseArray data;
    data.header = msg->header;

    double min_x = std::numeric_limits<double>::max();
    double min_vel = 0.0, min_acc = 0.0;

    geometry_msgs::msg::Point object_pose;

    for (const auto & track : msg->tracks)
    {
      const double x = track.position.x;
      const double y = track.position.y;

      double const vel = (std::abs((track.velocity.x  + current_vel)) < 0.1) ? 0.0 : (track.velocity.x + current_vel);
      const double acc = track.acceleration.x + current_accel; // Relative acceleration from radar

      const double y_max = std::max(y_base, (x * std::tan(max_angle_rad_)));
      double distance = std::sqrt(x * x + y * y);

      if (x <= -0.3 || x >= longitudinal_limit) continue;
      if (std::abs(y) > y_max) continue;

      geometry_msgs::msg::Point p;
      p.x = x;
      p.y = y;
      p.z = track.position.z;

      geometry_msgs::msg::Pose poi;
      poi.position.x = x;
      poi.position.y = y;
      poi.position.z = distance;
      poi.orientation.x = vel;
      poi.orientation.y = acc;

      if (min_x > x)
      {
        min_x = x;
        min_vel = vel;
        min_acc = acc;
        min_y = y;
      }

      data.poses.push_back(poi);

      visualization_msgs::msg::Marker text;
      text.header.frame_id = "radar_1";
      text.header.stamp = msg->header.stamp;
      text.ns = "radar_labels";
      text.id = id++;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;

      text.pose.position.x = x;
      text.pose.position.y = y;
      text.pose.position.z = 0.6;

      text.scale.z = 1.0;

      text.color.r = 1.0;
      text.color.g = 1.0;
      text.color.b = 1.0;
      text.color.a = 1.0;

      text.text = "Vel: " + std::to_string(vel);
      text.lifetime = rclcpp::Duration::from_seconds(0.2);

      points.points.push_back(p);

      marker_pub_->publish(text);
      marker_pub_->publish(points);
    }

    const bool has_leader = (min_x != std::numeric_limits<double>::max());

    if (has_leader)
    {
      // Timestamp + dt
      rclcpp::Time t(msg->header.stamp);
      if (last_filt_time_.nanoseconds() == 0) last_filt_time_ = t;

      double dt = (t - last_filt_time_).seconds();
      if (dt <= 1e-4) dt = 0.1; // fallback (prevents divide-by-zero / tiny dt)
      dt = clampT(dt, 0.01, 0.5); // prevent crazy dt spikes

      // Raw measurements
      const double gap_raw = min_x;      // (your "gap" = longitudinal x in radar frame)
      const double vlead_raw = min_vel;  // absolute leader speed proxy
      const double alead_raw = min_acc; // Absolute leader acceleration = relative + ego accel

      // --- Stage A: median window ---
      pushWindow(gap_win_, gap_raw);
      pushWindow(vlead_win_, vlead_raw);
      pushWindow(alead_win_, alead_raw);

      const double gap_med = medianOfDeque(gap_win_);
      const double vlead_med = medianOfDeque(vlead_win_);
      const double alead_med = medianOfDeque(alead_win_);

      const double v_max_step = a_limit_ * dt;
      const double gap_max_step = (std::abs(current_vel) + v_margin_gap_) * dt;
      const double a_max_step = jerk_limit_ * dt;

      // --- Stage B: LPF ---
      if (!filt_initialized_)
      {
        gap_filt_ = gap_med;
        vlead_filt_ = vlead_med;
        alead_filt_ = alead_med;
        filt_initialized_ = true;

        gap_prev_ = gap_filt_;
        vlead_prev_ = vlead_filt_;
        alead_prev_ = alead_filt_;
        last_filt_time_ = t;
      }
      else
      {
        gap_filt_   = gap_filt_   + alpha_gap_ * (gap_med   - gap_filt_);
        vlead_filt_ = vlead_filt_ + alpha_vel_ * (vlead_med - vlead_filt_);
        alead_filt_ = alead_filt_ + alpha_acc_ * (alead_med - alead_filt_);

        // Rate limiters
        gap_filt_ = clampT(gap_filt_, gap_prev_ - gap_max_step, gap_prev_ + gap_max_step);
        vlead_filt_ = clampT(vlead_filt_, vlead_prev_ - v_max_step, vlead_prev_ + v_max_step);
        alead_filt_ = clampT(alead_filt_, alead_prev_ - a_max_step, alead_prev_ + a_max_step);

        // Value clamping (negative accel allowed)
        gap_filt_   = clampT(gap_filt_,   0.0, longitudinal_limit);
        vlead_filt_ = clampT(vlead_filt_, 0.0, 80.0); 
        alead_filt_ = clampT(alead_filt_, -10.0, 10.0); // Allow decel and accel within realistic bounds

        // Update prev + time
        gap_prev_ = gap_filt_;
        vlead_prev_ = vlead_filt_;
        alead_prev_ = alead_filt_;
        last_filt_time_ = t;
      }

      // Publish filtered
      tier4_debug_msgs::msg::Float64Stamped final_vel;
      final_vel.stamp = msg->header.stamp;
      final_vel.data = vlead_filt_;

      tier4_debug_msgs::msg::Float64Stamped final_longitudinal_x;
      final_longitudinal_x.stamp = msg->header.stamp;
      final_longitudinal_x.data = gap_filt_;

      tier4_debug_msgs::msg::Float64Stamped final_acceleration;
      final_acceleration.stamp = msg->header.stamp;
      final_acceleration.data = alead_filt_; // Now uses filtered absolute leader acceleration

      vel_pub_->publish(final_vel);
      acc_pub_->publish(final_acceleration);
      gap_pub_->publish(final_longitudinal_x);
    }

    // leader visualization
    if (has_leader)
    {
      leader.pose.position.x = min_x;
      leader.pose.position.y = min_y;
      leader.pose.position.z = 0.2;

      leader_text.pose.position.x = min_x;
      leader_text.pose.position.y = min_y;
      leader_text.pose.position.z = 1.2;

      leader_text.scale.z = 1.0;

      leader_text.color.r = 0.8;
      leader_text.color.g = 0.8;
      leader_text.color.b = 0.8;
      leader_text.color.a = 1.0;

      // Updated visualization to include filtered acceleration
      leader_text.text = "LEADER v=" + std::to_string(vlead_filt_) + 
                         " a=" + std::to_string(alead_filt_) + 
                         " gap=" + std::to_string(gap_filt_);

      leader_marker_pub_->publish(leader);
      leader_marker_pub_->publish(leader_text);
    }
    else
    {
      // Delete old leader markers so RViz doesn't keep last one
      leader.action = visualization_msgs::msg::Marker::DELETE;
      leader_text.action = visualization_msgs::msg::Marker::DELETE;
      leader_marker_pub_->publish(leader);
      leader_marker_pub_->publish(leader_text);
    }

    closest_point_pub_->publish(data);
  }


  rclcpp::Subscription<radar_msgs::msg::RadarTracks>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr closest_point_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float64Stamped>::SharedPtr gap_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float64Stamped>::SharedPtr vel_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float64Stamped>::SharedPtr acc_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vehicle_state_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr accel_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr leader_marker_pub_;

  double max_angle_rad_;
  double y_base;
  double longitudinal_limit;
  double delta_x;
  double gap_prev_ = 0.0;
  double vlead_prev_ = 0.0;
  double alead_prev_ = 0.0;
  int median_window_;
  double alpha_gap_ ;
  double alpha_vel_ ;
  double alpha_acc_ ;
  double a_limit_;
  double jerk_limit_;
  double v_margin_gap_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontObjectDetector>());
  rclcpp::shutdown();
  return 0;
}