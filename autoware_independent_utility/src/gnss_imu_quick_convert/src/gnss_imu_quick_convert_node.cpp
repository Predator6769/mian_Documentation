// src/gnss_imu_quick_convert_node.cpp

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "radar_msgs/msg/radar_tracks.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "gps_msgs/msg/gps_fix.hpp"
#include "novatel_oem7_msgs/msg/bestvel.hpp"
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "novatel_oem7_msgs/msg/heading2.hpp"
#include "novatel_oem7_msgs/msg/inspvax.hpp"
#include <autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "novatel_oem7_msgs/msg/bestgnsspos.hpp"
#include "novatel_oem7_msgs/msg/inspva.hpp"
#include <angles/angles.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/float32.hpp>

using std::placeholders::_1;

class GnssImuQuickConvert : public rclcpp::Node
{
public:
  
  GnssImuQuickConvert(const rclcpp::NodeOptions & options)
  : Node("gnss_imu_quick_convert", options)
  {
    auto qos = rclcpp::QoS(10);
    gnss_inspva_sideslip_ = create_subscription<novatel_oem7_msgs::msg::INSPVA>(
      "/novatel/oem7/inspva", qos,
      std::bind(&GnssImuQuickConvert::gnss_inspvax_sideslip, this, _1));
    gnss_inspva_ = create_subscription<novatel_oem7_msgs::msg::INSPVA>(
      "/novatel/oem7/inspva", qos,
      std::bind(&GnssImuQuickConvert::gnss_inspva, this, _1));
    localization_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      "/localization/kinematic_state", qos,
      std::bind(&GnssImuQuickConvert::localization_odom, this, _1));
    gnss_heading_ = create_subscription<novatel_oem7_msgs::msg::HEADING2>(
      "/novatel/oem7/heading2", qos,
      std::bind(&GnssImuQuickConvert::gnss_heading, this, _1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/novatel/oem7/imu/data_raw", qos,
      std::bind(&GnssImuQuickConvert::imu_callback, this, _1));
    imu_pro_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/novatel/oem7/imu/data", qos,
      std::bind(&GnssImuQuickConvert::imu_pro_callback, this, _1));  
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      "/novatel/oem7/imu/data_raw_frame", qos);
    gnss_compass_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/localization/pose_twist_estimator/eagleye/gnss_compass_pose", qos);
    gnss_orientation_pub_ = create_publisher<autoware_sensing_msgs::msg::GnssInsOrientationStamped>(
      "/autoware_orientation", qos);
    gnss_sub_ = create_subscription<gps_msgs::msg::GPSFix>(
      "/novatel/oem7/gps", qos,
      std::bind(&GnssImuQuickConvert::gnss_callback, this, _1));
    
    gnss_coverted_pose = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/sensing/gnss/pose_with_covariance", qos,
      std::bind(&GnssImuQuickConvert::gnss_conv_callback, this, _1));
    // gnss_sub_ = create_subscription<novatel_oem7_msgs::msg::BESTGNSSPOS>(
    //   "/novatel/oem7/bestgnsspos", qos,
    //   std::bind(&GnssImuQuickConvert::gnss_callback, this, _1));    
    gnss_vel_sub_ = create_subscription<novatel_oem7_msgs::msg::BESTVEL>(
      "/novatel/oem7/bestvel", qos,
      std::bind(&GnssImuQuickConvert::gnss_vel_callback, this, _1));
    gnss_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(
      "/novatel/oem7/fix_frame", qos);
    gnss_vel_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/novatel/oem7/twist", qos);

    radar_sub_ = create_subscription<radar_msgs::msg::RadarTracks>(
      "/radar_1/radar_tracks", qos,
      std::bind(&GnssImuQuickConvert::radar_callback, this, _1));
    radar_pub_ = create_publisher<radar_msgs::msg::RadarTracks>(
      "/sensing/radar_1/objects_raw", qos);

    beta_pub_ = create_publisher<std_msgs::msg::Float32>("/sideslip", 10);

    RCLCPP_INFO(get_logger(), "gnss_imu_quick_convert started");
  }

private:

  novatel_oem7_msgs::msg::HEADING2 gnss_heading_info;

  sensor_msgs::msg::Imu pro_imu;

  nav_msgs::msg::Odometry odom;

  void localization_odom(nav_msgs::msg::Odometry::SharedPtr msg){
    this->odom = *msg;
  }

  void gnss_inspvax_sideslip(novatel_oem7_msgs::msg::INSPVA::SharedPtr msg){
    const double vE = msg->east_velocity;
    const double vN = msg->north_velocity;

    /* 2. Course over ground γ ----------------------------------- */
    const double gamma = std::atan2(vN, vE);     // atan2(+N,+E) : ENU 0 = East

    /* 3. Vehicle yaw ψ from fused pose -------------------------- */
    tf2::Quaternion q;
    tf2::fromMsg(odom.pose.pose.orientation, q);
    double roll, pitch, yaw_ENU;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_ENU);   // yaw_ENU = ψ

    /* 4. Sideslip β --------------------------------------------- */
    const double beta = angles::normalize_angle(gamma - yaw_ENU);

    /* 5. Publish ------------------------------------------------- */
    std_msgs::msg::Float32 beta_msg;
    // rclcpp::Time now = this->get_clock()->now();
    // int64_t nsec = now.nanoseconds();
    // // fill the builtin_interfaces::msg::Time fields
    // beta_msg.header.stamp.sec     = static_cast<int32_t>(nsec / 1000000000ULL);
    // beta_stamped.header.stamp.nanosec = static_cast<uint32_t>(nsec % 1000000000ULL); 
    // beta_msg.header.frame_id = "base_link";
    beta_msg.data  = static_cast<float>(beta);
    beta_pub_->publish(beta_msg);
  }

  void gnss_conv_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.pose = msg->pose.pose;
    rclcpp::Time now = this->get_clock()->now();
    int64_t nsec = now.nanoseconds();
    // fill the builtin_interfaces::msg::Time fields
    pose_stamped.header.stamp.sec     = static_cast<int32_t>(nsec / 1000000000ULL);
    pose_stamped.header.stamp.nanosec = static_cast<uint32_t>(nsec % 1000000000ULL); 
    pose_stamped.header.frame_id = "gnss_link";

    // gnss_compass_pub_->publish(pose_stamped);

  }

  void gnss_inspva(novatel_oem7_msgs::msg::INSPVA::SharedPtr msg){
    tf2::Quaternion q;
    double pitch = static_cast<double>(msg->pitch) * M_PI / 180.0;
    double roll = static_cast<double>(msg->roll) * M_PI / 180.0;
    double azimuth = static_cast<double>((msg->azimuth) - 90) * M_PI / 180.0;
    // double azimuth_1 = static_cast<double>((msg->azimuth)) * M_PI / 180.0;

        /*
        * in clap b7 roll-> y-axis pitch-> x axis azimuth->left-handed rotation around z-axis
        * in ros imu msg roll-> x-axis pitch-> y axis azimuth->right-handed rotation around z-axis
        */
    q.setRPY(pitch, roll, -azimuth);

    geometry_msgs::msg::PoseStamped pose_stamped;
    autoware_sensing_msgs::msg::GnssInsOrientationStamped orientation_msg;
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();
    rclcpp::Time now = this->get_clock()->now();
    int64_t nsec = now.nanoseconds();
    // fill the builtin_interfaces::msg::Time fields
    pose_stamped.header.stamp.sec     = static_cast<int32_t>(nsec / 1000000000ULL);
    pose_stamped.header.stamp.nanosec = static_cast<uint32_t>(nsec % 1000000000ULL); 
    pose_stamped.header.frame_id = "gnss_link";

    gnss_compass_pub_->publish(pose_stamped);

    orientation_msg.orientation.orientation.x = q.x();
    orientation_msg.orientation.orientation.y = q.y();
    orientation_msg.orientation.orientation.z = q.z();
    orientation_msg.orientation.orientation.w = q.w();


    // orientation_msg.orientation.rmse_rotation_x = msg->pitch_stdev * msg->pitch_stdev;
    // orientation_msg.orientation.rmse_rotation_y = msg->roll_stdev * msg->roll_stdev;
    // orientation_msg.orientation.rmse_rotation_z = msg->azimuth_stdev * msg->azimuth_stdev;

    // rclcpp::Time now = this->get_clock()->now();
    // int64_t nsec = now.nanoseconds();
    // fill the builtin_interfaces::msg::Time fields
    orientation_msg.header.stamp.sec     = static_cast<int32_t>(nsec / 1000000000ULL);
    orientation_msg.header.stamp.nanosec = static_cast<uint32_t>(nsec % 1000000000ULL);
    orientation_msg.header.frame_id = "gnss_link";
    gnss_orientation_pub_->publish(orientation_msg);

  }

  void gnss_inspvax(novatel_oem7_msgs::msg::INSPVAX::SharedPtr msg){
    autoware_sensing_msgs::msg::GnssInsOrientationStamped orientation_msg;
    tf2::Quaternion q,q1;
    double pitch = static_cast<double>(msg->pitch) * M_PI / 180.0;
    double roll = static_cast<double>(msg->roll) * M_PI / 180.0;
    double azimuth = static_cast<double>((msg->azimuth) - 90) * M_PI / 180.0;
    double azimuth_1 = static_cast<double>((msg->azimuth)) * M_PI / 180.0;

        /*
        * in clap b7 roll-> y-axis pitch-> x axis azimuth->left-handed rotation around z-axis
        * in ros imu msg roll-> x-axis pitch-> y axis azimuth->right-handed rotation around z-axis
        */
    q.setRPY(pitch, roll, -azimuth);
    q1.setRPY(pitch, roll, azimuth_1);

    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();
    rclcpp::Time now = this->get_clock()->now();
    int64_t nsec = now.nanoseconds();
    // fill the builtin_interfaces::msg::Time fields
    pose_stamped.header.stamp.sec     = static_cast<int32_t>(nsec / 1000000000ULL);
    pose_stamped.header.stamp.nanosec = static_cast<uint32_t>(nsec % 1000000000ULL); 
    pose_stamped.header.frame_id = "gnss_link";

    // gnss_compass_pub_->publish(pose_stamped);

    orientation_msg.orientation.orientation.x = q.x();
    orientation_msg.orientation.orientation.y = q.y();
    orientation_msg.orientation.orientation.z = q.z();
    orientation_msg.orientation.orientation.w = q.w();


    orientation_msg.orientation.rmse_rotation_x = msg->pitch_stdev * msg->pitch_stdev;
    orientation_msg.orientation.rmse_rotation_y = msg->roll_stdev * msg->roll_stdev;
    orientation_msg.orientation.rmse_rotation_z = msg->azimuth_stdev * msg->azimuth_stdev;

    // rclcpp::Time now = this->get_clock()->now();
    // int64_t nsec = now.nanoseconds();
    // fill the builtin_interfaces::msg::Time fields
    orientation_msg.header.stamp.sec     = static_cast<int32_t>(nsec / 1000000000ULL);
    orientation_msg.header.stamp.nanosec = static_cast<uint32_t>(nsec % 1000000000ULL);
    orientation_msg.header.frame_id = "gnss_link";
    gnss_orientation_pub_->publish(orientation_msg);
  }

  void imu_pro_callback(const sensor_msgs::msg::Imu::SharedPtr msg){
    this->pro_imu = *msg;
  }

  void gnss_heading(novatel_oem7_msgs::msg::HEADING2::SharedPtr msg){
    this->gnss_heading_info = *msg;
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    auto out = *msg;
    rclcpp::Time now = this->get_clock()->now();
    int64_t nsec = now.nanoseconds();
    // fill the builtin_interfaces::msg::Time fields
    out.header.stamp.sec     = static_cast<int32_t>(nsec / 1000000000ULL);
    out.header.stamp.nanosec = static_cast<uint32_t>(nsec % 1000000000ULL);
    out.header.frame_id = "imu_link";
    imu_pub_->publish(out);
  }

  void gnss_callback(const gps_msgs::msg::GPSFix::SharedPtr msg)
  {
  // void gnss_callback(const novatel_oem7_msgs::msg::BESTGNSSPOS::SharedPtr msg)
  // {
    sensor_msgs::msg::NavSatFix out;

    // Copy and retime the header
    out.header = msg->header;
    rclcpp::Time now = this->get_clock()->now();
    int64_t nsec = now.nanoseconds();
    // fill the builtin_interfaces::msg::Time fields
    out.header.stamp.sec     = static_cast<int32_t>(nsec / 1000000000ULL);
    out.header.stamp.nanosec = static_cast<uint32_t>(nsec % 1000000000ULL); 
    out.header.frame_id = "gnss_link";
    out.status.status = 2;
    out.status.service = 3;
    // Map the core fields
    out.latitude  = msg->latitude;
    out.longitude = msg->longitude;
    out.altitude  = msg->altitude;

    // out.position_covariance = msg->position_covariance;

    // If GPSFix provides a 3×3 covariance matrix, copy it directly.
    // Otherwise you can set it to UNKNOWN or a default.
    if (msg->position_covariance.size() == 9) {
      for (size_t i = 0; i < 9; ++i) {
        out.position_covariance[i] = msg->position_covariance[i];
      }
      out.position_covariance_type = 
        sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN;
    } else {
      // no covariance provided
      out.position_covariance_type = 
        sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    }

    gnss_pub_->publish(out);
  }

  void gnss_vel_callback(const novatel_oem7_msgs::msg::BESTVEL::SharedPtr msg){
    geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
    twist_msg.header = msg->header;
    rclcpp::Time now = this->get_clock()->now();
    int64_t nsec = now.nanoseconds();
    // fill the builtin_interfaces::msg::Time fields
    twist_msg.header.stamp.sec     = static_cast<int32_t>(nsec / 1000000000ULL);
    twist_msg.header.stamp.nanosec = static_cast<uint32_t>(nsec % 1000000000ULL);

    twist_msg.header.frame_id = "gnss_link";
    double speed     = static_cast<double>(msg->hor_speed);
    double track_rad = static_cast<double>(msg->trk_gnd) * M_PI / 180.0;
    double heading = static_cast<double>(this->gnss_heading_info.heading) * M_PI / 180.0;

    if(cos(heading - track_rad) < 0.0) {
        twist_msg.twist.twist.linear.x = speed;
    }
    else{
        twist_msg.twist.twist.linear.x = -speed;
    }

    twist_msg.twist.twist.linear.y = 0.0;
    twist_msg.twist.twist.linear.z = 0.0;

    twist_msg.twist.twist.angular.z = pro_imu.angular_velocity.z;
    twist_msg.twist.twist.angular.y = pro_imu.angular_velocity.y;
    twist_msg.twist.twist.angular.x = pro_imu.angular_velocity.x;

    // East  = speed * sin(track)
    // North = speed * cos(track)
    // twist_msg.twist.twist.linear.x = speed * std::sin(track_rad);
    // twist_msg.twist.twist.linear.y = speed * std::cos(track_rad);
    // twist_msg.twist.twist.linear.z = static_cast<double>(msg->ver_speed);

    // // 3) no angular velocities in BESTVEL
    // twist_msg.twist.twist.angular.x = 0.0;
    // twist_msg.twist.twist.angular.y = 0.0;
    // twist_msg.twist.twist.angular.z = 0.0;

    // 4) fill covariance (here we zero it out; adjust if you have real covariance)
    for (size_t i = 0; i < twist_msg.twist.covariance.size(); ++i) {
      twist_msg.twist.covariance[i] = 0.0;
    }

    // 5) publish
    gnss_vel_pub_->publish(twist_msg);

  }

  void radar_callback(const radar_msgs::msg::RadarTracks::SharedPtr msg) {
    auto out = *msg;

    // Manually convert rclcpp::Time to builtin_interfaces::msg::Time
    rclcpp::Time now = this->get_clock()->now();
    int64_t ns = now.nanoseconds();
    out.header.stamp.sec     = static_cast<int32_t>(ns / 1000000000ULL);
    out.header.stamp.nanosec = static_cast<uint32_t>(ns % 1000000000ULL);

    out.header.frame_id = "radar_1_link";
    radar_pub_->publish(out);
  }

  rclcpp::Subscription<novatel_oem7_msgs::msg::HEADING2>::SharedPtr gnss_heading_;
  // rclcpp::Subscription<novatel_oem7_msgs::msg::INSPVAX>::SharedPtr gnss_inspvax_;
  rclcpp::Subscription<novatel_oem7_msgs::msg::INSPVA>::SharedPtr gnss_inspva_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr localization_odom_;
  rclcpp::Subscription<novatel_oem7_msgs::msg::INSPVA>::SharedPtr gnss_inspva_sideslip_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr        imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr        imu_pro_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr           imu_pub_;
  rclcpp::Publisher<autoware_sensing_msgs::msg::GnssInsOrientationStamped>::SharedPtr gnss_orientation_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr        gnss_coverted_pose;
  rclcpp::Subscription<gps_msgs::msg::GPSFix>::SharedPtr  gnss_sub_;
  // rclcpp::Subscription<novatel_oem7_msgs::msg::BESTGNSSPOS>::SharedPtr  gnss_sub_;
  rclcpp::Subscription<novatel_oem7_msgs::msg::BESTVEL>::SharedPtr gnss_vel_sub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr     gnss_pub_;
  rclcpp::Subscription<radar_msgs::msg::RadarTracks>::SharedPtr radar_sub_;
  rclcpp::Publisher<radar_msgs::msg::RadarTracks>::SharedPtr    radar_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr gnss_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr gnss_compass_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr      beta_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opts;
  opts.use_intra_process_comms(true);
  auto node = std::make_shared<GnssImuQuickConvert>(opts);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

