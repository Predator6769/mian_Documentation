#include <rclcpp/rclcpp.hpp>
#include <radar_msgs/msg/radar_tracks.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <cmath>
#include <limits>

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

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/radar_1/front_track_marker", 10
    );

     closest_point_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseArray>(
        "/radar_1/front_track_data", 10);

    // Tunable parameters
    max_angle_rad_ = this->declare_parameter("max_angle_deg", 5) * M_PI / 180.0;
    y_base = this->declare_parameter("vehicle_width_half", 0.9);
    RCLCPP_INFO(this->get_logger(), "Front Object Detector initialized");
  }

  float lateral_distance = 0.3;

private:
  void tracksCallback(const radar_msgs::msg::RadarTracks::SharedPtr msg)
  {
    visualization_msgs::msg::Marker points;
    points.header.frame_id = "radar_1";
    points.header.stamp = msg->header.stamp;
    points.ns = "radar_points";
    points.id = 0;
    points.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    points.action = visualization_msgs::msg::Marker::ADD;

    // Pink color
    points.color.r = 1.0;
    points.color.g = 0.4;
    points.color.b = 0.7;
    points.color.a = 1.0;

    points.scale.x = 1.0;
    points.scale.y = 1.0;
    points.scale.z = 1.0;

    // Lifetime → auto-vanish
    points.lifetime = rclcpp::Duration::from_seconds(0.02);

    int id = 0;

    geometry_msgs::msg::PoseArray data;
    data.header = msg->header;

    const double y_base = 0.9;

    for (const auto & track : msg->tracks)
    {
      const double x = track.position.x;  // forward
      const double y = track.position.y;  // lateral

      double distance = std::sqrt(x * x + y * y);

      const double y_max = std::max(y_base, (x*std::tan(max_angle_rad_)));
      // Must be in front
      if (x <= -0.3 || x>=50) {
        continue;
      }

      if(std::abs(y) > y_max)
      continue;



    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = track.position.z;

    geometry_msgs::msg::Pose poi;
    poi.position.x = x;
    poi.position.y = y;
    poi.position.z = distance;

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

    // White text
    text.color.r = 1.0;
    text.color.g = 1.0;
    text.color.b = 1.0;
    text.color.a = 1.0;

    text.text = "Dist: " + std::to_string(distance);

    // Auto-expire
    text.lifetime = rclcpp::Duration::from_seconds(0.02);
    points.points.push_back(p);

    marker_pub_->publish(text);
    marker_pub_->publish(points);
    }

    closest_point_pub_->publish(data);
  }

  rclcpp::Subscription<radar_msgs::msg::RadarTracks>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr closest_point_pub_;
  double max_angle_rad_;
  double y_base;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontObjectDetector>());
  rclcpp::shutdown();
  return 0;
}
