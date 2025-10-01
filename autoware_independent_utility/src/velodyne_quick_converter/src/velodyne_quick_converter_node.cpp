// src/velodyne_quick_converter_node.cpp

#include <cmath>
#include <cstring>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

using std::placeholders::_1;

class VelodyneQuickConverter : public rclcpp::Node
{
public:
  static constexpr size_t OUT_STEP = 35;

  VelodyneQuickConverter(const rclcpp::NodeOptions & options)
  : Node("velodyne_quick_converter", options)
  {
    // Declare as int to avoid ambiguity, then cast:
    int max_pts_int = this->declare_parameter<int>("max_points", 200000);
    const size_t max_points = static_cast<size_t>(max_pts_int);

    // Reserve once to avoid per-scan reallocations
    out_msg_.data.reserve(max_points * OUT_STEP);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                   .best_effort()
                   .durability_volatile();

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points", qos,
      std::bind(&VelodyneQuickConverter::callback, this, _1));
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/sensing/lidar/top/velodyne_points", qos);

    // Predefine output fields
    out_msg_.fields.clear();
    add_field("x",           0,  sensor_msgs::msg::PointField::FLOAT32, 1);
    add_field("y",           4,  sensor_msgs::msg::PointField::FLOAT32, 1);
    add_field("z",           8,  sensor_msgs::msg::PointField::FLOAT32, 1);
    add_field("intensity",  12,  sensor_msgs::msg::PointField::FLOAT32, 1);
    add_field("ring",       16,  sensor_msgs::msg::PointField::UINT16,  1);
    add_field("azimuth",    18,  sensor_msgs::msg::PointField::FLOAT32, 1);
    add_field("distance",   22,  sensor_msgs::msg::PointField::FLOAT32, 1);
    add_field("return_type",26,  sensor_msgs::msg::PointField::UINT8,   1);
    add_field("time_stamp", 27,  sensor_msgs::msg::PointField::FLOAT64, 1);

    RCLCPP_INFO(this->get_logger(),
      "velodyne_quick_converter started (intra-process %s), max_points=%zu",
      options.use_intra_process_comms() ? "ENABLED" : "DISABLED",
      max_points);
  }

private:
  void add_field(const std::string & name,
                 uint32_t offset,
                 uint8_t datatype,
                 uint32_t count)
  {
    sensor_msgs::msg::PointField f;
    f.name     = name;
    f.offset   = offset;
    f.datatype = datatype;
    f.count    = count;
    out_msg_.fields.push_back(f);
  }

  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr in_msg)
  {
    const auto &in = *in_msg;
    size_t n_pts   = static_cast<size_t>(in.width) * in.height;
    size_t in_step = in.point_step;
    const uint8_t *src = in.data.data();

    // Prepare output
    sensor_msgs::msg::PointCloud2 out;
    out.header          = in.header;
    rclcpp::Time now = this->get_clock()->now();
    int64_t nsec = now.nanoseconds();
    // fill the builtin_interfaces::msg::Time fields
    out.header.stamp.sec     = static_cast<int32_t>(nsec / 1000000000ULL);
    out.header.stamp.nanosec = static_cast<uint32_t>(nsec % 1000000000ULL);
    out.header.frame_id = "velodyne_top";
    out.height          = in.height;
    out.width           = in.width;
    out.is_bigendian    = false;
    out.is_dense        = in.is_dense;
    out.fields          = out_msg_.fields;

    // Resize only (no new allocation)
    out.point_step = OUT_STEP;
    out.row_step   = OUT_STEP * out.width;
    out.data.resize(n_pts * OUT_STEP);

    uint8_t *dst = out.data.data();
    for (size_t i = 0; i < n_pts; ++i) {
      const uint8_t *p = src + i * in_step;
      uint8_t       *q = dst + i * OUT_STEP;

      // unpack
      float x         = *reinterpret_cast<const float*>(p + 0);
      float y         = *reinterpret_cast<const float*>(p + 4);
      float z         = *reinterpret_cast<const float*>(p + 8);
      float intensity = *reinterpret_cast<const float*>(p + 12);
      uint16_t ring   = static_cast<uint16_t>(*(p + 16));
      double timestamp= static_cast<double>(
                          *reinterpret_cast<const float*>(p + 18));

      // compute extras
      float distance   = std::hypot(x, y, z);
      float azimuth    = std::atan2(y, x);
      uint8_t return_type = 1;

      // pack
      std::memcpy(q +  0, &x,           sizeof(float));
      std::memcpy(q +  4, &y,           sizeof(float));
      std::memcpy(q +  8, &z,           sizeof(float));
      std::memcpy(q + 12, &intensity,   sizeof(float));
      std::memcpy(q + 16, &ring,        sizeof(uint16_t));
      std::memcpy(q + 18, &azimuth,     sizeof(float));
      std::memcpy(q + 22, &distance,    sizeof(float));
      std::memcpy(q + 26, &return_type, sizeof(uint8_t));
      std::memcpy(q + 27, &timestamp,   sizeof(double));
    }

    pub_->publish(out);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_;
  sensor_msgs::msg::PointCloud2                                  out_msg_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);

  auto node = std::make_shared<VelodyneQuickConverter>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

