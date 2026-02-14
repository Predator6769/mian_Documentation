#include <rclcpp/rclcpp.hpp>
#include "longitudinal_control/longitudinal_control.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LongitudinalController>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
