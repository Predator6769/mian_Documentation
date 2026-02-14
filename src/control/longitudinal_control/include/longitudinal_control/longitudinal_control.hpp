#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <fstream>
#include <optional>



class LongitudinalController : public rclcpp::Node
{
    public:
        explicit LongitudinalController();

    private:
        autoware_auto_planning_msgs::msg::Trajectory current_trajectory;
        std::optional<nav_msgs::msg::Odometry> current_vehicle_state;

        double look_ahead_distance = 5.0;

        double timer_duration_msec = 10;

        std::optional<double> target_velocity;

        std::string csv_file_path_;

        std::pair<autoware_auto_planning_msgs::msg::TrajectoryPoint, size_t> findNearestTrajectoryPoint(const autoware_auto_planning_msgs::msg::Trajectory & traj, const nav_msgs::msg::Odometry vehicle_state);


        // retruns start and end index 
        std::pair<size_t, size_t> getIndecesWithinLookAheadDistance(const autoware_auto_planning_msgs::msg::Trajectory & traj, const double lookahead, size_t start_index);


        autoware_auto_planning_msgs::msg::Trajectory InterpolateVelocities(const autoware_auto_planning_msgs::msg::Trajectory & traj, size_t start_index, size_t end_index, const double current_vehicle_velocity, const double lookahead);


        void trajectory_callback(const autoware_auto_planning_msgs::msg::Trajectory & traj);

        void vehicle_state_callback(const nav_msgs::msg::Odometry & vehicle_s);

        
        void set_target_velocity(const std_msgs::msg::Float64::SharedPtr tar_vel); 

        bool read_csv_to_trajectory(const std::string & file_path);

        void publishUpdatedTrajectory();



        rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr traj_subs_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_vel_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vehicle_state_sub_;
        rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr updated_traj_publisher_;


        rclcpp::TimerBase::SharedPtr publish_timer_;

};