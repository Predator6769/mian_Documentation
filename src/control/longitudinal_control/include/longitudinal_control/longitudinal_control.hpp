#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <fstream>
#include <optional>



class LongitudinalController : public rclcpp::Node
{
    public:
        explicit LongitudinalController();

    private:
        autoware_auto_planning_msgs::msg::Trajectory current_trajectory;
        std::optional<nav_msgs::msg::Odometry> current_vehicle_state;
        std::optional<geometry_msgs::msg::AccelWithCovarianceStamped> current_acceleration;

        double look_ahead_distance;

        double timer_duration_msec;

        int control_mode;



        std::optional<double> target_velocity;

        std::optional<double> target_acceleration;

        std::string csv_file_path_;

        std::pair<autoware_auto_planning_msgs::msg::TrajectoryPoint, size_t> findNearestTrajectoryPoint(const autoware_auto_planning_msgs::msg::Trajectory & traj, const nav_msgs::msg::Odometry vehicle_state);


        // retruns start and end index 
        std::pair<size_t, size_t> getIndecesWithinLookAheadDistance(const autoware_auto_planning_msgs::msg::Trajectory & traj, const double lookahead, size_t start_index);


        autoware_auto_planning_msgs::msg::Trajectory InterpolateVelocities(const autoware_auto_planning_msgs::msg::Trajectory & traj, size_t start_index, size_t end_index, const double current_vehicle_velocity, const double lookahead);

        autoware_auto_planning_msgs::msg::Trajectory InterpolateAccelerations(const autoware_auto_planning_msgs::msg::Trajectory & traj, size_t start_index, size_t end_index, const double current_vehicle_accelerations, const double current_vehicle_velocity, const double lookahead);

        autoware_auto_planning_msgs::msg::Trajectory InterpolateAccelerationsAndVelocity(const autoware_auto_planning_msgs::msg::Trajectory & traj_b,size_t start_index,size_t end_index,const double current_vehicle_acceleration,const double current_vehicle_velocity,const double lookahead);


        void trajectory_callback(const autoware_auto_planning_msgs::msg::Trajectory & traj);

        void vehicle_state_callback(const nav_msgs::msg::Odometry & vehicle_s);

        void acceleration_callback(const geometry_msgs::msg::AccelWithCovarianceStamped & tar_acc); 

        void set_target_velocity(const geometry_msgs::msg::Twist::SharedPtr tar_vel); 

        bool read_csv_to_trajectory(const std::string & file_path);

        void publishUpdatedTrajectory();



        rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr traj_subs_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr target_vel_sub_;
        rclcpp::Subscription<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr acceleration_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vehicle_state_sub_;
        rclcpp::Subscription<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr vehicle_acc_sub_;
        rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr updated_traj_publisher_;

        


        rclcpp::TimerBase::SharedPtr publish_timer_;

};