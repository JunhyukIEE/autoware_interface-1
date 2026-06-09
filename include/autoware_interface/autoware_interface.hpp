#ifndef AUTOWARE_INTERFACE_HPP_
#define AUTOWARE_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <chrono>

// ROS Message Headers
#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/control_mode_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "autoware_adapi_v1_msgs/srv/change_operation_mode.hpp"

namespace autoware_interface_ns {

class AutowareInterface : public rclcpp::Node {
public:
    AutowareInterface();

private:
    // Callbacks
    void aw_cmd_callback(const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg);
    void velocity_status_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void steering_status_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void vehicle_mode_status_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void switch_callback(const std_msgs::msg::Bool::SharedPtr msg); // JHH
    void timer_callback();
    void aw_stop_timer_callback();

    // Subscribers
    rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr AW_command_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr angle_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vehicle_mode_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr operation_mode_switch_status_sub_;

    // Publishers
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr AW_velocity_pub_;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr AW_steering_pub_;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr AW_control_mode_pub_;
    rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr AW_control_cmd_pub_;
    rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr AW_control_cmd_sim_pub_; //260523 LJH
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr TC_velocity_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr TC_steer_cmd_pub_;

    rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr AW_stop_client_;
    rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr auto_mode_client_; 

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr aw_stop_timer_;

    // Member variables
    double speed_command_ = 0.0;
    double angle_command_ = 0.0;
    double speed_command_sim_ = 0.0;    //260523 LJH
    double angle_command_sim_ = 0.0;
    double vehicle_speed_ = 0.0;
    double steering_angle_ = 0.0;
    bool vehicle_mode_status_ = 0;
    rclcpp::Clock system_clock_{RCL_SYSTEM_TIME};
};

} // namespace autoware_interface_ns

#endif // AUTOWARE_INTERFACE_HPP_
