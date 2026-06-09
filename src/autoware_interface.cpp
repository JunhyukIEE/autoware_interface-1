#include "autoware_interface/autoware_interface.hpp"

namespace autoware_interface_ns {

AutowareInterface::AutowareInterface()
    : Node("autoware_interface") {
    /* From Autoware */
    /*IRL*/
    AW_command_sub_ = this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
        "/control/command/control_cmd", rclcpp::QoS(1), std::bind(&AutowareInterface::aw_cmd_callback, this, std::placeholders::_1));
        
    /*Simulation*/  //260523 LJH dyno mode
    // AW_command_sub_ = this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
        // "/control/command/control_cmd", rclcpp::QoS(1), std::bind(&AutowareInterface::aw_cmd_callback, this, std::placeholders::_1));

    /* From CANMessageHandler */
    speed_status_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/can_message_receiver/velocity_status", rclcpp::QoS(1), std::bind(&AutowareInterface::velocity_status_callback, this, std::placeholders::_1));
    angle_status_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/can_message_receiver/steering_status", rclcpp::QoS(1), std::bind(&AutowareInterface::steering_status_callback, this, std::placeholders::_1));
    vehicle_mode_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/can_message_receiver/vehicle_mode_status", rclcpp::QoS(1), std::bind(&AutowareInterface::vehicle_mode_status_callback, this, std::placeholders::_1));
    operation_mode_switch_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/can_message_receiver/operation_mode_switch", 10, std::bind(&AutowareInterface::switch_callback, this, std::placeholders::_1));

    /* To Autoware */
    AW_velocity_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>(
        "/vehicle/status/velocity_status", rclcpp::QoS(1));
    AW_steering_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>(
        "/vehicle/status/steering_status", rclcpp::QoS(1));
    AW_control_mode_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>(
        "/vehicle/status/control_mode", rclcpp::QoS(1));
    AW_control_cmd_sim_pub_ = this->create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>( //260523 LJH dyno mode
        "/control/command/control_cmd_sim", rclcpp::QoS(1));    //260523 LJH dyno mode
    clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::QoS(1));
    AW_stop_client_ = this->create_client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>("/api/operation_mode/change_to_stop");
    auto_mode_client_ = this->create_client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>("/api/operation_mode/change_to_autonomous");

    /* To TwistController */
    TC_velocity_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "/twist_controller/input/velocity_cmd", rclcpp::QoS(1));
    TC_steer_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "/twist_controller/input/steer_cmd", rclcpp::QoS(1));

    // Timer
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&AutowareInterface::timer_callback, this));
    aw_stop_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&AutowareInterface::aw_stop_timer_callback, this));
}

void AutowareInterface::aw_cmd_callback(const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg) {
    speed_command_ = msg->longitudinal.speed;
    angle_command_ = msg->lateral.steering_tire_angle;

    std_msgs::msg::Float64 TC_velocity_cmd_msg;
    std_msgs::msg::Float64 TC_steer_cmd_msg;

    TC_velocity_cmd_msg.data = msg->longitudinal.speed;
    TC_steer_cmd_msg.data = msg->lateral.steering_tire_angle;

    TC_velocity_cmd_pub_->publish(TC_velocity_cmd_msg);
    TC_steer_cmd_pub_->publish(TC_steer_cmd_msg);
}

void AutowareInterface::velocity_status_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    vehicle_speed_ = msg->data;
}

void AutowareInterface::steering_status_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    steering_angle_ = msg->data;
}

void AutowareInterface::vehicle_mode_status_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    vehicle_mode_status_ = msg->data;
}

void AutowareInterface::timer_callback() {
    // To Autoware
    const auto now = system_clock_.now();

    rosgraph_msgs::msg::Clock clock_msg;
    clock_msg.clock = now;
    clock_pub_->publish(clock_msg);

    autoware_auto_vehicle_msgs::msg::VelocityReport velocity_report_msg;
    velocity_report_msg.header.stamp = now;
    velocity_report_msg.header.frame_id = "base_link";
    velocity_report_msg.longitudinal_velocity = vehicle_speed_;
    AW_velocity_pub_->publish(velocity_report_msg);

    autoware_auto_vehicle_msgs::msg::SteeringReport steering_report_msg;
    steering_report_msg.stamp = now;
    steering_report_msg.steering_tire_angle = steering_angle_;
    AW_steering_pub_->publish(steering_report_msg);

    autoware_auto_vehicle_msgs::msg::ControlModeReport control_mode_msg;
    control_mode_msg.stamp = now;
    // Mode 1 corresponds to AUTONOMOUS
    control_mode_msg.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
    AW_control_mode_pub_->publish(control_mode_msg);

    autoware_auto_control_msgs::msg::AckermannControlCommand control_cmd_msg;   //260523 LJH dyno mode
    /*IRL*/
    // control_cmd_msg.longitudinal.speed = speed_command_;
    // control_cmd_msg.lateral.steering_tire_angle = angle_command_;

    // AW_control_cmd_pub_->publish(control_cmd_msg);

    /*IRL*/

    /*260526LJH dyno mode*/
    control_cmd_msg.stamp = now;
    control_cmd_msg.longitudinal.stamp = now;
    control_cmd_msg.lateral.stamp = now;
    control_cmd_msg.longitudinal.speed = vehicle_speed_;
    control_cmd_msg.lateral.steering_tire_angle = steering_angle_;
    
    AW_control_cmd_sim_pub_->publish(control_cmd_msg);
    /*260526LJH dyno mode*/


    // To TwistController
    // std_msgs::msg::Float64 speed_cmd_msg;
    // speed_cmd_msg.data = speed_command_;
    // TC_velocity_cmd_pub_->publish(speed_cmd_msg);

    // std_msgs::msg::Float64 steer_cmd_msg;
    // steer_cmd_msg.data = angle_command_;
    // TC_steer_cmd_pub_->publish(steer_cmd_msg);
}

void AutowareInterface::aw_stop_timer_callback()
{
    if(vehicle_mode_status_ == 0)
    {
        std::shared_ptr<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request> request = 
            std::make_shared<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request>();
        std::shared_future<std::shared_ptr<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Response>> result = 
            AW_stop_client_->async_send_request(request);
    }
}

void AutowareInterface::switch_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if(msg->data && !vehicle_mode_status_ == 0)
    {
        std::shared_ptr<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request>request = 
            std::make_shared<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request>();
        std::shared_future<std::shared_ptr<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Response>>result = 
            auto_mode_client_->async_send_request(request);
    }
}
} // namespace autoware_interface_ns

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<autoware_interface_ns::AutowareInterface>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
