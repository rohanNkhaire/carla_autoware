#include "autoware_carla_transfer/autoware_carla_transfer_core.hpp"

AutowareCarlaTransfer::AutowareCarlaTransfer()
: Node("autoware_carla_transfer")
{
	// Subscribers
  control_cmd_sub_ = create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
    "control/command/control_cmd", rclcpp::QoS{100},
    std::bind(&AutowareCarlaTransfer::callbackControlCmd, this, std::placeholders::_1));

	// Publishers
	carla_control_pub_ = create_publisher<ackermann_msgs::msg::AckermannDrive>(
    "carla/ego_vehicle/ackermann_cmd", rclcpp::QoS{10});
}

AutowareCarlaTransfer::~AutowareCarlaTransfer()
{
}

void AutowareCarlaTransfer::callbackControlCmd(
	const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr control_msg_ptr_)
{
	autoware_auto_control_msgs::msg::AckermannControlCommand control_msg_ = *control_msg_ptr_;

	ackermann_msgs::msg::AckermannDrive carla_control_cmd_;
	carla_control_cmd_.speed = control_msg_.longitudinal.speed;
	carla_control_cmd_.acceleration = control_msg_.longitudinal.acceleration;
	carla_control_cmd_.jerk = control_msg_.longitudinal.jerk;
	carla_control_cmd_.steering_angle = control_msg_.lateral.steering_tire_angle;
	carla_control_cmd_.steering_angle_velocity = control_msg_.lateral.steering_tire_rotation_rate;

	carla_control_pub_->publish(carla_control_cmd_);
}
