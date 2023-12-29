#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

// Subscribers
#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"

// Publishers
#include "ackermann_msgs/msg/ackermann_drive.hpp"


class AutowareCarlaTransfer : public rclcpp::Node
{
	public:
		AutowareCarlaTransfer();
		~AutowareCarlaTransfer();

	private:
		void callbackControlCmd(const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr control_cmd);

		// Subscriber
		rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr control_cmd_sub_;

		// Publisher
		rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr carla_control_pub_;

};