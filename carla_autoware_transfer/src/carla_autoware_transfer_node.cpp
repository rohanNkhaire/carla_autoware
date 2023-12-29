#include "carla_autoware_transfer/carla_autoware_transfer_core.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<CarlaTopicTransfer>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
