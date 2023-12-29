#include "autoware_carla_transfer/autoware_carla_transfer_core.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<AutowareCarlaTransfer>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}