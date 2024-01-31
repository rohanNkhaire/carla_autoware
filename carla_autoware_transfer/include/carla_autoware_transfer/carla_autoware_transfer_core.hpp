#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <deque>
#include <memory>
#include <string>

// Importing subscriber msgs
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "derived_object_msgs/msg/object_array.hpp"
#include "carla_msgs/msg/carla_ego_vehicle_status.hpp"

// Importing publisher msgs
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_auto_perception_msgs/msg/detected_object.hpp"
#include "autoware_auto_perception_msgs/msg/detected_objects.hpp"
#include "autoware_auto_perception_msgs/msg/shape.hpp"
#include "autoware_auto_perception_msgs/msg/detected_object_kinematics.hpp"
#include "autoware_auto_perception_msgs/msg/object_classification.hpp"

// TF2
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"


class CarlaTopicTransfer : public rclcpp::Node
{
public:
  CarlaTopicTransfer();
  ~CarlaTopicTransfer();

private:
  void callbackOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr odometry_msg_ptr);
  void callbackCarlaObjects(const derived_object_msgs::msg::ObjectArray::ConstSharedPtr carla_obstacle_msg_ptr);
  autoware_auto_vehicle_msgs::msg::VelocityReport VelocityReportMsg(
  const std::deque<nav_msgs::msg::Odometry> & vehicle_odom_queue_);
  autoware_auto_perception_msgs::msg::DetectedObjects AutowareObstacleInfo(
  const std::deque<derived_object_msgs::msg::ObjectArray>& carla_objects_queue_);

  void setupTF();

  void publishGroundTruthLocalization(const std::deque<nav_msgs::msg::Odometry>& vehicle_pose_);

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

// Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    vehicle_odom_sub_;
  rclcpp::Subscription<derived_object_msgs::msg::ObjectArray>::SharedPtr
    carla_objects_sub_;    


// Publishers
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr carla_objects_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr carla_nav_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr carla_twist_pub_;

  bool groundtruth_localization_;

  std::deque<nav_msgs::msg::Odometry> vehicle_odom_queue_;
  std::deque<sensor_msgs::msg::Imu> gyro_queue_;
  std::deque<derived_object_msgs::msg::ObjectArray> carla_objects_queue_;
};
