#include "carla_autoware_transfer/carla_autoware_transfer_core.hpp"

#include <cmath>
#include <memory>
#include <string>


autoware_auto_vehicle_msgs::msg::VelocityReport CarlaTopicTransfer::VelocityReportMsg(
  const std::deque<nav_msgs::msg::Odometry> & vehicle_odom_queue_)
{

  autoware_auto_vehicle_msgs::msg::VelocityReport velocity_report_raw;
  const auto latest_vehicle_stamp = rclcpp::Time(vehicle_odom_queue_.back().header.stamp);
  velocity_report_raw.header.stamp = latest_vehicle_stamp;
  velocity_report_raw.header.frame_id = "base_link";
  velocity_report_raw.longitudinal_velocity = vehicle_odom_queue_.back().twist.twist.linear.x;
  velocity_report_raw.lateral_velocity = vehicle_odom_queue_.back().twist.twist.linear.y;
  velocity_report_raw.heading_rate = vehicle_odom_queue_.back().twist.twist.angular.z;

  return velocity_report_raw;
}

void CarlaTopicTransfer::publishGroundTruthLocalization(const std::deque<nav_msgs::msg::Odometry> & vehicle_odom_queue_)
{
  // Odometry
  nav_msgs::msg::Odometry carla_pose_ = vehicle_odom_queue_.back();
  carla_pose_.child_frame_id = "base_link";

  // Vehicle twist
  geometry_msgs::msg::TwistWithCovarianceStamped carla_twist_;
  carla_twist_.header = vehicle_odom_queue_.back().header;
  carla_twist_.header.frame_id = "base_link";
  carla_twist_.twist = vehicle_odom_queue_.back().twist;

  // use tf broadcaster to transform pose to base_link
  geometry_msgs::msg::TransformStamped transformStamped; 
  transformStamped.header.stamp = rclcpp::Time(vehicle_odom_queue_.back().header.stamp);
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = "base_link";
  transformStamped.transform.translation.x = vehicle_odom_queue_.back().pose.pose.position.x;
  transformStamped.transform.translation.y = vehicle_odom_queue_.back().pose.pose.position.y;
  transformStamped.transform.translation.z = 0.0;
  transformStamped.transform.rotation.x = 0.0;
  transformStamped.transform.rotation.y = 0.0;
  transformStamped.transform.rotation.z = vehicle_odom_queue_.back().pose.pose.orientation.z;
  transformStamped.transform.rotation.w = vehicle_odom_queue_.back().pose.pose.orientation.w;

  tf_broadcaster_->sendTransform(transformStamped);
        
  // publish topics
  carla_nav_pub_->publish(carla_pose_);
  carla_twist_pub_->publish(carla_twist_);
}

autoware_auto_perception_msgs::msg::DetectedObjects CarlaTopicTransfer::AutowareObstacleInfo(
  const std::deque<derived_object_msgs::msg::ObjectArray>& carla_objects_queue_)
{
  autoware_auto_perception_msgs::msg::DetectedObjects detected_objects_;

  // populating header field
  const auto latest_carla_object_stamp = rclcpp::Time(carla_objects_queue_.back().header.stamp);
  detected_objects_.header.stamp = latest_carla_object_stamp;
  detected_objects_.header.frame_id = "map";

  if (carla_objects_queue_.back().objects.empty())
  {
    return detected_objects_;
  }

  for (const auto & carla_object : carla_objects_queue_.back().objects)
  {
    autoware_auto_perception_msgs::msg::DetectedObject detected_obj_;
    autoware_auto_perception_msgs::msg::Shape shape_;
    autoware_auto_perception_msgs::msg::DetectedObjectKinematics detected_kin_obj_;
    autoware_auto_perception_msgs::msg::ObjectClassification obj_classification_;
    geometry_msgs::msg::PoseStamped pose_in_;
    geometry_msgs::msg::PoseStamped pose_out_;

    // Shape
    shape_.type = 0;
    shape_.dimensions.x = carla_object.shape.dimensions[0];
    shape_.dimensions.y = carla_object.shape.dimensions[1];
    shape_.dimensions.z = carla_object.shape.dimensions[2];

    // DetectedObjectKinematic
    detected_kin_obj_.has_position_covariance = false;
    detected_kin_obj_.has_twist_covariance = false;
    detected_kin_obj_.has_twist = true;
    detected_kin_obj_.orientation_availability = 0;

    detected_kin_obj_.pose_with_covariance.pose = carla_object.pose;
    detected_kin_obj_.twist_with_covariance.twist = carla_object.twist;

    // ObjectClassification
    switch(carla_object.classification)
    {
      case 0:
        obj_classification_.label = 0;
        break;
      case 4:
        obj_classification_.label = 7;
        break;
      case 6:
        obj_classification_.label = 1;
        break;
      case 5:
        obj_classification_.label = 6;
        break;
      case 8:
        obj_classification_.label = 5;
        break;
      case 7:
        obj_classification_.label = 2;
        break;
      default:
        obj_classification_.label = 0;
        break;          
    }
    obj_classification_.probability = 1.0;

    // Populating DetectedObject
    detected_obj_.existence_probability = 1.0;
    detected_obj_.shape = shape_;
    detected_obj_.kinematics = detected_kin_obj_;
    detected_obj_.classification.emplace_back(obj_classification_);

    detected_objects_.objects.emplace_back(detected_obj_);
  }

  return detected_objects_;
}  

void CarlaTopicTransfer::setupTF()
{
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

}

CarlaTopicTransfer::CarlaTopicTransfer()
: Node("carla_autoware_transfer"),
  groundtruth_localization_(declare_parameter<bool>("groundtruth_localization"))
{
  // Subscribers
  vehicle_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "carla/ego_vehicle/odometry", rclcpp::QoS{100},
    std::bind(&CarlaTopicTransfer::callbackOdometry, this, std::placeholders::_1));

  carla_objects_sub_ = create_subscription<derived_object_msgs::msg::ObjectArray>(
    "carla/ego_vehicle/objects", rclcpp::QoS{100},
    std::bind(&CarlaTopicTransfer::callbackCarlaObjects, this, std::placeholders::_1));          

  // Publishers

  carla_objects_pub_ = create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>(
    "perception/object_recognition/detection/carla/autoware_objects", rclcpp::QoS{10});

  carla_nav_pub_ = create_publisher<nav_msgs::msg::Odometry>(
    "localization/kinematic_state", rclcpp::QoS{10});

  carla_twist_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "localization/twist_estimator/twist_with_covariance", rclcpp::QoS{10});    

    setupTF();

}

CarlaTopicTransfer::~CarlaTopicTransfer()
{
}

void CarlaTopicTransfer::callbackOdometry(
  const nav_msgs::msg::Odometry::ConstSharedPtr odometry_msg_ptr)
{
  vehicle_odom_queue_.push_back(*odometry_msg_ptr);

  const autoware_auto_vehicle_msgs::msg::VelocityReport velocity_report_raw =
    VelocityReportMsg(vehicle_odom_queue_);

  if (groundtruth_localization_)
  {
    publishGroundTruthLocalization(vehicle_odom_queue_);  
  }

  vehicle_odom_queue_.clear();
}

void CarlaTopicTransfer::callbackCarlaObjects(
  const derived_object_msgs::msg::ObjectArray::ConstSharedPtr carla_obstacle_msg_ptr)
{
  carla_objects_queue_.push_back(*carla_obstacle_msg_ptr);

  const autoware_auto_perception_msgs::msg::DetectedObjects carla_objects_raw =
    AutowareObstacleInfo(carla_objects_queue_);

  // publishing data  
  autoware_auto_perception_msgs::msg::DetectedObjects carla_objects_;
  carla_objects_.header = carla_objects_raw.header;
  carla_objects_ = carla_objects_raw;
  carla_objects_pub_->publish(carla_objects_);

  carla_objects_queue_.clear();
}