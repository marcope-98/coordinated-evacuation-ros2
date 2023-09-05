#ifndef RPL_ROS2_ADAPTERS_POSEADAPTER_HPP_
#define RPL_ROS2_ADAPTERS_POSEADAPTER_HPP_

// rclcpp
#include "rclcpp/rclcpp.hpp"
// rpl
#include "rpl/types.hpp"
// rpl_msgs
#include "rpl_msgs/msg/pose.hpp"

template<>
struct rclcpp::TypeAdapter<rpl::Pose, rpl_msgs::msg::Pose>
{
  using is_specialized   = std::true_type;
  using custom_type      = rpl::Pose;
  using ros_message_type = rpl_msgs::msg::Pose;

  static void convert_to_ros_message(const custom_type &source,
                                     ros_message_type & destination)
  {
    destination = rpl_msgs::build<ros_message_type>()
                      .x(source.x())
                      .y(source.y())
                      .theta(source.theta);
  }

  static void convert_to_custom(const ros_message_type &source,
                                custom_type &           destination)
  {
    destination = custom_type{{source.x, source.y}, source.theta};
  }
};

#endif // RPL_ROS2_ADAPTERS_POSEADAPTER_HPP_