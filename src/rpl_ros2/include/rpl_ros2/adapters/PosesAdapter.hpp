#ifndef RPL_ROS2_ADAPTERS_POSESADAPTER_HPP_
#define RPL_ROS2_ADAPTERS_POSESADAPTER_HPP_

// rclcpp
#include "rclcpp/rclcpp.hpp"
// rpl
#include "rpl/types.hpp"
// rpl_msgs
#include "rpl_msgs/msg/poses.hpp"
// rpl_ros2
#include "rpl_ros2/adapters/PoseAdapter.hpp"

template<>
struct rclcpp::TypeAdapter<rpl::Poses, rpl_msgs::msg::Poses>
{
  using is_specialized   = std::true_type;
  using custom_type      = rpl::Poses;
  using ros_message_type = rpl_msgs::msg::Poses;
  using PoseAdapter      = rclcpp::TypeAdapter<rpl::Pose, rpl_msgs::msg::Pose>;

  static void convert_to_ros_message(const custom_type &source,
                                     ros_message_type & destination)
  {
    destination.poses.resize(source.size());
    for (std::size_t i = 0; i < source.size(); ++i)
      PoseAdapter::convert_to_ros_message(source[i], destination.poses[i]);
  }

  static void convert_to_custom(const ros_message_type &source,
                                custom_type &           destination)
  {
    destination.resize(source.poses.size());
    for (std::size_t i = 0; i < source.poses.size(); ++i)
      PoseAdapter::convert_to_custom(source.poses[i], destination[i]);
  }
};
#endif