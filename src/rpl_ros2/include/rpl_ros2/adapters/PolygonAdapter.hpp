#ifndef RPL_ROS2_ADAPTERS_POLYGONADAPTER_HPP_
#define RPL_ROS2_ADAPTERS_POLYGONADAPTER_HPP_

// rclcpp
#include "rclcpp/rclcpp.hpp"
// rpl
#include "rpl/types.hpp"
// rpl_msgs
#include "rpl_msgs/msg/polygon.hpp"
// rpl_ros2
#include "rpl_ros2/adapters/PointAdapter.hpp"

template<>
struct rclcpp::TypeAdapter<rpl::Polygon, rpl_msgs::msg::Polygon>
{
  using is_specialized   = std::true_type;
  using custom_type      = rpl::Polygon;
  using ros_message_type = rpl_msgs::msg::Polygon;
  using PointAdapter     = rclcpp::TypeAdapter<rpl::Point, rpl_msgs::msg::Point>;

  static void convert_to_ros_message(const custom_type &source,
                                     ros_message_type & destination)
  {
    destination.vertices.resize(source.size());
    for (std::size_t i = 0; i < source.size(); ++i)
      PointAdapter::convert_to_ros_message(source[i], destination.vertices[i]);
  }

  static void convert_to_custom(const ros_message_type &source,
                                custom_type &           destination)
  {
    destination.resize(source.vertices.size());
    for (std::size_t i = 0; i < source.vertices.size(); ++i)
      PointAdapter::convert_to_custom(source.vertices[i], destination[i]);
  }
};
#endif