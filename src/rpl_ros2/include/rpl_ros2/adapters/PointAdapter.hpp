#ifndef RPL_ROS2_ADAPTERS_POINTADAPTER_HPP_
#define RPL_ROS2_ADAPTERS_POINTADAPTER_HPP_

// rclcpp
#include "rclcpp/rclcpp.hpp"
// rpl
#include "rpl/types.hpp"
// rpl_msgs
#include "rpl_msgs/msg/point.hpp"

template<>
struct rclcpp::TypeAdapter<rpl::Point, rpl_msgs::msg::Point>
{
  using is_specialized   = std::true_type;
  using custom_type      = rpl::Point;
  using ros_message_type = rpl_msgs::msg::Point;

  static void convert_to_ros_message(const custom_type &source,
                                     ros_message_type & destination)
  {
    destination = rpl_msgs::build<ros_message_type>()
                      .x(source.x)
                      .y(source.y);
  }

  static void convert_to_custom(const ros_message_type &source,
                                custom_type &           destination)
  {
    destination = custom_type(source.x, source.y);
  }
};

#endif // RPL_ROS2_ADAPTERS_POINTADAPTER_HPP_