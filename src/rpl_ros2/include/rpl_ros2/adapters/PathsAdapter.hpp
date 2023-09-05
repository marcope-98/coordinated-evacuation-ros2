#ifndef RPL_ROS2_ADAPTERS_PATHSADAPTER_HPP_
#define RPL_ROS2_ADAPTERS_PATHSADAPTER_HPP_

// rclcpp
#include "rclcpp/rclcpp.hpp"
// rpl
#include "rpl/types.hpp"
// rpl_msgs
#include "rpl_msgs/msg/paths.hpp"
// rpl_ros2
#include "rpl_ros2/adapters/PathAdapter.hpp"

template<>
struct rclcpp::TypeAdapter<rpl::Paths, rpl_msgs::msg::Paths>
{
  using is_specialized   = std::true_type;
  using custom_type      = rpl::Paths;
  using ros_message_type = rpl_msgs::msg::Paths;
  using PathAdapter      = rclcpp::TypeAdapter<rpl::Path, rpl_msgs::msg::Path>;

  static void convert_to_ros_message(const custom_type &source,
                                     ros_message_type & destination)
  {
    destination.paths.resize(source.size());
    for (std::size_t i = 0; i < source.size(); ++i)
      PathAdapter::convert_to_ros_message(source[i], destination.paths[i]);
  }

  static void convert_to_custom(const ros_message_type &source,
                                custom_type &           destination)
  {
    destination.resize(source.paths.size());
    for (std::size_t i = 0; i < source.paths.size(); ++i)
      PathAdapter::convert_to_custom(source.paths[i], destination[i]);
  }
};

#endif // RPL_ROS2_ADAPTERS_PATHADAPTER_HPP_