#ifndef RPL_ROS2_ADAPTERS_WORLDDESCRIPTORADAPTER_HPP_
#define RPL_ROS2_ADAPTERS_WORLDDESCRIPTORADAPTER_HPP_

// rclcpp
#include "rclcpp/rclcpp.hpp"
// rpl
#include "rpl/types.hpp"
// rpl_msgs
#include "rpl_msgs/msg/world_descriptor.hpp"
// rpl_ros2
#include "rpl_ros2/adapters/PointAdapter.hpp"
#include "rpl_ros2/adapters/PolygonAdapter.hpp"

template<>
struct rclcpp::TypeAdapter<rpl::WorldDescriptor, rpl_msgs::msg::WorldDescriptor>
{
  using is_specialized   = std::true_type;
  using custom_type      = rpl::WorldDescriptor;
  using ros_message_type = rpl_msgs::msg::WorldDescriptor;
  using PointAdapter     = rclcpp::TypeAdapter<rpl::Point, rpl_msgs::msg::Point>;
  using PolygonAdapter   = rclcpp::TypeAdapter<rpl::Polygon, rpl_msgs::msg::Polygon>;

  static void convert_to_ros_message(const custom_type &source,
                                     ros_message_type & destination)
  {
    destination.obstacles.resize(source.obstacles.size());
    destination.gates.resize(source.obstacles.size());

    for (std::size_t i = 0; i < source.obstacles.size(); ++i)
      PolygonAdapter::convert_to_ros_message(source.obstacles[i], destination.obstacles[i]);

    for (std::size_t i = 0; i < source.gates.size(); ++i)
      PointAdapter::convert_to_ros_message(source.gates[i], destination.gates[i]);

    PolygonAdapter::convert_to_ros_message(source.border, destination.border);
  }

  static void convert_to_custom(const ros_message_type &source,
                                custom_type &           destination)
  {
    destination.obstacles.resize(source.obstacles.size());
    destination.gates.resize(source.obstacles.size());

    for (std::size_t i = 0; i < source.obstacles.size(); ++i)
      PolygonAdapter::convert_to_custom(source.obstacles[i], destination.obstacles[i]);

    for (std::size_t i = 0; i < source.gates.size(); ++i)
      PointAdapter::convert_to_custom(source.gates[i], destination.gates[i]);

    PolygonAdapter::convert_to_custom(source.border, destination.border);
  }
};

#endif // RPL_ROS2_ADAPTERS_WORLDDESCRIPTORADAPTER_HPP_