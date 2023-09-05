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
    destination.obstacles_inner.resize(source.obstacles_inner.size());
    destination.obstacles_outer.resize(source.obstacles_outer.size());
    destination.gates.resize(source.gates.size());

    for (std::size_t i = 0; i < source.obstacles_inner.size(); ++i)
      PolygonAdapter::convert_to_ros_message(source.obstacles_inner[i], destination.obstacles_inner[i]);

    for (std::size_t i = 0; i < source.obstacles_outer.size(); ++i)
      PolygonAdapter::convert_to_ros_message(source.obstacles_outer[i], destination.obstacles_outer[i]);

    for (std::size_t i = 0; i < source.gates.size(); ++i)
      PointAdapter::convert_to_ros_message(source.gates[i], destination.gates[i]);

    PolygonAdapter::convert_to_ros_message(source.border_inner, destination.border_inner);
    PolygonAdapter::convert_to_ros_message(source.border_outer, destination.border_outer);
  }

  static void convert_to_custom(const ros_message_type &source,
                                custom_type &           destination)
  {
    destination.obstacles_inner.resize(source.obstacles_inner.size());
    destination.obstacles_outer.resize(source.obstacles_outer.size());
    destination.gates.resize(source.gates.size());

    for (std::size_t i = 0; i < source.obstacles_inner.size(); ++i)
      PolygonAdapter::convert_to_custom(source.obstacles_inner[i], destination.obstacles_inner[i]);

    for (std::size_t i = 0; i < source.obstacles_outer.size(); ++i)
      PolygonAdapter::convert_to_custom(source.obstacles_outer[i], destination.obstacles_outer[i]);

    for (std::size_t i = 0; i < source.gates.size(); ++i)
      PointAdapter::convert_to_custom(source.gates[i], destination.gates[i]);

    PolygonAdapter::convert_to_custom(source.border_inner, destination.border_inner);
    PolygonAdapter::convert_to_custom(source.border_outer, destination.border_outer);
  }
};

#endif // RPL_ROS2_ADAPTERS_WORLDDESCRIPTORADAPTER_HPP_