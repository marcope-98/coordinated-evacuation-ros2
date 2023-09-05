#ifndef RPL_ROS2_ADAPTERS_ROADMAPADAPTER_HPP_
#define RPL_ROS2_ADAPTERS_ROADMAPADAPTER_HPP_

// rclcpp
#include "rclcpp/rclcpp.hpp"
// rpl
#include "rpl/map/RoadMap.hpp"
// rpl_ros2
#include "rpl_ros2/adapters/GraphAdapter.hpp"
#include "rpl_ros2/adapters/TableAdapter.hpp"
// rpl_msgs
#include "rpl_msgs/msg/road_map.hpp"

template<>
struct rclcpp::TypeAdapter<rpl::RoadMap, rpl_msgs::msg::RoadMap>
{
  using is_specialized   = std::true_type;
  using custom_type      = rpl::RoadMap;
  using ros_message_type = rpl_msgs::msg::RoadMap;
  using GraphAdapter     = rclcpp::TypeAdapter<rpl::Graph, rpl_msgs::msg::Graph>;
  using TableAdapter     = rclcpp::TypeAdapter<rpl::Table, rpl_msgs::msg::Table>;

  static void convert_to_ros_message(const custom_type &source,
                                     ros_message_type & destination)
  {
    GraphAdapter::convert_to_ros_message(source.graph(), destination.graph);
    TableAdapter::convert_to_ros_message(source.table(), destination.table);
  }

  static void convert_to_custom(const ros_message_type &source,
                                custom_type &           destination)
  {
    GraphAdapter::convert_to_custom(source.graph, destination.graph());
    TableAdapter::convert_to_custom(source.table, destination.table());
  }
};
#endif