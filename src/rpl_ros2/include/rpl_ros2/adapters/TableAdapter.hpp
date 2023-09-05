#ifndef RPL_ROS2_ADAPTERS_TABLEADAPTER_HPP_
#define RPL_ROS2_ADAPTERS_TABLEADAPTER_HPP_
// rclcpp
#include "rclcpp/rclcpp.hpp"
// rpl
#include "rpl/map/Table.hpp"
// rpl_msgs
#include "rpl_msgs/msg/table.hpp"
// rpl_ros2
#include "rpl_ros2/adapters/PointAdapter.hpp"

template<>
struct rclcpp::TypeAdapter<rpl::Table, rpl_msgs::msg::Table>
{
  using is_specialized   = std::true_type;
  using custom_type      = rpl::Table;
  using ros_message_type = rpl_msgs::msg::Table;
  using PointAdapter     = rclcpp::TypeAdapter<rpl::Point, rpl_msgs::msg::Point>;

  static void convert_to_ros_message(const custom_type &source,
                                     ros_message_type & destination)
  {
    destination.points.resize(source.size());
    destination.v_next.reserve(source.size());
    destination.v_prev.reserve(source.size());

    for (std::size_t i = 0; i < source.size(); ++i)
    {
      PointAdapter::convert_to_ros_message(source.point(i), destination.points[i]);
      destination.v_prev.emplace_back(source.prev(i));
      destination.v_next.emplace_back(source.next(i));
    }
  }

  static void convert_to_custom(const ros_message_type &source,
                                custom_type &           destination)
  {
    rpl::Table temp(source.points.size());
    rpl::Point point;
    for (std::size_t i = 0; i < source.points.size(); ++i)
    {
      PointAdapter::convert_to_custom(source.points[i], point);
      temp.emplace_back(point, source.v_prev[i], source.v_next[i]);
    }
    destination = std::move(temp);
  }
};
#endif