#ifndef RPL_ROS2_ADAPTERS_GRAPHADAPTER_HPP_
#define RPL_ROS2_ADAPTERS_GRAPHADAPTER_HPP_

// rclcpp
#include "rclcpp/rclcpp.hpp"
// rpl
#include "rpl/map/Graph.hpp"
// rpl_msgs
#include "rpl_msgs/msg/graph.hpp"

template<>
struct rclcpp::TypeAdapter<rpl::Graph, rpl_msgs::msg::Graph>
{
  using is_specialized   = std::true_type;
  using custom_type      = rpl::Graph;
  using ros_message_type = rpl_msgs::msg::Graph;

  static void convert_to_ros_message(const custom_type &source,
                                     ros_message_type & destination)
  {
    destination.nodes = (std::uint64_t)source.height();
    destination.data.reserve(source.capacity());
    for (std::size_t i = 0; i < source.size(); i += 16)
    {
      destination.data.emplace_back(source[i + 0]);
      destination.data.emplace_back(source[i + 1]);
      destination.data.emplace_back(source[i + 2]);
      destination.data.emplace_back(source[i + 3]);
      destination.data.emplace_back(source[i + 4]);
      destination.data.emplace_back(source[i + 5]);
      destination.data.emplace_back(source[i + 6]);
      destination.data.emplace_back(source[i + 7]);
      destination.data.emplace_back(source[i + 8]);
      destination.data.emplace_back(source[i + 9]);
      destination.data.emplace_back(source[i + 10]);
      destination.data.emplace_back(source[i + 11]);
      destination.data.emplace_back(source[i + 12]);
      destination.data.emplace_back(source[i + 13]);
      destination.data.emplace_back(source[i + 14]);
      destination.data.emplace_back(source[i + 15]);
    }
  }

  static void convert_to_custom(const ros_message_type &source,
                                custom_type &           destination)
  {
    rpl::Graph temp((std::size_t)source.nodes);
    for (std::size_t i = 0; i < temp.size(); i += 16)
    {
      temp[i + 0]  = source.data[i + 0];
      temp[i + 1]  = source.data[i + 1];
      temp[i + 2]  = source.data[i + 2];
      temp[i + 3]  = source.data[i + 3];
      temp[i + 4]  = source.data[i + 4];
      temp[i + 5]  = source.data[i + 5];
      temp[i + 6]  = source.data[i + 6];
      temp[i + 7]  = source.data[i + 7];
      temp[i + 8]  = source.data[i + 8];
      temp[i + 9]  = source.data[i + 9];
      temp[i + 10] = source.data[i + 10];
      temp[i + 11] = source.data[i + 11];
      temp[i + 12] = source.data[i + 12];
      temp[i + 13] = source.data[i + 13];
      temp[i + 14] = source.data[i + 14];
      temp[i + 15] = source.data[i + 15];
    }
    destination = std::move(temp);
  }
};
#endif