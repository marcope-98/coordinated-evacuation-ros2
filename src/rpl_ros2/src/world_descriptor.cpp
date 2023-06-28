#include "rpl_ros2/world_descriptor.hpp"

#include <functional>
#include <iostream>
#include <memory>

#include "rpl/types.hpp"

using std::placeholders::_1;

rpl_ros2::WorldDescriptorNode::WorldDescriptorNode() : rclcpp::Node("world_descriptor"),
                                                       wd()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
  sub_gate = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "gate_position", qos, std::bind(&WorldDescriptorNode::gate_cb, this, _1));
  sub_borders = this->create_subscription<geometry_msgs::msg::Polygon>(
      "map_borders", qos, std::bind(&WorldDescriptorNode::border_cb, this, _1));
  sub_obstacles = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
      "obstacles", qos, std::bind(&WorldDescriptorNode::obstacles_cb, this, _1));
}

void rpl_ros2::WorldDescriptorNode::gate_cb(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  bool is_bit_set = (((this->called >> 0u) & 0x01) == 0x01);
  if (is_bit_set) return;
  this->called |= (0x01 << 0u);

  // process message
  this->wd.gates.reserve(msg->poses.size());
  for (const auto &pose : msg->poses)
    this->wd.gates.emplace_back(pose.position.x, pose.position.y);
}

void rpl_ros2::WorldDescriptorNode::border_cb(const geometry_msgs::msg::Polygon::SharedPtr msg)
{
  bool is_bit_set = (((this->called >> 1u) & 0x01) == 0x01);
  if (is_bit_set) return;
  this->called |= (0x01 << 1u);

  // process message
  rpl::Polygon border;
  border.reserve(msg->points.size());
  for (const auto &point : msg->points)
    border.emplace_back(point.x, point.y);

  this->wd.process_border(border);
}

void rpl_ros2::WorldDescriptorNode::obstacles_cb(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
{
  bool is_bit_set = (((this->called >> 2u) & 0x01) == 0x01);
  if (is_bit_set) return;
  this->called |= (0x01 << 2u);

  // process message
  std::vector<rpl::Polygon> obstacles;
  obstacles.reserve(msg->obstacles.size());

  rpl::Polygon poly;
  for (const auto &obstacle : msg->obstacles)
  {
    poly.clear();
    poly.reserve(obstacles.size());
    for (const auto &point : obstacle.polygon.points)
      poly.emplace_back(point.x, point.y);
    obstacles.emplace_back(poly);
  }
  this->wd.process_obstacles(obstacles);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rpl_ros2::WorldDescriptorNode>());
  rclcpp::shutdown();
  return 0;
}