#include "rpl_ros2/world_descriptor.hpp"

// stl
#include <functional>
#include <memory>
// rpl
#include "rpl/types.hpp"
// rpl_msgs
#include "rpl_msgs/msg/point.hpp"
#include "rpl_msgs/msg/pose.hpp"

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

  pub_ = this->create_publisher<rpl::WorldDescriptor>(
      "world_description", qos);

  this->thread_ = std::thread(std::bind(&WorldDescriptorNode::wd_publisher, this));
}

rpl_ros2::WorldDescriptorNode::~WorldDescriptorNode()
{
  if (this->thread_.joinable()) this->thread_.join();
}

void rpl_ros2::WorldDescriptorNode::gate_cb(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  auto is_bit_set = bool(this->called & 0x01);
  if (is_bit_set) return;
  this->called |= 0x01; // 0000 0001

  // process message
  this->wd.gates.reserve(msg->poses.size());
  for (const auto &pose : msg->poses)
    this->wd.gates.emplace_back(pose.position.x, pose.position.y);
}

void rpl_ros2::WorldDescriptorNode::border_cb(const geometry_msgs::msg::Polygon::SharedPtr msg)
{
  auto is_bit_set = bool(this->called & 0x02);
  if (is_bit_set) return;
  this->called |= 0x02; // 0000 0010

  // process message
  rpl::Polygon border;
  border.reserve(msg->points.size());
  for (const auto &point : msg->points)
    border.emplace_back(point.x, point.y);

  this->wd.process_border(border);
}

void rpl_ros2::WorldDescriptorNode::obstacles_cb(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
{
  auto is_bit_set = bool(this->called & 0x04);
  if (is_bit_set) return;
  this->called |= 0x04; // 0000 0100

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

void rpl_ros2::WorldDescriptorNode::wd_publisher()
{
  while (rclcpp::ok())
  {
    if (!(this->called == 0x07)) continue; // if this->called = 0000 0111
    this->called = 0x0F;                   // 0000 1111

    this->pub_->publish(this->wd);
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rpl_ros2::WorldDescriptorNode>());
  rclcpp::shutdown();
  return 0;
}