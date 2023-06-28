#include "simulator/send_obstacles.hpp"
// stl
#include <chrono>
#include <cstdint>
#include <functional>
#include <string>
#include <unistd.h>
#include <vector>
// messages
#include "geometry_msgs/msg/point32.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"
#include "std_msgs/msg/header.hpp"
// string utils
#include "utils.hpp"

using std::placeholders::_1;

ObstaclePublisher::ObstaclePublisher() : Node("obstacle_publisher")
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
  pub_     = this->create_publisher<obstacles_msgs::msg::ObstacleArrayMsg>(
      "obstacles", qos);
  sub_ = this->create_subscription<gazebo_msgs::msg::LinkStates>(
      "link_states", 10, std::bind(&ObstaclePublisher::obstacles_cb, this, _1));
}

void ObstaclePublisher::obstacles_cb(const gazebo_msgs::msg::LinkStates::SharedPtr msg) const
{
  obstacles_msgs::msg::ObstacleMsg      obstacle_msg;
  obstacles_msgs::msg::ObstacleArrayMsg out;

  std::vector<geometry_msgs::msg::Point32>      points;
  std::vector<obstacles_msgs::msg::ObstacleMsg> obstacles;

  out.header = std_msgs::build<std_msgs::msg::Header>()
                   .stamp(this->now())
                   .frame_id("map");

  std::vector<std::string> link_state;
  for (std::size_t i = 0; i < msg->name.size(); ++i)
  {
    link_state = utils::split_by_delimiter(msg->name[i], "::");
    if (!(utils::contains(link_state[0], "mindstorm_map") &&
          utils::contains(link_state[1], "obstacle"))) continue;

    if (utils::contains(link_state[2], "link"))
    {
      obstacle_msg.polygon.points = points;
      obstacles.emplace_back(obstacle_msg);
      points.clear();
    }
    else
      points.emplace_back(geometry_msgs::build<geometry_msgs::msg::Point32>()
                              .x(float(msg->pose[i].position.x))
                              .y(float(msg->pose[i].position.y))
                              .z(0.f));
  }
  obstacle_msg.polygon.points = points;
  obstacles.emplace_back(obstacle_msg);
  points.clear();

  out.obstacles = obstacles;

  for (;;)
  {
    this->pub_->publish(out);
    usleep(1000000);
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstaclePublisher>());
  rclcpp::shutdown();
  return 0;
}