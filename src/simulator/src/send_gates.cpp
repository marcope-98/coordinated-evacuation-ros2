#include "simulator/send_gates.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <unistd.h>

// messages
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/header.hpp"
// string utils
#include "utils.hpp"

using std::placeholders::_1;

GatesPublisher::GatesPublisher() : Node("send_gates")
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
  sub_     = this->create_subscription<gazebo_msgs::msg::LinkStates>(
      "link_states", 10, std::bind(&GatesPublisher::gate_cb, this, _1));
  pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "gate_position", qos);
}

void GatesPublisher::gate_cb(const gazebo_msgs::msg::LinkStates::SharedPtr msg) const
{
  geometry_msgs::msg::PoseArray out;

  out.header = std_msgs::build<std_msgs::msg::Header>()
                   .stamp(this->now())
                   .frame_id("map");

  for (std::size_t i = 0; i < msg->name.size(); ++i)
    if (utils::contains(msg->name[i], "mindstorm_map::gate"))
      out.poses.emplace_back(geometry_msgs::build<geometry_msgs::msg::Pose>()
                                 .position(msg->pose[i].position)
                                 .orientation(msg->pose[i].orientation));

  for (;;)
  {
    pub_->publish(out);
    usleep(1000000);
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GatesPublisher>());
  rclcpp::shutdown();
  return 0;
}