#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>
#include <vector>

// rclcpp
#include "rclcpp/rclcpp.hpp"
// messages
#include "gazebo_msgs/msg/link_states.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/header.hpp"
// string utils
#include "utils.hpp"

using std::placeholders::_1;

class GatesPublisher : public rclcpp::Node
{
public:
  GatesPublisher() : Node("send_gates")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    sub_     = this->create_subscription<gazebo_msgs::msg::LinkStates>(
        "link_states", 10, std::bind(&GatesPublisher::gate_cb, this, _1));
    pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("gate_position", qos);
  }

private:
  // callbacks
  void gate_cb(const gazebo_msgs::msg::LinkStates::SharedPtr msg) const
  {
    std_msgs::msg::Header                 hh;
    geometry_msgs::msg::Pose              pose;
    std::vector<geometry_msgs::msg::Pose> pose_array;
    geometry_msgs::msg::PoseArray         pose_array_msg;

    hh.stamp    = this->now();
    hh.frame_id = "map";

    pose.position.z    = 0;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 0;

    for (std::size_t i = 0; i < msg->name.size(); ++i)
      if (utils::contains(msg->name[i], "mindstorm_map::gate"))
      {
        pose.position.x = msg->pose[i].position.x;
        pose.position.y = msg->pose[i].position.y;
        pose_array.emplace_back(pose);
      }
    pose_array_msg.header = hh;
    pose_array_msg.poses  = pose_array;

    while (1)
    {
      pub_->publish(pose_array_msg);
      usleep(1000000);
    }
  }

  // Member variables

  rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr   pub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GatesPublisher>());
  rclcpp::shutdown();
  return 0;
}