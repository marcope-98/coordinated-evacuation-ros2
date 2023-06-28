#ifndef SIMULATOR_SEND_GATES_HPP_
#define SIMULATOR_SEND_GATES_HPP_

// rclcpp
#include "rclcpp/rclcpp.hpp"

// messages
#include "gazebo_msgs/msg/link_states.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

class GatesPublisher : public rclcpp::Node
{
public:
  GatesPublisher();

private:
  void gate_cb(const gazebo_msgs::msg::LinkStates::SharedPtr msg) const;

  rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr   pub_;
};

#endif // SIMULATOR_SEND_GATES_HPP_