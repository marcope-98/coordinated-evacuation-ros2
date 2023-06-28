#ifndef SIMULATOR_SEND_OBSTACLES_HPP_
#define SIMULATOR_SEND_OBSTACLES_HPP_

// rclcpp
#include "rclcpp/rclcpp.hpp"
// messages
#include "gazebo_msgs/msg/link_states.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"

class ObstaclePublisher : public rclcpp::Node
{
public:
  // Constructor
  ObstaclePublisher();

private:
  // Callback
  void obstacles_cb(const gazebo_msgs::msg::LinkStates::SharedPtr msg) const;
  // Member variables
  rclcpp::Publisher<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr pub_;
  rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr       sub_;
};

#endif // SIMULATOR_SEND_OBSTACLES_HPP_