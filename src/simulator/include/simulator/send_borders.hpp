#ifndef SIMULATOR_SEND_BORDERS_HPP_
#define SIMULATOR_SEND_BORDERS_HPP_

// rclcpp
#include "rclcpp/rclcpp.hpp"

// messages
#include "geometry_msgs/msg/polygon.hpp"

class BordersPublisher : public rclcpp::Node
{
public:
  BordersPublisher();

private:
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr pub_;
};

#endif // SIMULATOR_SEND_BORDERS_HPP_