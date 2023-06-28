#ifndef SIMUALTOR_GET_POSITIONS_HPP_
#define SIMUALTOR_GET_POSITIONS_HPP_

// rclcpp
#include "rclcpp/rclcpp.hpp"
// messages
#include "geometry_msgs/msg/transform_stamped.h"
// tf2
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class PositionListener : public rclcpp::Node
{
public:
  PositionListener();

private:
  // callbacks
  void timer_callback();

  // Member variables
  std::shared_ptr<tf2_ros::TransformListener>                        tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer>                                   tf_buffer_;
  rclcpp::TimerBase::SharedPtr                                       timer_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr publisher_;
  size_t                                                             count_;
};

#endif // SIMUALTOR_GET_POSITIONS_HPP_