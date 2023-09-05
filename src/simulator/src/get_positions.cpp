#include "simulator/get_positions.hpp"
// stl
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <unistd.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

PositionListener::PositionListener() : Node("get_positions")
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

  tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  publisher_   = this->create_publisher<geometry_msgs::msg::TransformStamped>("transform", qos);
  // timer_       = this->create_wall_timer(500ms, std::bind(&PositionListener::timer_callback, this));
  timer_ = this->create_wall_timer(100ms, std::bind(&PositionListener::timer_callback, this));
}

void PositionListener::timer_callback()
{
  geometry_msgs::msg::TransformStamped t;

  try
  {
    rclcpp::Time now = this->get_clock()->now();
    t                = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero, 1s);
  }
  catch (const tf2::TransformException &ex)
  {
    // RCLCPP_INFO(this->get_logger(), "Could not transform map to base_link: %s", ex.what());
    // return;
  }
  publisher_->publish(t);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionListener>());
  rclcpp::shutdown();
  return 0;
}
