#include "simulator/send_borders.hpp"

// stl
#include <chrono>
#include <functional>
#include <memory>
#include <unistd.h>

// rclcpp
#include "rclcpp/rclcpp.hpp"

// messages
#include "geometry_msgs/msg/point32.hpp"

BordersPublisher::BordersPublisher() : Node("send_borders")
{
  auto qos   = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
  this->pub_ = this->create_publisher<geometry_msgs::msg::Polygon>(
      "map_borders", qos);

  geometry_msgs::msg::Polygon              polygon;
  std::vector<geometry_msgs::msg::Point32> v_point32;
  v_point32.emplace_back(geometry_msgs::build<geometry_msgs::msg::Point32>().x(0.2f).y(0.2f).z(0.f));
  v_point32.emplace_back(geometry_msgs::build<geometry_msgs::msg::Point32>().x(0.2f).y(10.4f).z(0.f));
  v_point32.emplace_back(geometry_msgs::build<geometry_msgs::msg::Point32>().x(15.4f).y(10.4f).z(0.f));
  v_point32.emplace_back(geometry_msgs::build<geometry_msgs::msg::Point32>().x(15.4f).y(0.2f).z(0.f));
  polygon.points = v_point32;

  for (;;)
  {
    this->pub_->publish(polygon);
    usleep(1000000);
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BordersPublisher>());
  rclcpp::shutdown();
  return 0;
}
