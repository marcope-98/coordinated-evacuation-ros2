// stl
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <unistd.h>

// rclcpp
#include "rclcpp/rclcpp.hpp"

// messages
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"

class BordersPublisher : public rclcpp::Node
{
public:
  BordersPublisher()
      : Node("send_borders")
  {
    auto qos   = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    publisher_ = this->create_publisher<geometry_msgs::msg::Polygon>("map_borders", qos);

    geometry_msgs::msg::Polygon pol;
    geometry_msgs::msg::Point32 point;

    std::vector<geometry_msgs::msg::Point32> points_temp;

    point.x = 0.2;
    point.y = 0.2;
    point.z = 0;
    points_temp.push_back(point);

    point.x = 0.2;
    point.y = 10.4;
    point.z = 0;
    points_temp.push_back(point);

    point.x = 15.4;
    point.y = 10.4;
    point.z = 0;
    points_temp.push_back(point);

    point.x = 15.4;
    point.y = 0.2;
    point.z = 0;
    points_temp.push_back(point);

    pol.points = points_temp;

    while (1)
    {
      publisher_->publish(pol);
      usleep(1000000);
    }
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BordersPublisher>());
  rclcpp::shutdown();
  return 0;
}
