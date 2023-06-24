#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/header.hpp"

using std::placeholders::_1;

class RPLNode : public rclcpp::Node
{
public:
  RPLNode() : Node("rpl_node")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    sub_     = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "gate_position", qos, std::bind(&RPLNode::topic_callback, this, _1));
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;

  bool initialized = false;

  void topic_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    if (this->initialized) return;
    this->initialized = true;

    for (const auto &elem : msg->poses)
    {
      std::cerr << elem.position.x << "\n";
      std::cerr << elem.position.y << "\n";
      std::cerr << elem.position.z << "\n";
      std::cerr << elem.orientation.x << "\n";
      std::cerr << elem.orientation.y << "\n";
      std::cerr << elem.orientation.z << "\n";
      std::cerr << elem.orientation.w << "\n";
    }
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RPLNode>();
  RCLCPP_INFO(node->get_logger(), "RPL node started.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
