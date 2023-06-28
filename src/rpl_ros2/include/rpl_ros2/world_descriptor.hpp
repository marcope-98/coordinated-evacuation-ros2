#ifndef RPL_ROS2_WORLD_DESCRIPTOR_HPP
#define RPL_ROS2_WORLD_DESCRIPTOR_HPP

#include <cstdint>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"

#include "rpl/io/WorldDescriptor.hpp"

namespace rpl_ros2
{
  class WorldDescriptorNode : public rclcpp::Node
  {
  public:
    // Constructor
    WorldDescriptorNode();

  private:
    // Callbacks
    void gate_cb(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void border_cb(const geometry_msgs::msg::Polygon::SharedPtr msg);
    void obstacles_cb(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg);

    // Member variables
    rpl::WorldDescriptor wd;
    std::uint8_t         called = 0u;

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr         sub_gate;
    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr           sub_borders;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr sub_obstacles;
  };
} // namespace rpl_ros2

#endif // RPL_ROS2_WORLD_DESCRIPTOR_HPP