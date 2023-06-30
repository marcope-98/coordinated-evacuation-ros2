#ifndef RPL_ROS2_WORLD_DESCRIPTOR_HPP
#define RPL_ROS2_WORLD_DESCRIPTOR_HPP

#include <cstdint>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"

#include "rpl/io/WorldDescriptor.hpp"
#include "rpl_ros2/adapters/WorldDescriptorAdapter.hpp"

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(rpl::WorldDescriptor, rpl_msgs::msg::WorldDescriptor);

namespace rpl_ros2
{
  class WorldDescriptorNode final : public rclcpp::Node
  {
  public:
    // Constructor
    WorldDescriptorNode();
    ~WorldDescriptorNode();

  private:
    // Callbacks
    void gate_cb(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void border_cb(const geometry_msgs::msg::Polygon::SharedPtr msg);
    void obstacles_cb(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg);
    void wd_publisher();

    // Member variables
    rpl::WorldDescriptor wd;
    std::uint8_t         called = 0u;

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr         sub_gate;
    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr           sub_borders;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr sub_obstacles;
    rclcpp::Publisher<rpl::WorldDescriptor>::SharedPtr                     pub_;
    std::thread                                                            thread_;
  };
} // namespace rpl_ros2

#endif // RPL_ROS2_WORLD_DESCRIPTOR_HPP