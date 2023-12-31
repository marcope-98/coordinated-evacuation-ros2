#ifndef RPL_ROS2_ROADMAP_NODE_HPP_
#define RPL_ROS2_ROADMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

// rpl
#include "rpl/io/WorldDescriptor.hpp"
#include "rpl/map/RoadMap.hpp"
#include "rpl_ros2/adapters/RoadMapAdapter.hpp"
#include "rpl_ros2/adapters/WorldDescriptorAdapter.hpp"

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(rpl::WorldDescriptor, rpl_msgs::msg::WorldDescriptor);
RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(rpl::RoadMap, rpl_msgs::msg::RoadMap);

namespace rpl_ros2
{
  class RoadmapNode final : public rclcpp::Node
  {
  public:
    RoadmapNode();

  private:
    // Callbacks
    void world_descriptor_cb(const rpl::WorldDescriptor &wd);

    rpl::RoadMap rm;
    bool         initialized = false;

    rclcpp::Subscription<rpl::WorldDescriptor>::SharedPtr sub_;
    rclcpp::Publisher<rpl::RoadMap>::SharedPtr            pub_;
  };

} // namespace rpl_ros2

#endif // RPL_ROS2_ROADMAP_NODE_HPP_