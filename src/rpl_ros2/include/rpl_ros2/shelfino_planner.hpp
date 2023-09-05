#ifndef RPL_ROS2_SHELFINOPLANNER_HPP
#define RPL_ROS2_SHELFINOPLANNER_HPP
#include <cstdint>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"

#include "rpl/types.hpp"

#include "rpl/io/WorldDescriptor.hpp"
#include "rpl/map/RoadMap.hpp"
#include "rpl/planning/CollisionDetection.hpp"
#include "rpl_msgs/msg/paths.hpp"
#include "rpl_msgs/msg/road_map.hpp"
#include "rpl_msgs/msg/world_descriptor.hpp"
#include "rpl_ros2/adapters/PathsAdapter.hpp"
#include "rpl_ros2/adapters/RoadMapAdapter.hpp"
#include "rpl_ros2/adapters/WorldDescriptorAdapter.hpp"

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(rpl::Paths, rpl_msgs::msg::Paths);
RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(rpl::RoadMap, rpl_msgs::msg::RoadMap);
RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(rpl::WorldDescriptor, rpl_msgs::msg::WorldDescriptor);

namespace rpl_ros2
{
  class ShelfinoPlannerNode final : public rclcpp::Node
  {
  public:
    ShelfinoPlannerNode();
    ~ShelfinoPlannerNode();

  private:
    // Callbacks
    void roadmap_cb(const rpl::RoadMap &rm);
    void pose_cb(const geometry_msgs::msg::TransformStamped::SharedPtr msg);
    void world_description_cb(const rpl::WorldDescriptor &msg);
    void exec();

    // subscribers
    rclcpp::Subscription<rpl::RoadMap>::SharedPtr                         sub_rm;
    rclcpp::Subscription<rpl::WorldDescriptor>::SharedPtr                 sub_wd;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr sub_pose;
    std::uint8_t                                                          called = 0x00;
    // member variables
    rpl::RoadMap            rm;
    rpl::CollisionDetection cd;
    rpl::Pose               start_pose;
    // publisher
    std::thread                              thread_;
    rclcpp::Publisher<rpl::Paths>::SharedPtr pub;
  };
} // namespace rpl_ros2

#endif // RPL_ROS2_ROADMAPPLANNER_HPP