#ifndef RPL_ROS2_ADAPTERS_PATHADAPTER_HPP_
#define RPL_ROS2_ADAPTERS_PATHADAPTER_HPP_

// rclcpp
#include "rclcpp/rclcpp.hpp"
// rpl
#include "rpl/types.hpp"
// rpl_msgs
#include "rpl_msgs/msg/path.hpp"
// rpl_ros2
#include "rpl_ros2/adapters/PoseAdapter.hpp"

template<>
struct rclcpp::TypeAdapter<rpl::Path, rpl_msgs::msg::Path>
{
  using is_specialized   = std::true_type;
  using custom_type      = rpl::Path;
  using ros_message_type = rpl_msgs::msg::Path;
  using PoseAdapter      = rclcpp::TypeAdapter<rpl::Pose, rpl_msgs::msg::Pose>;

  static void convert_to_ros_message(const custom_type &source,
                                     ros_message_type & destination)
  {
    destination.type = source.type;
    PoseAdapter::convert_to_ros_message(source.start, destination.start);
    PoseAdapter::convert_to_ros_message(source.end, destination.end);
    destination.lengths = {source.s1, source.s2, source.s3};
    destination.sum     = source.sum;
  }

  static void convert_to_custom(const ros_message_type &source,
                                custom_type &           destination)
  {
    destination.type = source.type;
    PoseAdapter::convert_to_custom(source.start, destination.start);
    PoseAdapter::convert_to_custom(source.end, destination.end);
    destination.s1  = source.lengths[0];
    destination.s2  = source.lengths[1];
    destination.s3  = source.lengths[2];
    destination.sum = source.sum;
  }
};

#endif // RPL_ROS2_ADAPTERS_PATHADAPTER_HPP_