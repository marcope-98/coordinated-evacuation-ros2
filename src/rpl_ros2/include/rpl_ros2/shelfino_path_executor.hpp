#ifndef RPL_ROS2_SHELFINOPATHEXECUTOR_HPP_
#define RPL_ROS2_SHELFINOPATHEXECUTOR_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rpl/Timer.hpp"
#include "rpl/common.hpp"
#include "rpl/types.hpp"
#include "rpl_msgs/msg/paths.hpp"
#include "rpl_ros2/adapters/PathsAdapter.hpp"

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(rpl::Paths, rpl_msgs::msg::Paths);

namespace rpl_ros2
{
  class ShelfinoPathExecutorNode : public rclcpp::Node
  {
  public:
    ShelfinoPathExecutorNode();

  private:
    void compute_deltas(const rpl::Pose &current, float &deltav, float &deltaw);
    void get_waypoints();
    void get_waypoints2();

    geometry_msgs::msg::Twist straight(const float &deltav, const float &deltaw)
    {
      geometry_msgs::msg::Twist res = this->stop();
      res.angular.z                 = deltaw;
      if (deltaw > rpl::settings::angular())
        res.angular.z = rpl::settings::angular();
      if (deltaw < -rpl::settings::angular())
        res.angular.z = -rpl::settings::angular();

      res.linear.x = rpl::settings::linear() + deltav;
      return res;
    }

    geometry_msgs::msg::Twist stop()
    {
      geometry_msgs::msg::Twist res;
      res.linear.x  = 0.;
      res.linear.y  = 0.;
      res.linear.z  = 0.;
      res.angular.x = 0.;
      res.angular.y = 0.;
      res.angular.z = 0.;
      return res;
    }

    void paths_cb(const rpl::Paths &paths);

    void pose_cb(const geometry_msgs::msg::TransformStamped::SharedPtr msg);

  private:
    struct Command
    {
      rpl::Pose goal;
      float     primitive; // 0 straight, +1 left, -1 right
    };

    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr pose_sub;
    rclcpp::Subscription<rpl::Paths>::SharedPtr                           paths_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr               cmd_vel_pub;

    rpl::Paths           path;
    std::vector<Command> commands;
    std::size_t          current_waypoint{0};

    float kp = 0.f;
    float kt = 1.f;

    rpl::Timer timer;

    bool received{false};
    bool once{true};
  };
} // namespace rpl_ros2

#endif // RPL_ROS2_SHELFINOPATHEXECUTOR_HPP_