#include "rpl_ros2/shelfino_path_executor.hpp"
#include <cstdlib>
#include <cstring>
#include <functional>

#include "rpl/internal/rplintrin.hpp"
#include "rpl/internal/utils.hpp"

#include "geometry_msgs/msg/vector3.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"

using std::placeholders::_1;

rpl_ros2::ShelfinoPathExecutorNode::ShelfinoPathExecutorNode() : rclcpp::Node("shelfino_executor_node")
{
  // subscriber to path
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

  this->poses_sub = this->create_subscription<rpl::Poses>(
      "final_path", qos, std::bind(&ShelfinoPathExecutorNode::poses_cb, this, _1));
  this->cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel", 10);
}

void rpl_ros2::ShelfinoPathExecutorNode::poses_cb(const rpl::Poses &poses)
{
  if (this->received || poses.empty()) return;
  this->received = true;
  this->path     = std::move(poses);

  auto qos       = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
  this->pose_sub = this->create_subscription<geometry_msgs::msg::TransformStamped>(
      "transform", qos, std::bind(&ShelfinoPathExecutorNode::pose_cb, this, _1));
  this->timer.start();
}

void rpl_ros2::ShelfinoPathExecutorNode::compute_deltas(const rpl::Pose &current, float &deltav, float &deltaw)
{
  rpl::Pose goal   = this->path[this->current_waypoint];
  float     ex     = (current.x() - goal.x());
  float     ey     = (current.y() - goal.y());
  float     etheta = rpl::utils::rangeSymm(current.theta - goal.theta);
  float     exy    = sqrtf(ex * ex + ey * ey);
  float     psi    = atan2f(ey, ex);
  float     alpha  = rpl::utils::rangeSymm(current.theta) + rpl::utils::rangeSymm(goal.theta);

  deltav = -this->kp * exy * cosf(rpl::utils::rangeSymm(current.theta) - psi);
  deltaw = -rpl::settings::linear() * rpl::utils::sinc(etheta * 0.5f) * sinf(psi - 0.5f * alpha) - this->kt * etheta;
}

void rpl_ros2::ShelfinoPathExecutorNode::pose_cb(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
{
  tf2::Quaternion tf_quat(msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w);
  tf2::Matrix3x3  m(tf_quat);
  double          r, p, y;
  m.getRPY(r, p, y);

  rpl::Pose current{
      rpl::Point{float(msg->transform.translation.x),
                 float(msg->transform.translation.y)},
      rpl::utils::mod2pi(float(y))};

  float deltav, deltaw;
  this->compute_deltas(current, deltav, deltaw);
  if (this->current_waypoint == this->path.size())
  {
    this->cmd_vel_pub->publish(this->stop());
    if (once)
    {
      std::cerr << "Travel time: " << float(timer.stop()) * 0.001f << "ms\n";
      once = false;
    }
    return;
  }

  if ((current.point() - this->path[this->current_waypoint].point()).norm() < 0.4f)
    this->current_waypoint += 1;
  else
    this->cmd_vel_pub->publish(this->straight(deltav, deltaw));
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rpl_ros2::ShelfinoPathExecutorNode>());
  rclcpp::shutdown();
}