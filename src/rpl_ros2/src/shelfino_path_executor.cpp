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

  this->paths_sub = this->create_subscription<rpl::Paths>(
      "final_path", qos, std::bind(&ShelfinoPathExecutorNode::paths_cb, this, _1));
  this->cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel", 10);
}

void rpl_ros2::ShelfinoPathExecutorNode::get_waypoints2()
{
  Command     command;
  rpl::Pose   current;
  const float step = 0.2f;
  float       rem  = 0.f;
  for (const auto &elem : this->path)
  {
    // rem     = 0.f;
    current = elem.start;

    // 1st segment
    command.primitive = bool(elem.type & 1u) ? -1.f : +1.f;
    if (elem.s1 < step)
      rem = step - elem.s1;
    else
    {
      while (rem < elem.s1)
      {
        command.goal = rpl::utils::interpolate(current, rem, command.primitive * rpl::settings::kappa());
        this->commands.emplace_back(command);
        rem += step;
      }
      rem -= elem.s1;
    }
    current = rpl::utils::interpolate(current, elem.s1, command.primitive * rpl::settings::kappa());

    // 2nd segment
    switch (elem.type)
    {
      case 4:
        command.primitive = -1.f;
        break;
      case 5:
        command.primitive = +1.f;
        break;
      default:
        command.primitive = +0.f;
    }

    if (rem > elem.s2)
      rem -= elem.s2;
    else
    {
      while (rem < elem.s2)
      {
        command.goal = rpl::utils::interpolate(current, rem, command.primitive * rpl::settings::kappa());
        this->commands.emplace_back(command);
        rem += step;
      }
      rem -= elem.s2;
    }
    current = rpl::utils::interpolate(current, elem.s2, command.primitive * rpl::settings::kappa());

    // 3rd segment
    command.primitive = (bool(elem.type & 3u) || elem.type == 5u) ? -1.f : +1.f;
    if (rem <= elem.s3)
    {
      while (rem < elem.s3)
      {
        command.goal = rpl::utils::interpolate(current, rem, command.primitive * rpl::settings::kappa());
        this->commands.emplace_back(command);
        rem += step;
      }
      rem -= elem.s3;
      command.goal = rpl::utils::interpolate(current, elem.s3, command.primitive * rpl::settings::kappa());
      this->commands.emplace_back(command);
    }
  }
}

void rpl_ros2::ShelfinoPathExecutorNode::get_waypoints()
{
  Command   command;
  rpl::Pose current;

  for (const auto &elem : this->path)
  {
    current = elem.start;

    // 1st segment
    command.primitive = bool(elem.type & 1u) ? -1.f : +1.f;
    current           = rpl::utils::interpolate(current, elem.s1, command.primitive * rpl::settings::kappa());
    command.goal      = current;
    if (rpl::ops::cmpgt_f32(elem.s1, 0.f))
      this->commands.emplace_back(command);

    // 2nd segment
    switch (elem.type)
    {
      case 4:
        command.primitive = -1.f;
        break;
      case 5:
        command.primitive = +1.f;
        break;
      default:
        command.primitive = +0.f;
    }
    current      = rpl::utils::interpolate(current, elem.s2, command.primitive * rpl::settings::kappa());
    command.goal = current;
    if (rpl::ops::cmpgt_f32(elem.s2, 0.f))
      this->commands.emplace_back(command);

    // 3rd segment
    command.primitive = (bool(elem.type & 3u) || elem.type == 5u) ? -1.f : +1.f;
    current           = rpl::utils::interpolate(current, elem.s3, command.primitive * rpl::settings::kappa());
    command.goal      = current;
    if (rpl::ops::cmpgt_f32(elem.s3, 0.f))
      this->commands.emplace_back(command);
  }
}

void rpl_ros2::ShelfinoPathExecutorNode::paths_cb(const rpl::Paths &paths)
{
  if (this->received || paths.empty()) return;
  this->received = true;
  // if (paths.empty()) std::cerr << this->get_namespace() << " has no solution\n";
  this->path = std::move(paths);
  this->get_waypoints2();

  auto qos       = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
  this->pose_sub = this->create_subscription<geometry_msgs::msg::TransformStamped>(
      "transform", qos, std::bind(&ShelfinoPathExecutorNode::pose_cb, this, _1));
  this->timer.start();
}

void rpl_ros2::ShelfinoPathExecutorNode::compute_deltas(const rpl::Pose &current, float &deltav, float &deltaw)
{
  rpl::Pose goal   = this->commands[this->current_waypoint].goal;
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

  rpl::Pose goal = this->commands[this->current_waypoint].goal;
  // std::cerr << goal.x() << " " << goal.y() << " " << goal.theta << "\n";
  float deltav, deltaw;
  this->compute_deltas(current, deltav, deltaw);
  if (this->current_waypoint == this->commands.size())
  {
    this->cmd_vel_pub->publish(this->stop());
    if (once)
    {
      std::cerr << "Travel time: " << float(timer.stop()) * 0.001f << "ms\n";
      once = false;
    }
    return;
  }

  if ((current.point() - this->commands[this->current_waypoint].goal.point()).norm() < 0.4f)
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