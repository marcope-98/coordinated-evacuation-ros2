#include "rpl_ros2/shelfino_delay.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <string>

#include "rpl/Timer.hpp"
#include "rpl/common.hpp"
#include "rpl/internal/geometry.hpp"
#include "rpl/internal/rplintrin.hpp"
#include "rpl/internal/utils.hpp"

using std::placeholders::_1;

rpl_ros2::ShelfinoDelayNode::ShelfinoDelayNode() : rclcpp::Node("shelfino_delay")
{
  this->declare_parameter("n_shelfinos", "0");
  this->n_shelfinos = std::atoll(this->get_parameter("n_shelfinos").as_string().c_str());
  auto qos          = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
  switch (this->n_shelfinos)
  {
    case 3:
      // subscribe to /shelfino3/
      this->subscribers[2] = this->create_subscription<rpl::Paths>(
          "shelfino3/generated_path", qos, std::bind(&ShelfinoDelayNode::shelfino3_cb, this, _1));
      this->publishers[2] = this->create_publisher<rpl::Poses>(
          "shelfino3/final_path", qos);
    case 2:
      // subscribe to /shelfino2/
      this->subscribers[1] = this->create_subscription<rpl::Paths>(
          "shelfino2/generated_path", qos, std::bind(&ShelfinoDelayNode::shelfino2_cb, this, _1));
      this->publishers[1] = this->create_publisher<rpl::Poses>(
          "shelfino2/final_path", qos);
    case 1:
      // subscribe to /shelfino1/
      this->subscribers[0] = this->create_subscription<rpl::Paths>(
          "shelfino1/generated_path", qos, std::bind(&ShelfinoDelayNode::shelfino1_cb, this, _1));
      this->publishers[0] = this->create_publisher<rpl::Poses>(
          "shelfino1/final_path", qos);
      break;
    default:
      break;
  }
  this->target = 0x07u >> (3 - this->n_shelfinos);

  this->thread_ = std::thread(std::bind(&ShelfinoDelayNode::exec, this));
}

rpl_ros2::ShelfinoDelayNode::~ShelfinoDelayNode()
{
  if (this->thread_.joinable()) this->thread_.join();
}

void rpl_ros2::ShelfinoDelayNode::get_waypoints(const std::size_t &shelfino)
{
  rpl::Pose   current;
  const float step = 0.2f;
  float       rem  = 0.f;
  float       primitive;
  for (const auto &path : this->paths[shelfino])
  {
    current = path.start;

    // 1st segment
    primitive = bool(path.type & 1u) ? -1.f : +1.f;
    if (path.s1 < step)
      rem = step;
    else
      while (rem < path.s1)
      {
        this->trajectory[shelfino].emplace_back(rpl::utils::interpolate(current, rem, primitive * rpl::settings::kappa()));
        rem += step;
      }
    rem -= path.s1;

    current = rpl::utils::interpolate(current, path.s1, primitive * rpl::settings::kappa());

    // 2nd segment
    switch (path.type)
    {
      case 4:
        primitive = -1.f;
        break;
      case 5:
        primitive = +1.f;
        break;
      default:
        primitive = 0.f;
    }

    while (rem < path.s2)
    {
      this->trajectory[shelfino].emplace_back(rpl::utils::interpolate(current, rem, primitive * rpl::settings::kappa()));
      rem += step;
    }

    current = rpl::utils::interpolate(current, path.s2, primitive * rpl::settings::kappa());
    rem -= path.s2;

    // 3rd segment
    primitive = (bool(path.type & 3u) || path.type == 5u) ? -1.f : +1.f;
    while (rem < path.s3)
    {
      this->trajectory[shelfino].emplace_back(rpl::utils::interpolate(current, rem, primitive * rpl::settings::kappa()));
      rem += step;
    }
    rem -= path.s3;
  }
}

void rpl_ros2::ShelfinoDelayNode::shelfino1_cb(const rpl::Paths &msg)
{
  auto is_bit_set = bool(this->received & 0x01);
  if (is_bit_set) return;
  this->paths[0] = msg;
  this->get_waypoints(0);
  this->received |= 0x01;
}

void rpl_ros2::ShelfinoDelayNode::shelfino2_cb(const rpl::Paths &msg)
{
  auto is_bit_set = bool(this->received & 0x02);
  if (is_bit_set) return;
  this->paths[1] = msg;
  this->get_waypoints(1);
  this->received |= 0x02;
}

void rpl_ros2::ShelfinoDelayNode::shelfino3_cb(const rpl::Paths &msg)
{
  auto is_bit_set = bool(this->received & 0x04);
  if (is_bit_set) return;
  this->paths[2] = msg;
  this->get_waypoints(2);
  this->received |= 0x04;
}

float rpl_ros2::ShelfinoDelayNode::cumulative_sums(const rpl::Paths &path) const
{
  if (path.empty()) return rpl::ops::hex_to_f32(0x7F800000);
  float sum = 0.f;
  for (const auto &segment : path)
    sum += segment.sum;
  return sum;
}

void rpl_ros2::ShelfinoDelayNode::set_priorities()
{
  float cum_sum[3] = {0.f, 0.f, 0.f};
  for (std::size_t i = 0; i < 3; ++i)
    cum_sum[i] = this->cumulative_sums(this->paths[i]);

  std::sort(this->priorities,
            this->priorities + this->n_shelfinos,
            [&](const std::size_t &L1, const std::size_t &L2)
            { return cum_sum[L1] < cum_sum[L2]; });
}

void rpl_ros2::ShelfinoDelayNode::exec()
{
  while (rclcpp::ok())
  {
    if (!(this->received == this->target)) continue;
    this->received = 0xFF;
    rpl::Timer timer;
    timer.start();
    // set priorities
    this->set_priorities();

    // compute delays
    float delay = 0.f;
    // Handle two lowest priority paths
    delay = this->compute_delay_ms(this->priorities[1], this->priorities[2]);
    this->delays[this->priorities[2]] += delay;
    // Handle two highest priority paths
    delay += this->compute_delay_ms(this->priorities[0], this->priorities[1]);
    // add delays to lower priority paths
    this->delays[this->priorities[1]] += delay;
    this->delays[this->priorities[2]] += delay;

    for (std::size_t i = 0; i < this->n_shelfinos; ++i)
      std::cerr << this->delays[i] << "\n";

    std::cerr << "Shelfino Delay computation time: " << float(timer.stop()) * 0.001f << "ms\n";

    // register timer_callback and execute
    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&ShelfinoDelayNode::timer_cb, this));
  }
}

float rpl_ros2::ShelfinoDelayNode::compute_delay_ms(const std::size_t &high, const std::size_t &low)
{
  float delay = 0.f;

  rpl::Poses pathH = this->trajectory[high];
  rpl::Poses pathL = this->trajectory[low];

  for (std::size_t i = 0, j = 0;
       i < pathH.size() && j < pathL.size();
       ++i, ++j)
  {
    if ((pathH[i].point() - pathL[j].point()).norm() < .7f)
    {
      delay += .4f;
      --j;
    }
  }

  return ceilf(delay * 1000.f / rpl::settings::linear());
}

void rpl_ros2::ShelfinoDelayNode::timer_cb()
{
  for (std::size_t i = 0; i < this->n_shelfinos; ++i)
  {
    if (this->delays[i] > 0.f)
      this->delays[i] -= 200.f;
    else
      this->publishers[i]->publish(this->trajectory[i]);
  }
  // NOTE: no need to sleep since this is a timer callback
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rpl_ros2::ShelfinoDelayNode>());
  rclcpp::shutdown();
}
