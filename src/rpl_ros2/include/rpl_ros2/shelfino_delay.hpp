#ifndef RPL_ROS2_SHELFINODELAY_HPP_
#define RPL_ROS2_SHELFINODELAY_HPP_

#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "rpl/types.hpp"
#include "rpl_msgs/msg/paths.hpp"
#include "rpl_msgs/msg/poses.hpp"
#include "rpl_ros2/adapters/PathsAdapter.hpp"
#include "rpl_ros2/adapters/PosesAdapter.hpp"

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(rpl::Paths, rpl_msgs::msg::Paths);
RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(rpl::Poses, rpl_msgs::msg::Poses);

namespace rpl_ros2
{
  class ShelfinoDelayNode : public rclcpp::Node
  {
  public:
    ShelfinoDelayNode();
    ~ShelfinoDelayNode();

  private:
    void get_waypoints(const std::size_t &shelfino);

    void shelfino1_cb(const rpl::Paths &msg);
    void shelfino2_cb(const rpl::Paths &msg);
    void shelfino3_cb(const rpl::Paths &msg);

    void exec();
    void timer_cb();

    void  set_priorities();
    float cumulative_sums(const rpl::Paths &path) const;
    float compute_delay_ms(const std::size_t &high, const std::size_t &low);

  private:
    // std::size_t n_shelfinos;
    rpl::Paths  paths[3];
    rpl::Poses  trajectory[3];
    float       delays[3]     = {0.f, 0.f, 0.f};
    std::size_t priorities[3] = {0, 1, 2};

    rclcpp::Subscription<rpl::Paths>::SharedPtr subscribers[3] = {nullptr, nullptr, nullptr};
    rclcpp::Publisher<rpl::Poses>::SharedPtr    publishers[3]  = {nullptr, nullptr, nullptr};
    std::size_t                                 n_shelfinos{0};
    std::uint8_t                                received{0u};
    std::uint8_t                                target{0u};

    rclcpp::TimerBase::SharedPtr timer_;
    std::thread                  thread_;
  };
} // namespace rpl_ros2

#endif // RPL_ROS2_SHELFINODELAY_HPP_