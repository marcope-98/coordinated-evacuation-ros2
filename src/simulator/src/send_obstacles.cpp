// stl
#include <chrono>
#include <cstdint>
#include <functional>
#include <string>
#include <unistd.h>
#include <vector>
// rclcpp and gazebo
#include "rclcpp/rclcpp.hpp"
// messages
#include "gazebo_msgs/msg/link_states.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"
#include "std_msgs/msg/header.hpp"
// string utils
#include "utils.hpp"
#include <iostream>

using std::placeholders::_1;

class ObstaclePublisher : public rclcpp::Node
{
public:
  ObstaclePublisher() : Node("obstacle_publisher")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    pub_     = this->create_publisher<obstacles_msgs::msg::ObstacleArrayMsg>("obstacles", qos);
    sub_     = this->create_subscription<gazebo_msgs::msg::LinkStates>(
        "link_states", 10, std::bind(&ObstaclePublisher::obstacles_cb, this, _1));
  }

private:
  // Callbacks
  void obstacles_cb(const gazebo_msgs::msg::LinkStates::SharedPtr msg) const
  {
    std_msgs::msg::Header                 hh;
    geometry_msgs::msg::Point32           point;
    geometry_msgs::msg::Polygon           pol;
    obstacles_msgs::msg::ObstacleMsg      obs_msg;
    obstacles_msgs::msg::ObstacleArrayMsg obs_arr_msg;

    std::vector<geometry_msgs::msg::Point32>      points;
    std::vector<obstacles_msgs::msg::ObstacleMsg> obstacles;

    hh.stamp    = this->now();
    hh.frame_id = "map";
    point.z     = 0;
    std::vector<std::string> link_state;
    for (std::size_t i = 0; i < msg->name.size(); ++i)
    {
      link_state = utils::split_by_delimiter(msg->name[i], "::");
      if (!utils::contains(link_state[0], "mindstorm_map") ||
          !utils::contains(link_state[1], "obstacle"))
        continue;

      if (utils::contains(link_state[2], "link"))
      {
        // if (!points.empty())
        // {
        pol.points      = points;
        obs_msg.polygon = pol;
        obstacles.emplace_back(obs_msg);
        points.clear();
        //   }
      }
      else
      {
        point.x = msg->pose[i].position.x;
        point.y = msg->pose[i].position.y;
        points.emplace_back(point);
      }
    }

    pol.points      = points;
    obs_msg.polygon = pol;
    obstacles.emplace_back(obs_msg);
    points.clear();
    obs_arr_msg.header    = hh;
    obs_arr_msg.obstacles = obstacles;

    while (1)
    {
      pub_->publish(obs_arr_msg);
      usleep(1000000);
    }
  }
  // Member variables
  rclcpp::Publisher<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr pub_;
  rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr       sub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstaclePublisher>());
  rclcpp::shutdown();
  return 0;
}