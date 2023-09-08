#include "rpl_ros2/shelfino_planner.hpp"

#include <chrono>
#include <functional>
#include <iostream>
#include <vector>

#include "rpl/Timer.hpp"
#include "rpl/planning/Dubins.hpp"

#include "geometry_msgs/msg/vector3.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

rpl_ros2::ShelfinoPlannerNode::~ShelfinoPlannerNode()
{
  if (this->thread_.joinable()) this->thread_.join();
}

rpl_ros2::ShelfinoPlannerNode::ShelfinoPlannerNode() : rclcpp::Node("shelfino_planner")
{
  auto qos     = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
  this->sub_wd = this->create_subscription<rpl::WorldDescriptor>(
      "world_description", qos, std::bind(&ShelfinoPlannerNode::world_description_cb, this, _1));
  this->sub_rm = this->create_subscription<rpl::RoadMap>(
      "roadmap_topic", qos, std::bind(&ShelfinoPlannerNode::roadmap_cb, this, _1));
  this->sub_pose = this->create_subscription<geometry_msgs::msg::TransformStamped>(
      "transform", qos, std::bind(&ShelfinoPlannerNode::pose_cb, this, _1));

  this->thread_ = std::thread(std::bind(&ShelfinoPlannerNode::exec, this));
  this->pub     = this->create_publisher<rpl::Paths>(
      "generated_path", qos);
}

void rpl_ros2::ShelfinoPlannerNode::pose_cb(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
{
  auto is_bit_set = bool(this->called & 0x01);
  if (is_bit_set) return;

  tf2::Quaternion tf_quat(msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w);
  tf2::Matrix3x3  m(tf_quat);
  double          r, p, y;
  m.getRPY(r, p, y);
  this->start_pose = rpl::Pose(rpl::Point{float(msg->transform.translation.x), float(msg->transform.translation.y)}, rpl::utils::mod2pi(float(y)));
  this->called |= 0x01;
}

void rpl_ros2::ShelfinoPlannerNode::roadmap_cb(const rpl::RoadMap &msg)
{
  auto is_bit_set = bool(this->called & 0x02);
  if (is_bit_set) return;
  this->rm = msg;
  this->called |= 0x02;
}

void rpl_ros2::ShelfinoPlannerNode::world_description_cb(const rpl::WorldDescriptor &msg)
{
  auto is_bit_set = bool(this->called & 0x04);
  if (is_bit_set) return;
  std::vector<rpl::Polygon> obstacles;
  for (const auto &polygon : msg.obstacles_inner)
    obstacles.emplace_back(polygon);
  obstacles.emplace_back(msg.border_inner);
  this->cd = std::move(rpl::CollisionDetection(obstacles));
  this->called |= 0x04;
}

void rpl_ros2::ShelfinoPlannerNode::exec()
{
  while (rclcpp::ok())
  {
    if (!(this->called == 0x07)) continue;
    this->called = 0x0F;

    rpl::Timer timer;
    timer.start();

    rpl::Dubins                           db;
    std::vector<rpl ::RoadMap::Candidate> rm_candidates = this->rm.dijkstra(this->start_pose);
    rpl::Paths                            db_candidates;
    rpl::Paths                            solution;

    std::size_t i = 0;
    for (i = 0; i < rm_candidates.size() - 1; ++i)
    {
      db.execute(rm_candidates[i].pose, rm_candidates[i + 1].pose,
                 rm_candidates[i + 1].min_range, rm_candidates[i + 1].max_range,
                 db_candidates);

      std::size_t best = rpl::settings::primitives();
      for (std::size_t primitive = 0; primitive < rpl::settings::primitives(); ++primitive)
      {
        if (!this->cd.check_collision(db_candidates[primitive]))
        {
          best = primitive;
          break;
        }
      }

      if (best == rpl::settings::primitives())
      {
        this->rm.graph().rm_edge(rm_candidates[i].ref_index,
                                 rm_candidates[i + 1].ref_index); // remove edge from graph
        solution.clear();                                         // solution clear
        rm_candidates = this->rm.dijkstra(this->start_pose);      // repeat dijkstra
        if (rm_candidates.empty()) break;                         // no solution
        i = -1;
      }
      else
      {
        rm_candidates[i + 1].pose.theta = db_candidates[best].end.theta;
        solution.emplace_back(db_candidates[best]); // solution.emplace_back
      }
    }

    std::cerr << "Solution time: " << float(timer.stop()) * 0.001f << "ms\n";
    if (solution.empty()) std::cerr << "no solution\n";

    for (;;)
    {
      this->pub->publish(solution);
      usleep(10000); // NOTE: 10ms
    }
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rpl_ros2::ShelfinoPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
