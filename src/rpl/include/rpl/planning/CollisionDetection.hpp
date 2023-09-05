#ifndef RPL_PLANNING_COLLISIONDETECTION_HPP_
#define RPL_PLANNING_COLLISIONDETECTION_HPP_
#include <cstdint>
#include <utility>
#include <vector>

#include "rpl/types.hpp"

#include "rpl/planning/Dubins.hpp"

namespace rpl
{
  struct CollisionDetection
  {
  public:
    CollisionDetection() = default;
    explicit CollisionDetection(std::vector<Polygon> obstacles) : d_obstacles(std::move(obstacles)) {}
    CollisionDetection(const CollisionDetection &other) { *this = other; }
    CollisionDetection(CollisionDetection &&other) noexcept { *this = std::move(other); }
    ~CollisionDetection() = default;

    CollisionDetection &operator=(const CollisionDetection &other)
    {
      this->d_obstacles = other.d_obstacles;
      return *this;
    }
    CollisionDetection &operator=(CollisionDetection &&other) noexcept
    {
      std::swap(this->d_obstacles, other.d_obstacles);
      return *this;
    }

    bool check_collision(const Path &candidate) const;

  private:
    bool s_collision(Pose &current, const float &length) const;
    bool lr_collision(Pose &current, const float &length, const bool &right) const;
    bool lsr_collision(Pose &current, const float &length, const std::size_t &type) const;

  private:
    std::vector<Polygon> d_obstacles;
  };
} // namespace rpl
#endif // RPL_PLANNING_COLLISIONDETECTION_HPP_