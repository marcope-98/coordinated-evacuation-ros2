#ifndef RPL_TYPES_HPP_
#define RPL_TYPES_HPP_

#include <cstdint>
#include <vector>

#include "rpl/internal/point.hpp"

namespace rpl
{
  // POSE
  struct Pose : public Point
  {
    float theta{0.f};
    Pose() : Point() {}
    Pose(const Point &point, const float &theta_) : Point{point}, theta{theta_} {}

    Point point() const { return Point(this->x(), this->y()); }
  };

  struct Circle
  {
    Point center;
    float radius{0.f};
  };

  struct Path
  {
    std::size_t type;
    Pose        start;
    Pose        end;
    float       s1;
    float       s2;
    float       s3;
    float       sum;
  };

  // ALIASES
  using Polygon = std::vector<Point>;
  using Paths   = std::vector<Path>;

} // namespace rpl

#endif // RPL_TYPES_HPP_