#ifndef RPL_TYPES_HPP_
#define RPL_TYPES_HPP_

#ifndef INCLUDE_VECTOR
#include <vector>
#define INCLUDE_VECTOR
#endif

namespace rpl
{
  struct Point
  {
    float x = 0.f;
    float y = 0.f;
    Point() = default;
    Point(const float &x_, const float &y_) : x(x_), y(y_) {}
  };

  struct Pose : public Point
  {
    float theta = 0.f;
  };

  using Polygon = std::vector<Point>;
  using Vector  = Point[2];
} // namespace rpl

#endif // RPL_TYPES_HPP_
