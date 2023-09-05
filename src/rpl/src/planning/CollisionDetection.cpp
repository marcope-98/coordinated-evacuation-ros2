#include "rpl/planning/CollisionDetection.hpp"

#include <cmath>
#include <iostream>

#include "rpl/common.hpp"

#include "rpl/internal/geometry.hpp"
#include "rpl/internal/rplintrin.hpp"
#include "rpl/internal/utils.hpp"

bool rpl::CollisionDetection::check_collision(const Path &candidate) const
{
  if (!std::isfinite(candidate.sum)) return true;

  Pose current = candidate.start;

  if (this->lr_collision(current, candidate.s1, bool(candidate.type & 1u))) return true;
  if (this->lsr_collision(current, candidate.s2, candidate.type)) return true;
  if (this->lr_collision(current, candidate.s3, bool(candidate.type & 3u) || (candidate.type == 5))) return true;

  return false;
}

bool rpl::CollisionDetection::lr_collision(Pose &current, const float &length, const bool &right) const
{
  Point  displacement = settings::rho() * Point{-sinf(current.theta), cosf(current.theta)};
  Circle circle       = {current.point(), settings::rho()};

  // center
  /*
    [cos -sin x]   [   0    ]   [ x +- rho * -sin ]
    [sin  cos y] * [ +- rho ] = [ y +- rho *  cos ]
    [ 0    0  1]   [   1    ]   [ 1               ]
  */

  float factor = right ? -1.f : +1.f;
  circle.center += factor * displacement;
  float min_range = utils::mod2pi(current.theta - factor * M_PI);
  float max_range = utils::mod2pi(min_range + factor * length * settings::kappa());
  current         = utils::interpolate(current, length, factor * settings::kappa());
  if (right) std::swap(min_range, max_range);

  // check collision
  for (const auto &polygon : this->d_obstacles)
  {
    for (std::size_t i = 0; i < polygon.size() - 1; ++i)
      if (geometry::arc_segment_intersection(circle, polygon[i], polygon[i + 1], min_range, max_range)) return true;
    if (geometry::arc_segment_intersection(circle, polygon[0], polygon[polygon.size() - 1], min_range, max_range)) return true;
  }
  return false;
}

bool rpl::CollisionDetection::s_collision(Pose &current, const float &length) const
{
  Point p = current.point();
  Point q = current.point() + length * Point{cosf(current.theta), sinf(current.theta)};

  for (const auto &polygon : this->d_obstacles)
  {
    for (std::size_t i = 0; i < polygon.size() - 1; ++i)
      if (geometry::intersects(p, q, polygon[i], polygon[i + 1])) return true;
    if (geometry::intersects(p, q, polygon[0], polygon[polygon.size() - 1])) return true;
  }
  current = utils::interpolate(current, length, 0.f);

  return false;
}

bool rpl::CollisionDetection::lsr_collision(Pose &current, const float &length, const std::size_t &type) const
{
  switch (type)
  {
    case 0: // LSL
    case 1: // RSL
    case 2: // LSR
    case 3: // RSR
      return this->s_collision(current, length);
    case 4: // LRL
      return this->lr_collision(current, length, true);
    case 5: // RLR
      return this->lr_collision(current, length, false);
    default:
      return true;
  }
}