#ifndef RPL_INTERNAL_UTILS_HPP_
#define RPL_INTERNAL_UTILS_HPP_

#include <cmath>
#include <iostream>

#include "rpl/common.hpp"
#include "rpl/types.hpp"

#include "rpl/internal/rplintrin.hpp"

namespace rpl
{
  struct utils
  {
    static float mod2pi(float angle)
    {
      float temp = angle / settings::M_2PI();
      temp       = (angle - settings::M_2PI() * floorf(temp));
      if (ops::cmpge_f32(temp, settings::M_2PI())) temp -= settings::M_2PI();
      return temp;
    }

    static float rangeSymm(const float &angle)
    {
#if 1
      float res = angle;
      while (res <= -M_PI)
        res += settings::M_2PI();
      while (res > M_PI)
        res -= settings::M_2PI();
      return res;
#else
      float temp = (angle + M_PI) / settings::M_2PI();
      return (angle - settings::M_2PI() * floorf(temp));
#endif
    }

    static float sinc(const float &value)
    {
      const float one_over_120  = 1.f / 120.f;
      const float value_squared = value * value;
      if (ops::cmplt_f32(ops::abs_f32(value), 0.002f))
        return 1.f - one_over_120 * value_squared * (20.f - value_squared);
      return sinf(value) / value;
    }

    static float sinc2(const float &value)
    {
      if (ops::cmplt_f32(ops::abs_f32(value), 0.f))
        return 1.f;
      return sinf(value) / value;
    }

    static Pose interpolate(Pose pose, const float &L, const float &k)
    {
      pose.x() += L * sinc(k * L * 0.5f) * cosf(pose.theta + k * L * 0.5f);
      pose.y() += L * sinc(k * L * 0.5f) * sinf(pose.theta + k * L * 0.5f);
      pose.theta = mod2pi(pose.theta + k * L);
      return pose;
    }
  };

} // namespace rpl

#endif // RPL_INTERNAL_UTILS_HPP_