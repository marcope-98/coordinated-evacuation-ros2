#ifndef RPL_INTERNAL_UTILS_HPP_
#define RPL_INTERNAL_UTILS_HPP_

// stl
#include <cmath>
// rpl
#include "rpl/common.hpp"

namespace rpl
{
  struct utils
  {
    static float mod2pi(const float &angle)
    {
      float temp = angle / settings::M_2PI();
      return (angle - settings::M_2PI() * floorf(temp));
    }

    static float sinc(const float &value)
    {
      const float one_over_120 = 1.f / 120.f;
      const float abs_value    = fabsf(value);
      const float value_2      = value * value;
      if (abs_value < 0.002f)
        return 1.f - one_over_120 * value_2 * (20.f - value_2);
      return sinf(value) / value;
    }
  };
} // namespace rpl

#endif // RPL_INTERNAL_UTILS_HPP_