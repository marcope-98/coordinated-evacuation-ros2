#ifndef RPL_COMMON_HPP_
#define RPL_COMMON_HPP_

#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdint>

namespace rpl
{
  struct settings
  {
    // Math
    constexpr static float M_2PI() { return 2.f * M_PI; }

    // ClipperLib
    constexpr static float       working_envelope_inner() { return 0.3f; }
    constexpr static float       working_envelope_outer() { return 0.4f; }
    constexpr static std::size_t max_polygon_size() { return 6; }
    constexpr static float       exp_factor() { return 1000.f; }
    constexpr static float       iexp_factor() { return 1.f / exp_factor(); }

    // Dubins
    // static constexpr float       rho() { return linear() / angular(); }
    static constexpr float       rho() { return 0.9f; }
    static constexpr float       angular() { return 0.5f; }
    static constexpr float       linear() { return 0.3f; }
    static constexpr float       kappa() { return 1.f / rho(); }
    constexpr static std::size_t granularity() { return 360; }
    constexpr static std::size_t primitives() { return 6; }
    constexpr static float       step() { return M_2PI() / float(granularity()); }
  };

} // namespace rpl

#endif // RPL_COMMON_HPP_