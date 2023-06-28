#ifndef RPL_COMMON_HPP_
#define RPL_COMMON_HPP_

#define USE_MATH_DEFINES
#include <cmath>
#include <cstdint>

// TODO: separate mathematical constants from actual configuration
namespace rpl
{
  struct settings
  {
    static constexpr float       min_turning_radius() { return 1.F; }
    static constexpr float       kappa() { return 1.F / min_turning_radius(); }
    static constexpr float       working_envelope() { return 0.112F; }
    static constexpr float       exp_factor() { return 1000.F; }
    static constexpr float       iexp_factor() { return 1.F / exp_factor(); }
    static constexpr float       M_2PI() { return 2.F * M_PI; }
    static constexpr std::size_t max_polygon_size() { return 6; }
    static constexpr std::size_t granularity() { return 360; }
    static constexpr float       angle_step() { return M_2PI() / float(granularity()); }
    static constexpr float       interp_step() { return 0.1F; }
  };
} // namespace rpl

#endif // RPL_COMMON_HPP_