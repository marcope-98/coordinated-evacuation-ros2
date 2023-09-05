#ifndef RPL_PLANNING_DUBINS_HPP_
#define RPL_PLANNING_DUBINS_HPP_

#include <cstdint>
#include <vector>

#include "rpl/common.hpp"
#include "rpl/types.hpp"

namespace rpl
{
  struct Dubins
  {
  private:
    struct Standard;
    struct Internal
    {
      float phi{0.f};
      float kappa{0.f};
      //
      float  th0{0.f};
      float  cos0{0.f};
      float  sin0{0.f};
      float *cosf{nullptr};
      float *sinf{nullptr};
      float *cos0mf{nullptr};
    };

  public:
    Dubins();
    Dubins(const Dubins &)     = delete;
    Dubins(Dubins &&) noexcept = delete;
    ~Dubins();

    Dubins &operator=(const Dubins &) = delete;
    Dubins &operator=(Dubins &&) noexcept = delete;

    void execute(const Pose &start, const Pose &end,
                 const float &min_range, const float &max_range,
                 Paths &candidates);

  private:
    void        deallocate_all();
    void        accumulate();
    void        nullify_range(const float &min_range, const float &max_range);
    void        cvt_standard_to_internal(const Standard &standard);
    std::size_t find_minimum_segment(const std::size_t &primitive) const;

    void lsl();
    void rsr();
    void lsr();
    void rsl();
    void lrl();
    void rlr();

  private:
    float  d_lambda{0.f};
    float *p_thetaf{nullptr};
    float *p_sum{nullptr};
    float *p_segments[3] = {nullptr, nullptr, nullptr};

    Internal d_internal;
  };
} // namespace rpl

#endif // RPL_PLANNING_DUBINS_HPP_
