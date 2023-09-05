#ifndef RPL_INTERNAL_PARALLEL_HPP_
#define RPL_INTERNAL_PARALLEL_HPP_

#include <cmath>
#include <cstdint>
#include <limits>

#include <immintrin.h>

#include "rpl/common.hpp"

#include "rpl/internal/utils.hpp"

namespace rpl
{
  struct parallel
  {
  private:
    union f32_ps
    {
      float  f32[4];
      __m128 ps;
      explicit f32_ps(const __m128 &ps_) : ps{ps_} {}
    };

  public:
    // Constants
    static __m128 zero() { return _mm_setzero_ps(); }
    static __m128 FFFFFFFF() { return _mm_cmpeq_ps(zero(), zero()); }
    static __m128 minus() { return _mm_set1_ps(-0.f); }
    static __m128 epsilon() { return _mm_set1_ps(std::numeric_limits<float>::epsilon()); }

    // Utility
    static __m128 abs(const __m128 &value) { return _mm_andnot_ps(minus(), value); }
    static __m128 neg(const __m128 &value) { return _mm_andnot_ps(value, FFFFFFFF()); }

    // Comparison
    static __m128 cmplt(const __m128 &a, const __m128 &b)
    {
      const __m128 abs_a = abs(a);
      const __m128 abs_b = abs(b);

      __m128 mask = _mm_cmplt_ps(abs_a, abs_b);
      __m128 rhs  = _mm_add_ps(_mm_and_ps(mask, abs_b), _mm_andnot_ps(mask, abs_a));

      return _mm_cmpgt_ps(_mm_sub_ps(b, a),
                          _mm_mul_ps(rhs, epsilon()));
    }

    static __m128 cmpgt(const __m128 &a, const __m128 &b)
    {
      const __m128 abs_a = abs(a);
      const __m128 abs_b = abs(b);

      __m128 mask = _mm_cmplt_ps(abs_a, abs_b);
      __m128 rhs  = _mm_add_ps(_mm_and_ps(mask, abs_b), _mm_andnot_ps(mask, abs_a));

      return _mm_cmpgt_ps(_mm_sub_ps(a, b),
                          _mm_mul_ps(rhs, epsilon()));
    }

    static __m128 cmpge(const __m128 &a, const __m128 &b) { return neg(cmplt(a, b)); }
    static __m128 cmple(const __m128 &a, const __m128 &b) { return neg(cmpgt(a, b)); }
    static __m128 cmpeq(const __m128 &a, const __m128 &b) { return neg(_mm_or_ps(cmplt(a, b), cmpgt(a, b))); }

    // Trigonometry
    static __m128 mod2pi(const __m128 &angle)
    {
      const __m128 twopi = _mm_set1_ps(settings::M_2PI());
      __m128       temp  = _mm_div_ps(angle, twopi);
      temp               = _mm_sub_ps(angle, _mm_mul_ps(twopi, _mm_floor_ps(temp)));
      return _mm_sub_ps(temp, _mm_and_ps(twopi, cmpge(temp, twopi)));
    }

    static __m128 cos(const __m128 &angle)
    {
      f32_ps res(angle);
      res.f32[0] = cosf(res.f32[0]);
      res.f32[1] = cosf(res.f32[1]);
      res.f32[2] = cosf(res.f32[2]);
      res.f32[3] = cosf(res.f32[3]);
      return res.ps;
    }

    static __m128 sin(const __m128 &angle)
    {
      f32_ps res(angle);
      res.f32[0] = sinf(res.f32[0]);
      res.f32[1] = sinf(res.f32[1]);
      res.f32[2] = sinf(res.f32[2]);
      res.f32[3] = sinf(res.f32[3]);
      return res.ps;
    }

    static __m128 acos(const __m128 &angle)
    {
      f32_ps res(angle);
      res.f32[0] = acosf(res.f32[0]);
      res.f32[1] = acosf(res.f32[1]);
      res.f32[2] = acosf(res.f32[2]);
      res.f32[3] = acosf(res.f32[3]);
      return res.ps;
    }

    static __m128 atan2(const __m128 &y, const __m128 &x)
    {
      f32_ps res(y);
      f32_ps deltax(x);
      res.f32[0] = atan2f(res.f32[0], deltax.f32[0]);
      res.f32[1] = atan2f(res.f32[1], deltax.f32[1]);
      res.f32[2] = atan2f(res.f32[2], deltax.f32[2]);
      res.f32[3] = atan2f(res.f32[3], deltax.f32[3]);
      return res.ps;
    }
  };
} // namespace rpl

#endif // RPL_INTERNAL_PARALLEL_HPP_