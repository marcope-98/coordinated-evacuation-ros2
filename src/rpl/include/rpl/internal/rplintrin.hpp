#ifndef RPL_INTERNAL_RPLINTRIN_HPP_
#define RPL_INTERNAL_RPLINTRIN_HPP_

#include <cstdint>
#include <limits>

namespace rpl
{
  struct ops
  {
  private:
    union cvt
    {
      float         f32;
      std::uint32_t u32;
      explicit cvt(const float &a) : f32{a} {}
    };

  public:
    static float hex_to_f32(const std::uint32_t &hex)
    {
      cvt temp(0.f);
      temp.u32 = hex;
      return temp.f32;
    }

    static float abs_f32(const float &value)
    {
      cvt a(value);
      a.u32 &= (0x7FFFFFFFU);
      return a.f32;
    }

    static bool cmpgt_f32(const float &a, const float &b)
    {
      const float epsilon = std::numeric_limits<float>::epsilon();
      const float fabs_a  = abs_f32(a);
      const float fabs_b  = abs_f32(b);
      return (a - b) > ((fabs_a < fabs_b ? fabs_b : fabs_a) * epsilon);
    }

    static bool cmplt_f32(const float &a, const float &b)
    {
      const float epsilon = std::numeric_limits<float>::epsilon();
      const float fabs_a  = abs_f32(a);
      const float fabs_b  = abs_f32(b);
      return (b - a) > ((fabs_a < fabs_b ? fabs_b : fabs_a) * epsilon);
    }

    static bool cmpge_f32(const float &a, const float &b) { return !cmplt_f32(a, b); }
    static bool cmple_f32(const float &a, const float &b) { return !cmpgt_f32(a, b); }
    static bool cmpeq_f32(const float &a, const float &b) { return !(cmplt_f32(a, b) || cmpgt_f32(a, b)); }
  };
} // namespace rpl

#endif // RPL_INTERNAL_RPLINTRIN_HPP_