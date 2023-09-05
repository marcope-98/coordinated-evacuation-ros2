#include "rpl/planning/Dubins.hpp"

#include <algorithm>
#include <cmath>

#include "rpl/internal/parallel.hpp"
#include "rpl/internal/utils.hpp"

struct rpl::Dubins::Standard
{
  Pose start;
  Pose end;
};

rpl::Dubins::Dubins()
{
  this->p_thetaf      = new float[settings::granularity()];
  this->p_sum         = new float[settings::granularity() * settings::primitives()];
  this->p_segments[0] = new float[settings::granularity() * settings::primitives()];
  this->p_segments[1] = new float[settings::granularity() * settings::primitives()];
  this->p_segments[2] = new float[settings::granularity() * settings::primitives()];

  this->d_internal.cosf   = new float[settings::granularity()];
  this->d_internal.sinf   = new float[settings::granularity()];
  this->d_internal.cos0mf = new float[settings::granularity()];
}

rpl::Dubins::~Dubins() { this->deallocate_all(); }

void rpl::Dubins::deallocate_all()
{
  delete[] this->p_thetaf;
  delete[] this->p_sum;
  delete[] this->p_segments[0];
  delete[] this->p_segments[1];
  delete[] this->p_segments[2];

  delete[] this->d_internal.cosf;
  delete[] this->d_internal.sinf;
  delete[] this->d_internal.cos0mf;
}

void rpl::Dubins::execute(const Pose &start, const Pose &end,
                          const float &min_range, const float &max_range,
                          Paths &candidates)
{
  Standard standard = {start, end};
  this->cvt_standard_to_internal(standard);
  //
  this->lsl();
  this->rsl();
  this->lsr();
  this->rsr();
  this->lrl();
  this->rlr();
  //

  this->accumulate();
  // nullify range
  this->nullify_range(min_range, max_range);

  std::size_t index;
  candidates.clear();
  for (std::size_t primitive = 0; primitive < settings::primitives(); ++primitive)
  {
    index = this->find_minimum_segment(primitive);
    candidates.emplace_back(Path{
        primitive,
        start,
        Pose{end.point(), utils::mod2pi(this->p_thetaf[index % settings::granularity()] + this->d_internal.phi)},
        this->p_segments[0][index],
        this->p_segments[1][index],
        this->p_segments[2][index],
        this->p_sum[index]});
  }

  std::sort(candidates.begin(),
            candidates.end(),
            [](const Path &lhs, const Path &rhs)
            { return lhs.sum < rhs.sum; });
}

void rpl::Dubins::nullify_range(const float &min_range, const float &max_range)
{
  if (ops::cmpeq_f32(min_range, max_range)) return;

  const __m128 inf  = _mm_set1_ps(ops::hex_to_f32(0x7F800000));
  const __m128 mmin = _mm_set1_ps(utils::mod2pi(min_range - this->d_internal.phi));
  const __m128 mmax = _mm_set1_ps(utils::mod2pi(max_range - this->d_internal.phi));

  __m128 mask;
  __m128 sum, thetaf;

  float *dst = this->p_sum;

  if (min_range < max_range)
  {
    for (std::size_t j = 0; j < settings::primitives(); ++j)
    {
      for (std::size_t i = 0; i < settings::granularity(); i += 4)
      {
        sum    = _mm_load_ps(dst + i);
        thetaf = _mm_load_ps(this->p_thetaf + i);
        // if min_range < thetaf && thetaf < max_range ==> inf
        mask = _mm_and_ps(_mm_cmplt_ps(mmin, thetaf), _mm_cmplt_ps(thetaf, mmax));
        // _mm_add_ps(mask & inf, mask & p_sum)
        _mm_store_ps(dst + i, _mm_add_ps(_mm_and_ps(mask, inf),
                                         _mm_andnot_ps(mask, sum)));
      }
      dst += settings::granularity();
    }
  }
  else
  {
    for (std::size_t j = 0; j < settings::primitives(); ++j)
    {
      for (std::size_t i = 0; i < settings::granularity(); i += 4)
      {
        sum    = _mm_load_ps(dst + i);
        thetaf = _mm_load_ps(this->p_thetaf + i);
        // if !(max_range <= thetaf && thetaf <= min_range)
        mask = _mm_and_ps(_mm_cmple_ps(mmax, thetaf), _mm_cmple_ps(thetaf, mmin));
        // _mm_add_ps(!mask & inf, mask & p_sum)
        _mm_store_ps(dst + i, _mm_add_ps(_mm_andnot_ps(mask, inf),
                                         _mm_and_ps(mask, sum)));
      }
      dst += settings::granularity();
    }
  }
}

void rpl::Dubins::accumulate()
{
  const __m128      inf  = _mm_set1_ps(ops::hex_to_f32(0x7F800000));
  const std::size_t size = settings::primitives() * settings::granularity();
  __m128            s1, s2, s3, sum, mask;
  for (std::size_t i = 0; i < size; i += 4)
  {
    s1 = _mm_load_ps(this->p_segments[0] + i);
    s2 = _mm_load_ps(this->p_segments[1] + i);
    s3 = _mm_load_ps(this->p_segments[2] + i);

    sum  = _mm_add_ps(s1, _mm_add_ps(s2, s3));
    mask = _mm_cmpeq_ps(sum, sum);

    _mm_store_ps(this->p_sum + i, _mm_add_ps(_mm_and_ps(mask, sum), _mm_andnot_ps(mask, inf)));
  }
}

void rpl::Dubins::cvt_standard_to_internal(const Standard &standard)
{
  const float dx = standard.end.x() - standard.start.x();
  const float dy = standard.end.y() - standard.start.y();

  this->d_internal.phi   = atan2f(dy, dx);
  this->d_lambda         = 0.5f * hypotf(dx, dy);
  this->d_internal.kappa = settings::kappa() * this->d_lambda;

  this->d_internal.th0  = utils::mod2pi(standard.start.theta - this->d_internal.phi);
  this->d_internal.cos0 = cosf(this->d_internal.th0);
  this->d_internal.sin0 = sinf(this->d_internal.th0);

  const __m128 th0       = _mm_set1_ps(this->d_internal.th0);
  const __m128 increment = _mm_set1_ps(4.f * settings::step());
  __m128       theta     = _mm_setr_ps(
      -this->d_internal.phi,
      -this->d_internal.phi + settings::step(),
      -this->d_internal.phi + settings::step() * 2.f,
      -this->d_internal.phi + settings::step() * 3.f);

  for (std::size_t i = 0; i < settings::granularity(); i += 4)
  {
    _mm_store_ps(this->d_internal.cosf + i, parallel::cos(theta));
    _mm_store_ps(this->d_internal.sinf + i, parallel::sin(theta));
    _mm_store_ps(this->d_internal.cos0mf + i, parallel::cos(_mm_sub_ps(th0, theta)));
    _mm_store_ps(this->p_thetaf + i, parallel::mod2pi(theta));
    theta = _mm_add_ps(theta, increment);
  }
}

std::size_t rpl::Dubins::find_minimum_segment(const std::size_t &primitive) const
{
  auto *min_element = std::min_element(this->p_sum + settings::granularity() * primitive,
                                       this->p_sum + settings::granularity() * primitive + settings::granularity());
  return std::distance(this->p_sum, min_element);
}

// ========================================== PRIMITIVES ==========================================
void rpl::Dubins::lsl()
{
  const __m128 kappa     = _mm_set1_ps(this->d_internal.kappa);
  const __m128 inv_kappa = _mm_set1_ps(this->d_lambda / this->d_internal.kappa);
  const __m128 cos0      = _mm_set1_ps(this->d_internal.cos0);
  const __m128 sin0      = _mm_set1_ps(this->d_internal.sin0);
  const __m128 th0       = _mm_set1_ps(this->d_internal.th0);
  const __m128 two       = _mm_set1_ps(2.f);
  const __m128 four      = _mm_set1_ps(4.f);
  const __m128 constant  = _mm_set1_ps(2.f + 4.f * this->d_internal.kappa * (this->d_internal.kappa + this->d_internal.sin0));

  __m128 C, S, s1, s2, s3;
  __m128 cosf, sinf, thetaf, atan2_CS, cos0mf;

  float *dsts1 = this->p_segments[0];
  float *dsts2 = this->p_segments[1];
  float *dsts3 = this->p_segments[2];

  for (std::size_t i = 0; i < settings::granularity(); i += 4)
  {
    thetaf = _mm_load_ps(this->p_thetaf + i);
    cosf   = _mm_load_ps(this->d_internal.cosf + i);
    sinf   = _mm_load_ps(this->d_internal.sinf + i);
    cos0mf = _mm_load_ps(this->d_internal.cos0mf + i);

    C        = _mm_sub_ps(cosf, cos0);
    S        = _mm_add_ps(_mm_mul_ps(two, kappa), _mm_sub_ps(sin0, sinf));
    atan2_CS = parallel::atan2(C, S);

    s1 = parallel::mod2pi(_mm_sub_ps(atan2_CS, th0));
    s2 = _mm_sqrt_ps(_mm_sub_ps(constant, _mm_add_ps(
                                              _mm_mul_ps(two, cos0mf),
                                              _mm_mul_ps(four, _mm_mul_ps(kappa, sinf)))));
    s3 = parallel::mod2pi(_mm_sub_ps(thetaf, atan2_CS));

    _mm_store_ps(dsts1 + i, _mm_mul_ps(s1, inv_kappa));
    _mm_store_ps(dsts2 + i, _mm_mul_ps(s2, inv_kappa));
    _mm_store_ps(dsts3 + i, _mm_mul_ps(s3, inv_kappa));
  }
}

void rpl::Dubins::rsr()
{
  const __m128 kappa     = _mm_set1_ps(this->d_internal.kappa);
  const __m128 inv_kappa = _mm_set1_ps(this->d_lambda / this->d_internal.kappa);
  const __m128 cos0      = _mm_set1_ps(this->d_internal.cos0);
  const __m128 sin0      = _mm_set1_ps(this->d_internal.sin0);
  const __m128 th0       = _mm_set1_ps(this->d_internal.th0);
  const __m128 two       = _mm_set1_ps(2.f);
  const __m128 four      = _mm_set1_ps(4.f);
  const __m128 constant  = _mm_set1_ps(2.f + 4.f * this->d_internal.kappa * (this->d_internal.kappa - this->d_internal.sin0));

  __m128 C, S, s1, s2, s3;
  __m128 cosf, sinf, thetaf, atan2_CS, cos0mf;

  float *dsts1 = this->p_segments[0] + settings::granularity();
  float *dsts2 = this->p_segments[1] + settings::granularity();
  float *dsts3 = this->p_segments[2] + settings::granularity();

  for (std::size_t i = 0; i < settings::granularity(); i += 4)
  {
    thetaf = _mm_load_ps(this->p_thetaf + i);
    cosf   = _mm_load_ps(this->d_internal.cosf + i);
    sinf   = _mm_load_ps(this->d_internal.sinf + i);
    cos0mf = _mm_load_ps(this->d_internal.cos0mf + i);

    C        = _mm_sub_ps(cos0, cosf);
    S        = _mm_add_ps(_mm_mul_ps(two, kappa), _mm_sub_ps(sinf, sin0));
    atan2_CS = parallel::atan2(C, S);

    s1 = parallel::mod2pi(_mm_sub_ps(th0, atan2_CS));
    s2 = _mm_sqrt_ps(_mm_add_ps(constant,
                                _mm_sub_ps(
                                    _mm_mul_ps(four, _mm_mul_ps(kappa, sinf)),
                                    _mm_mul_ps(two, cos0mf))));
    s3 = parallel::mod2pi(_mm_sub_ps(atan2_CS, thetaf));

    _mm_store_ps(dsts1 + i, _mm_mul_ps(s1, inv_kappa));
    _mm_store_ps(dsts2 + i, _mm_mul_ps(s2, inv_kappa));
    _mm_store_ps(dsts3 + i, _mm_mul_ps(s3, inv_kappa));
  }
}

void rpl::Dubins::lsr()
{
  const __m128 kappa     = _mm_set1_ps(this->d_internal.kappa);
  const __m128 inv_kappa = _mm_set1_ps(this->d_lambda / this->d_internal.kappa);
  const __m128 cos0      = _mm_set1_ps(this->d_internal.cos0);
  const __m128 sin0      = _mm_set1_ps(this->d_internal.sin0);
  const __m128 th0       = _mm_set1_ps(this->d_internal.th0);
  const __m128 two       = _mm_set1_ps(2.f);
  const __m128 four      = _mm_set1_ps(4.f);
  const __m128 zero      = _mm_setzero_ps();
  const __m128 constant  = _mm_set1_ps(-2.f + 4.f * this->d_internal.kappa * (this->d_internal.kappa + this->d_internal.sin0));

  __m128 C, S, s1, s2, s3;
  __m128 cosf, sinf, thetaf, atan2_CS, atan2_2s2, cos0mf;

  float *dsts1 = this->p_segments[0] + 2 * settings::granularity();
  float *dsts2 = this->p_segments[1] + 2 * settings::granularity();
  float *dsts3 = this->p_segments[2] + 2 * settings::granularity();

  for (std::size_t i = 0; i < settings::granularity(); i += 4)
  {
    thetaf = _mm_load_ps(this->p_thetaf + i);
    cosf   = _mm_load_ps(this->d_internal.cosf + i);
    sinf   = _mm_load_ps(this->d_internal.sinf + i);
    cos0mf = _mm_load_ps(this->d_internal.cos0mf + i);

    C        = _mm_add_ps(cos0, cosf);
    S        = _mm_add_ps(_mm_mul_ps(two, kappa), _mm_add_ps(sin0, sinf));
    atan2_CS = parallel::atan2(_mm_sub_ps(zero, C), S);

    s2        = _mm_sqrt_ps(_mm_add_ps(constant,
                                _mm_add_ps(
                                    _mm_mul_ps(two, cos0mf),
                                    _mm_mul_ps(four, _mm_mul_ps(kappa, sinf)))));
    atan2_2s2 = _mm_sub_ps(atan2_CS, parallel::atan2(_mm_sub_ps(zero, two), s2));
    s1        = parallel::mod2pi(_mm_sub_ps(atan2_2s2, th0));
    s3        = parallel::mod2pi(_mm_sub_ps(atan2_2s2, thetaf));

    _mm_store_ps(dsts1 + i, _mm_mul_ps(s1, inv_kappa));
    _mm_store_ps(dsts2 + i, _mm_mul_ps(s2, inv_kappa));
    _mm_store_ps(dsts3 + i, _mm_mul_ps(s3, inv_kappa));
  }
}

void rpl::Dubins::rsl()
{
  const __m128 kappa     = _mm_set1_ps(this->d_internal.kappa);
  const __m128 inv_kappa = _mm_set1_ps(this->d_lambda / this->d_internal.kappa);
  const __m128 cos0      = _mm_set1_ps(this->d_internal.cos0);
  const __m128 sin0      = _mm_set1_ps(this->d_internal.sin0);
  const __m128 th0       = _mm_set1_ps(this->d_internal.th0);
  const __m128 two       = _mm_set1_ps(2.f);
  const __m128 four      = _mm_set1_ps(4.f);
  const __m128 constant  = _mm_set1_ps(-2.f + 4.f * this->d_internal.kappa * (this->d_internal.kappa - this->d_internal.sin0));

  __m128 C, S, s1, s2, s3;
  __m128 cosf, sinf, thetaf, atan2_CS, atan2_2s2, cos0mf;

  float *dsts1 = this->p_segments[0] + 3 * settings::granularity();
  float *dsts2 = this->p_segments[1] + 3 * settings::granularity();
  float *dsts3 = this->p_segments[2] + 3 * settings::granularity();

  for (std::size_t i = 0; i < settings::granularity(); i += 4)
  {
    thetaf = _mm_load_ps(this->p_thetaf + i);
    cosf   = _mm_load_ps(this->d_internal.cosf + i);
    sinf   = _mm_load_ps(this->d_internal.sinf + i);
    cos0mf = _mm_load_ps(this->d_internal.cos0mf + i);

    C        = _mm_add_ps(cos0, cosf);
    S        = _mm_sub_ps(_mm_mul_ps(two, kappa), _mm_add_ps(sin0, sinf));
    atan2_CS = parallel::atan2(C, S);

    s2 = _mm_sqrt_ps(_mm_add_ps(constant,
                                _mm_sub_ps(
                                    _mm_mul_ps(two, cos0mf),
                                    _mm_mul_ps(four, _mm_mul_ps(kappa, sinf)))));

    atan2_2s2 = _mm_sub_ps(parallel::atan2(two, s2), atan2_CS);
    s1        = parallel::mod2pi(_mm_add_ps(th0, atan2_2s2));
    s3        = parallel::mod2pi(_mm_add_ps(thetaf, atan2_2s2));

    _mm_store_ps(dsts1 + i, _mm_mul_ps(s1, inv_kappa));
    _mm_store_ps(dsts2 + i, _mm_mul_ps(s2, inv_kappa));
    _mm_store_ps(dsts3 + i, _mm_mul_ps(s3, inv_kappa));
  }
}

void rpl::Dubins::rlr()
{
  const __m128 half       = _mm_set1_ps(0.5f);
  const __m128 two        = _mm_set1_ps(2.f);
  const __m128 four       = _mm_set1_ps(4.f);
  const __m128 one_over_8 = _mm_set1_ps(0.125f);
  const __m128 twopi      = _mm_set1_ps(settings::M_2PI());
  const __m128 kappa      = _mm_set1_ps(this->d_internal.kappa);
  const __m128 inv_kappa  = _mm_set1_ps(this->d_lambda / this->d_internal.kappa);
  const __m128 th0        = _mm_set1_ps(this->d_internal.th0);
  const __m128 sin0       = _mm_set1_ps(this->d_internal.sin0);
  const __m128 cos0       = _mm_set1_ps(this->d_internal.cos0);
  const __m128 constant   = _mm_set1_ps(6.f + 4.f * this->d_internal.kappa * (this->d_internal.sin0 - this->d_internal.kappa));

  __m128 C, S, s1, s2, s3;
  __m128 cosf, sinf, thetaf, atan2_CS, cos0mf;

  float *dsts1 = this->p_segments[0] + 4 * settings::granularity();
  float *dsts2 = this->p_segments[1] + 4 * settings::granularity();
  float *dsts3 = this->p_segments[2] + 4 * settings::granularity();

  for (std::size_t i = 0; i < settings::granularity(); i += 4)
  {
    thetaf = _mm_load_ps(this->p_thetaf + i);
    cosf   = _mm_load_ps(this->d_internal.cosf + i);
    sinf   = _mm_load_ps(this->d_internal.sinf + i);
    cos0mf = _mm_load_ps(this->d_internal.cos0mf + i);

    C        = _mm_sub_ps(cos0, cosf);
    S        = _mm_add_ps(_mm_mul_ps(two, kappa), _mm_sub_ps(sinf, sin0));
    atan2_CS = parallel::atan2(C, S);

    s2 = parallel::mod2pi(
        _mm_sub_ps(twopi, parallel::acos(
                              _mm_mul_ps(one_over_8,
                                         _mm_add_ps(constant,
                                                    _mm_sub_ps(_mm_mul_ps(two, cos0mf),
                                                               _mm_mul_ps(four, _mm_mul_ps(kappa, sinf))))))));

    s1 = parallel::mod2pi(_mm_add_ps(_mm_mul_ps(half, s2), _mm_sub_ps(th0, atan2_CS)));
    s3 = parallel::mod2pi(_mm_add_ps(_mm_sub_ps(s2, s1), _mm_sub_ps(th0, thetaf)));

    _mm_store_ps(dsts1 + i, _mm_mul_ps(s1, inv_kappa));
    _mm_store_ps(dsts2 + i, _mm_mul_ps(s2, inv_kappa));
    _mm_store_ps(dsts3 + i, _mm_mul_ps(s3, inv_kappa));
  }
}

void rpl::Dubins::lrl()
{
  const __m128 half       = _mm_set1_ps(0.5f);
  const __m128 two        = _mm_set1_ps(2.f);
  const __m128 four       = _mm_set1_ps(4.f);
  const __m128 one_over_8 = _mm_set1_ps(0.125f);
  const __m128 twopi      = _mm_set1_ps(settings::M_2PI());
  const __m128 kappa      = _mm_set1_ps(this->d_internal.kappa);
  const __m128 inv_kappa  = _mm_set1_ps(this->d_lambda / this->d_internal.kappa);
  const __m128 th0        = _mm_set1_ps(this->d_internal.th0);
  const __m128 sin0       = _mm_set1_ps(this->d_internal.sin0);
  const __m128 cos0       = _mm_set1_ps(this->d_internal.cos0);
  const __m128 constant   = _mm_set1_ps(6.f - 4.f * this->d_internal.kappa * (this->d_internal.kappa + this->d_internal.sin0));
  __m128       C, S, s1, s2, s3;
  __m128       cosf, sinf, thetaf, atan2_CS, cos0mf;

  float *dsts1 = this->p_segments[0] + 5 * settings::granularity();
  float *dsts2 = this->p_segments[1] + 5 * settings::granularity();
  float *dsts3 = this->p_segments[2] + 5 * settings::granularity();

  for (std::size_t i = 0; i < settings::granularity(); i += 4)
  {
    thetaf = _mm_load_ps(this->p_thetaf + i);
    cosf   = _mm_load_ps(this->d_internal.cosf + i);
    sinf   = _mm_load_ps(this->d_internal.sinf + i);
    cos0mf = _mm_load_ps(this->d_internal.cos0mf + i);

    C        = _mm_sub_ps(cosf, cos0);
    S        = _mm_add_ps(_mm_mul_ps(two, kappa), _mm_sub_ps(sin0, sinf));
    atan2_CS = parallel::atan2(C, S);

    s2 = parallel::mod2pi(_mm_sub_ps(twopi, parallel::acos(
                                                _mm_mul_ps(one_over_8,
                                                           _mm_add_ps(constant,
                                                                      _mm_add_ps(_mm_mul_ps(two, cos0mf),
                                                                                 _mm_mul_ps(four, _mm_mul_ps(kappa, sinf))))))));

    s1 = parallel::mod2pi(_mm_add_ps(atan2_CS, _mm_sub_ps(_mm_mul_ps(half, s2), th0)));
    s3 = parallel::mod2pi(_mm_add_ps(_mm_sub_ps(thetaf, th0), _mm_sub_ps(s2, s1)));

    _mm_store_ps(dsts1 + i, _mm_mul_ps(s1, inv_kappa));
    _mm_store_ps(dsts2 + i, _mm_mul_ps(s2, inv_kappa));
    _mm_store_ps(dsts3 + i, _mm_mul_ps(s3, inv_kappa));
  }
}