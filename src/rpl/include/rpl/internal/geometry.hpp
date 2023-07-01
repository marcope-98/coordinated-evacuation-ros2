#ifndef RPL_INTERNAL_GEOMETRY_HPP_
#define RPL_INTERNAL_GEOMETRY_HPP_

#define USE_MATH_DEFINES
#include <cmath>
#include <cstdint>
#include <utility>

#include "rpl/common.hpp"
#include "rpl/types.hpp"

#include "rpl/internal/rplintrin.hpp"
#include "rpl/internal/utils.hpp"

namespace rpl
{
  struct geometry
  {
    static std::int32_t orientation(const Point &p0, const Point &p1, const Point &p2)
    {
      float res = ((p1.y - p0.y) * (p2.x - p1.x)) - ((p1.x - p0.x) * (p2.y - p1.y));
      return std::int32_t(ops::cmpgt_f32(res, 0.F)) - std::int32_t(ops::cmplt_f32(res, 0.F));
    }

    static bool intersects(const Vector &p, const Vector &q)
    {
      return (orientation(p[0], q[0], q[1]) != orientation(p[1], q[0], q[1])) &&
             (orientation(p[0], p[1], q[0]) != orientation(p[0], p[1], q[1]));
    }

    static float segment_segment_intersection(const Vector &v1, const Vector &v2)
    {
      float det = (v2[1].x - v2[0].x) * (v1[0].y - v1[1].y) -
                  (v1[0].x - v1[1].x) * (v2[1].y - v2[0].y);
      if (ops::cmpeq_f32(det, 0.F)) return 2.F;
      float det_inv = 1.F / det;
      float t       = det_inv * ((v2[0].y - v2[1].y) * (v1[0].x - v2[0].x) + (v2[1].x - v2[0].x) * (v1[0].y - v2[0].y));
      float u       = det_inv * ((v1[0].y - v1[1].y) * (v1[0].x - v2[0].x) + (v1[1].x - v1[0].x) * (v1[0].y - v2[0].y));
      if (ops::cmpgt_f32(t, 0.F) && ops::cmplt_f32(t, 1.F) && ops::cmpgt_f32(u, 0.F) && ops::cmplt_f32(u, 1.F))
        return t;
      return 2.F;
    }

    static float halfplane_intersection(const Point &origin, const Vector &vec)
    {
      float det = vec[0].y - vec[1].y;
      float u   = (vec[0].y - origin.y) / det;

      if (!ops::cmpeq_f32(det, 0.F) && ops::cmpgt_f32(u, 0.F) && ops::cmplt_f32(u, 1.F))
        return 0.5F;
      return 2.F;
    }

    static bool point_in_polygon(const Point &point, const Polygon &polygon)
    {
      std::size_t n    = polygon.size();
      std::size_t low  = 0;
      std::size_t high = n;
      std::size_t mid;

      do
      {
        mid = (low + high) >> 1;
        if (orientation(polygon[0], polygon[mid], point) == -1)
          low = mid;
        else
          high = mid;

      } while (low + 1 < high);

      if (low == 0 || high == n) return false;
      return orientation(polygon[low], polygon[high], point) == -1;
    }
  };
} // namespace rpl

#endif // RPL_INTERNAL_GEOMETRY_HPP_