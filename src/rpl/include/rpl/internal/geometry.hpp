#ifndef RPL_INTERNAL_GEOMETRY_HPP_
#define RPL_INTERNAL_GEOMETRY_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>

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
      float res = ((p1.y() - p0.y()) * (p2.x() - p1.x())) - ((p1.x() - p0.x()) * (p2.y() - p1.y()));
      return std::int32_t(ops::cmpgt_f32(res, 0.f)) - std::int32_t(ops::cmplt_f32(res, 0.f));
    }

    static bool intersects(const Point &p0, const Point &p1,
                           const Point &q0, const Point &q1)
    {
      return (orientation(p0, q0, q1) != orientation(p1, q0, q1)) &&
             (orientation(p0, p1, q0) != orientation(p0, p1, q1));
    }

    static float segment_segment_intersection(const Point &p0, const Point &p1,
                                              const Point &q0, const Point &q1)
    {
      float det = (q1.x() - q0.x()) * (p0.y() - p1.y()) - (p0.x() - p1.x()) * (q1.y() - q0.y());
      if (ops::cmpeq_f32(det, 0.f)) return 2.f;
      float det_inv = 1.f / det;
      float t       = det_inv * ((q0.y() - q1.y()) * (p0.x() - q0.x()) + (q1.x() - q0.x()) * (p0.y() - q0.y()));
      float u       = det_inv * ((p0.y() - p1.y()) * (p0.x() - q0.x()) + (p1.x() - p0.x()) * (p0.y() - q0.y()));
      if (ops::cmpgt_f32(t, 0.f) && ops::cmplt_f32(t, 1.f) &&
          ops::cmpgt_f32(u, 0.f) && ops::cmplt_f32(u, 1.f))
        return t;
      return 2.f;
    }

    static bool arc_segment_intersection(const Circle &circle,
                                         const Point &p, const Point &q,
                                         const float &min_range, const float &max_range)
    {
      Point       diff_qp = q - p;
      Point       diff_pc = p - circle.center;
      Point       temp    = diff_qp * diff_pc;
      const float a       = diff_qp.norm2();
      const float b       = temp.x() + temp.y();
      const float c       = diff_pc.norm2() - (circle.radius * circle.radius);

      const float Delta = sqrtf(b * b - a * c);
      if (std::isnan(Delta)) return false; // no intersection

      float t1 = (-b - Delta) / a;
      float t2 = (-b + Delta) / a;

      // find t1point, t2point
      Point diff    = q - p;
      float angle   = utils::mod2pi(atan2f(diff.y(), diff.x()));
      Point t1point = (diff_pc + t1 * Point{sinf(angle), cosf(angle)});
      Point t2point = (diff_pc + t2 * Point{sinf(angle), cosf(angle)});

      // find angle of intersection atan2(t1point - circle.center)
      float anglet1 = utils::mod2pi(atan2f(t1point.y(), t1point.x()));
      float anglet2 = utils::mod2pi(atan2f(t2point.y(), t2point.x()));

      // check range
      bool condition1, condition2;
      if (min_range <= max_range)
      {
        condition1 = (ops::cmple_f32(min_range, anglet1) && ops::cmple_f32(anglet1, max_range));
        condition2 = (ops::cmple_f32(min_range, anglet2) && ops::cmple_f32(anglet2, max_range));
      }
      else
      {
        // HACK: this feels weird
        condition1 = !(ops::cmplt_f32(max_range, anglet1) && ops::cmplt_f32(anglet1, min_range));
        condition2 = !(ops::cmplt_f32(max_range, anglet2) && ops::cmplt_f32(anglet2, min_range));
      }

      // if either conditions return true then an intersection occurs
      return condition1 || condition2;
    }

    static float halfplane_intersection(const Point &origin,
                                        const Point &p, const Point &q)
    {
      float det = p.y() - q.y();
      float u   = (p.y() - origin.y()) / det;

      if (!ops::cmpeq_f32(det, 0.f) && ops::cmpgt_f32(u, 0.f) && ops::cmplt_f32(u, 1.f))
        return 0.5F;
      return 2.f;
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

    static float norm(const Point &p, const Point &q)
    {
      const float dx = q.x() - p.x();
      const float dy = q.y() - p.y();
      return sqrtf(dx * dx + dy * dy);
    }

    static Point centroid(const Polygon &polygon)
    {
      Point res;
      for (const auto &point : polygon)
      {
        res.x() += point.x();
        res.y() += point.y();
      }
      res.x() /= float(polygon.size());
      res.y() /= float(polygon.size());
      return res;
    }

    static float max_distance(const Point &point, const Polygon &polygon)
    {
      std::vector<float> distances;
      distances.reserve(polygon.size());
      for (const auto &vertex : polygon)
        distances.emplace_back(norm(point, vertex));
      return *std::max_element(distances.begin(), distances.end());
    }
  };
} // namespace rpl

#endif // RPL_INTERNAL_GEOMETRY_HPP_