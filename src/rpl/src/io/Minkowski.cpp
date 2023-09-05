#include "rpl/io/Minkowski.hpp"
#include <algorithm>
#include <iostream>
// rpl
#include "rpl/common.hpp"

void rpl::Minkowski::execute(const std::vector<Polygon> &polygons, const float &offset,
                             std::vector<Polygon> &out)
{
  out.clear();
  this->reset();

  const auto int_offset = std::int32_t(offset * settings::exp_factor());

  for (const auto &polygon : polygons)
    this->cvt_polygon_to_paths(polygon);

  this->inflate(int_offset);
  this->cvt_paths_to_polygon(out);
}

void rpl::Minkowski::cvt_polygon_to_paths(const Polygon &polygon)
{
  Clipper2Lib::Path64 path;
  path.reserve(settings::max_polygon_size());
  for (const auto &point : polygon)
    path.emplace_back(Clipper2Lib::Point64(std::int64_t(point.x() * settings::exp_factor()),
                                           std::int64_t(point.y() * settings::exp_factor())));
  this->d_paths.emplace_back(path);
}

void rpl::Minkowski::inflate(const std::int32_t &offset)
{
  this->d_paths = Clipper2Lib::InflatePaths(this->d_paths,
                                            offset,
                                            Clipper2Lib::JoinType::Miter,
                                            Clipper2Lib::EndType::Polygon,
                                            10);
}

void rpl::Minkowski::reset()
{
  this->d_clipperOffset.Clear();
  this->d_paths.clear();
}

void rpl::Minkowski::cvt_paths_to_polygon(std::vector<Polygon> &out)
{
  rpl::Polygon polygon;
  polygon.reserve(settings::max_polygon_size());
  out.reserve(this->d_paths.size());
  for (auto &path : this->d_paths)
  {
    if (!Clipper2Lib::IsPositive(path))
      std::reverse(path.begin(), path.end());

    for (const auto &intpoint : path)
      polygon.emplace_back(Point{float(intpoint.x) * settings::iexp_factor(),
                                 float(intpoint.y) * settings::iexp_factor()});
    out.emplace_back(polygon);
    polygon.clear();
  }
}
