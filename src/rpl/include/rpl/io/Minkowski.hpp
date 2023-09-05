#ifndef RPL_IO_MINKOWSKI_HPP_
#define RPL_IO_MINKOWSKI_HPP_

// stl
#include <vector>
// rpl
#include "rpl/types.hpp"
// rpl/clipper2
#include "rpl/clipper2/clipper.h"

namespace rpl
{
  struct Minkowski
  {
  private:
    Clipper2Lib::ClipperOffset d_clipperOffset;
    Clipper2Lib::Paths64       d_paths;

  public:
    void execute(const std::vector<Polygon> &polygons, const float &offset,
                 std::vector<Polygon> &out);

  private:
    void cvt_polygon_to_paths(const Polygon &polygon);
    void cvt_paths_to_polygon(std::vector<Polygon> &out);
    void inflate(const std::int32_t &offset);
    void reset();
  };
} // namespace rpl

#endif // RPL_IO_MINKOWSKI_HPP_