#ifndef RPL_IO_WORLDDESCRIPTOR_HPP_
#define RPL_IO_WORLDDESCRIPTOR_HPP_

// stl
#include <vector>
// rpl
#include "rpl/types.hpp"

#include "rpl/io/Minkowski.hpp"

namespace rpl
{
  struct WorldDescriptor
  {
  public:
    std::vector<Polygon> obstacles;
    std::vector<Point>   gates;
    Polygon              border;

    void process_obstacles(const std::vector<Polygon> &obstacles);
    void process_border(const Polygon &border);

  private:
    Minkowski d_minkowski;
  };
} // namespace rpl

#endif // RPL_IO_WORLDDESCRIPTOR_HPP_