#include "rpl/io/WorldDescriptor.hpp"

// stl
#include <cstdint>

// rpl
#include "rpl/common.hpp"

void rpl::WorldDescriptor::process_obstacles(const std::vector<Polygon> &obstacles)
{
  this->d_minkowski.execute(obstacles, settings::working_envelope(), this->obstacles);
}

void rpl::WorldDescriptor::process_border(const Polygon &border)
{
  std::vector<Polygon> temp1, temp2;
  temp1.emplace_back(border);
  this->d_minkowski.execute(temp1, -settings::working_envelope(), temp2);
  this->border = std::move(temp2.front());
}
