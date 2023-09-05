#include "rpl/io/WorldDescriptor.hpp"
#include <cstdint>
#include <iostream>
#include <utility>

#include "rpl/common.hpp"
#include "rpl/internal/geometry.hpp"

void rpl::WorldDescriptor::process_obstacles(const std::vector<Polygon> &obstacles)
{
  this->obstacles_inner.clear();
  this->obstacles_outer.clear();
  this->d_minkowski.execute(obstacles, settings::working_envelope_inner(), this->obstacles_inner);
  this->d_minkowski.execute(obstacles, settings::working_envelope_outer(), this->obstacles_outer);
}

void rpl::WorldDescriptor::process_border(const Polygon &border)
{
  std::vector<Polygon> temp1, temp2;

  this->border_inner.clear();
  temp1.emplace_back(border);
  this->d_minkowski.execute(temp1, -settings::working_envelope_inner(), temp2);
  this->border_inner = std::move(temp2.front());

  temp1.clear();
  temp2.clear();

  this->border_outer.clear();
  temp1.emplace_back(border);
  this->d_minkowski.execute(temp1, -settings::working_envelope_outer(), temp2);
  this->border_outer = std::move(temp2.front());
}
