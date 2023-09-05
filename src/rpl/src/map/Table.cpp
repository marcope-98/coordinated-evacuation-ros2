#include "rpl/map/Table.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>

#include <immintrin.h>

#include "rpl/internal/utils.hpp"

rpl::Table::Table(const std::size_t &N) : Table() { this->reserve(N); }

rpl::Table::~Table() { this->deallocate_all(); }

void rpl::Table::deallocate_all()
{
  this->d_size     = 0;
  this->d_capacity = 0;
  delete[] this->p_x;
  delete[] this->p_y;
  delete[] this->p_prev;
  delete[] this->p_next;
}

void rpl::Table::emplace_back(const Point &point)
{
  if (this->capacity() == this->size()) this->reallocate(this->size() + 1);

  const std::size_t idx = this->size();
  this->p_prev[idx]     = (std::uint32_t)idx;
  this->p_next[idx]     = (std::uint32_t)idx;
  this->p_x[idx]        = point.x();
  this->p_y[idx]        = point.y();
  ++this->d_size;
}

void rpl::Table::emplace_back(const Polygon &polygon)
{
  if (this->capacity() < this->size() + polygon.size()) this->reallocate(this->size() + polygon.size());

  std::size_t i      = this->size();
  std::size_t first  = i;
  std::size_t second = i + polygon.size() - 1;
  for (const auto &point : polygon)
  {
    this->p_x[i]    = point.x();
    this->p_y[i]    = point.y();
    this->p_prev[i] = std::uint32_t(i) - 1;
    this->p_next[i] = std::uint32_t(i) + 1;
    ++i;
  }
  this->p_prev[first]  = (std::uint32_t)second;
  this->p_next[second] = (std::uint32_t)first;
  this->d_size += polygon.size();
}

void rpl::Table::emplace_back(const Point &point, const std::uint32_t &prev, const std::uint32_t &next)
{
  if (this->capacity() == this->size()) this->reallocate(this->size() + 1);

  const std::size_t idx = this->size();
  this->p_prev[idx]     = prev;
  this->p_next[idx]     = next;
  this->p_x[idx]        = point.x();
  this->p_y[idx]        = point.y();
  ++this->d_size;
}

void rpl::Table::reallocate(const std::size_t &size)
{
  const std::size_t capacity  = this->align_up(size);
  float *           temp_x    = new float[capacity]();
  float *           temp_y    = new float[capacity]();
  std::uint32_t *   temp_prev = new std::uint32_t[capacity]();
  std::uint32_t *   temp_next = new std::uint32_t[capacity]();

  for (std::size_t i = 0; i < this->d_size; i += 4)
  {
    _mm_store_ps(temp_x + i, _mm_load_ps(this->p_x + i));
    _mm_store_ps(temp_y + i, _mm_load_ps(this->p_y + i));
    _mm_store_si128((__m128i *)(temp_prev + i), _mm_load_si128((__m128i *)(this->p_prev + i)));
    _mm_store_si128((__m128i *)(temp_next + i), _mm_load_si128((__m128i *)(this->p_next + i)));
  }

  std::swap(this->p_x, temp_x);
  std::swap(this->p_y, temp_y);
  std::swap(this->p_prev, temp_prev);
  std::swap(this->p_next, temp_next);
  this->d_capacity = capacity;
  delete[] temp_x;
  delete[] temp_y;
  delete[] temp_prev;
  delete[] temp_next;
}

rpl::Table &rpl::Table::operator=(const Table &other)
{
  this->reserve(other.capacity());
  this->d_size = other.d_size;

  for (std::size_t i = 0; i < this->d_size; i += 4)
  {
    _mm_store_ps(this->p_x + i, _mm_load_ps(other.p_x + i));
    _mm_store_ps(this->p_y + i, _mm_load_ps(other.p_y + i));
    _mm_store_si128((__m128i *)(this->p_prev + i), _mm_load_si128((__m128i *)(other.p_prev + i)));
    _mm_store_si128((__m128i *)(this->p_next + i), _mm_load_si128((__m128i *)(other.p_next + i)));
  }

  this->d_capacity = other.capacity();
  return *this;
}

rpl::Table &rpl::Table::operator=(Table &&other) noexcept
{
  this->d_size     = other.d_size;
  this->d_capacity = other.d_capacity;
  std::swap(this->p_prev, other.p_prev);
  std::swap(this->p_next, other.p_next);
  std::swap(this->p_x, other.p_x);
  std::swap(this->p_y, other.p_y);
  return *this;
}

void rpl::Table::angular_sort(const std::size_t &v, std::vector<std::size_t> &sorted_vertices) const
{
  const std::size_t  size = this->d_size;
  std::vector<float> theta, rho;
  theta.reserve(size);
  rho.reserve(size);

  const Point ref = this->point(v);
  float       dx, dy;
  for (std::size_t i = 0; i < size; ++i)
  {
    dx = this->p_x[i] - ref.x();
    dy = this->p_y[i] - ref.y();
    rho.emplace_back(dx * dx + dy * dy);
    theta.emplace_back(utils::mod2pi(atan2f(dy, dx)));
  }
  theta[v] = 8.f;

  sorted_vertices.resize(size);
  std::iota(sorted_vertices.begin(), sorted_vertices.end(), 0);
  std::sort(sorted_vertices.begin(), sorted_vertices.end(), [&](const std::size_t &a, const std::size_t &b)
            { return (theta[a] < theta[b]) ||
                     ((theta[a] == theta[b]) && (rho[a] < rho[b])); });
}

bool rpl::Table::polygon_search(const std::size_t &v, const std::size_t &wi) const
{
  std::size_t vertex = this->next(wi);

  while (vertex != wi)
  {
    if (vertex == v) return true;
    vertex = this->next(vertex);
  }
  return false;
}

void rpl::Table::print() const
{
  std::cerr << this->size() << " " << this->capacity() << "\n";
  for (std::size_t i = 0; i < this->size(); ++i)
    std::cerr << this->p_x[i] << " " << this->p_y[i] << " " << this->p_next[i] << " " << this->p_prev[i] << "\n";
  std::cerr << "\n\n";
};

bool rpl::Table::in_polygon(const std::size_t &v, const std::size_t &wi) const
{
  std::size_t next = this->next(wi);
  std::size_t prev = this->prev(wi);
  // 2 cases: v and wi belong to the same polygon, v and wi do not belong to the same polygon

  if (this->polygon_search(v, wi))
  {
    if (prev == v || next == v) return false;
    bool one_left = false, one_right = false;
    for (std::size_t edge = next; edge != v; edge = this->next(edge))
    {
      if (this->orientation(wi, v, edge) == 1)
      {
        one_right = true;
        break;
      }
    }

    for (std::size_t edge = prev; edge != v; edge = this->prev(edge))
    {
      if (this->orientation(wi, v, edge) == -1)
      {
        one_left = true;
        break;
      }
    }
    return one_left && one_right;
  }

  return false;
}

void rpl::Table::reserve(const std::size_t &N)
{
  this->deallocate_all();
  const std::size_t capacity = this->align_up(N);
  this->p_prev               = new std::uint32_t[capacity]();
  this->p_next               = new std::uint32_t[capacity]();
  this->p_x                  = new float[capacity]();
  this->p_y                  = new float[capacity]();
  this->d_capacity           = capacity;
}