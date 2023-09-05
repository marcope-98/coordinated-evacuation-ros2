#include "rpl/map/Graph.hpp"

#include <bitset>
#include <cmath>
#include <iomanip>
#include <iostream>

#include <immintrin.h>

rpl::Graph::Graph(const std::size_t &N) : d_nodes{N}
{
  this->p_data = new std::uint8_t[this->capacity()]();
}

rpl::Graph::~Graph() { this->deallocate_all(); }

rpl::Graph &rpl::Graph::operator=(const Graph &other)
{
  if (this->capacity() < other.capacity()) this->reallocate(other.capacity());
  this->d_nodes = other.d_nodes;
  this->fast_copy(other.p_data, this->p_data);
  return *this;
}

rpl::Graph &rpl::Graph::operator=(Graph &&other) noexcept
{
  this->d_nodes = other.d_nodes;
  std::swap(this->p_data, other.p_data);
  return *this;
}

bool rpl::Graph::operator()(const std::size_t &n1, const std::size_t &n2) const
{
  const std::size_t div   = n2 >> 3u;
  const std::size_t rem   = n2 - (div << 3u);
  const std::size_t index = n1 * this->width() + div;
  return bool(this->p_data[index] & (0x80u >> rem));
}

void rpl::Graph::add_edge(const std::size_t &n1, const std::size_t &n2)
{
  if (n1 == n2) return;
  this->add_halfedge(n1, n2);
  this->add_halfedge(n2, n1);
}

void rpl::Graph::add_halfedge(const std::size_t &n1, const std::size_t &n2)
{
  const std::size_t div   = n2 >> 3u;
  const std::size_t rem   = n2 - (div << 3u);
  const std::size_t index = n1 * this->width() + div;
  this->p_data[index] |= (0x80u >> rem);
}

void rpl::Graph::rm_node(const std::size_t &node)
{
  const std::size_t s = this->height();
  for (std::size_t i = 0; i < s; ++i)
    this->rm_edge(node, i);
}

void rpl::Graph::rm_edge(const std::size_t &n1, const std::size_t &n2)
{
  this->rm_halfedge(n1, n2);
  this->rm_halfedge(n2, n1);
}

void rpl::Graph::rm_halfedge(const std::size_t &n1, const std::size_t &n2)
{
  const std::size_t div   = n2 >> 3u;
  const std::size_t rem   = n2 - (div << 3u);
  const std::size_t index = n1 * this->width() + div;
  this->p_data[index] &= ~(0x80u >> rem);
}

void rpl::Graph::deallocate_all()
{
  this->d_nodes = 0;
  delete[] this->p_data;
}

void rpl::Graph::reallocate(const std::size_t &capacity)
{
  this->deallocate_all();
  this->p_data = new std::uint8_t[capacity]();
}

void rpl::Graph::fast_copy(const std::uint8_t *const src, std::uint8_t *const dst)
{
  const std::size_t c_over_16 = this->capacity() >> 4u;
  __m128i *         src_m128i = (__m128i *)src;
  __m128i *         dst_m128i = (__m128i *)dst;
  for (std::size_t i = 0; i < c_over_16; ++i)
    _mm_store_si128(dst_m128i + i, _mm_load_si128(src_m128i + i));
}

void rpl::Graph::print() const
{
  std::size_t index = 0;
  std::cerr << "Metadata: " << this->width() << " " << this->height() << " " << this->size() << " " << this->capacity() << "\n\n";
  std::size_t max_n = floor(log10(this->height()) + 1);
  for (std::size_t row = 0; row < this->height(); ++row)
  {
    std::cerr << std::setw(max_n) << row << ": ";
    for (std::size_t col = 0; col < this->width(); ++col)
    {
      index = row * this->width() + col;
      std::cerr << std::bitset<8>(this->p_data[index]) << " ";
    }
    std::cerr << "\n";
  }
}

void rpl::Graph::add_node()
{
  const std::size_t height   = this->d_nodes + 1;
  const std::size_t width    = ((height + 7u) & ~(7u)) >> 3u;
  const std::size_t size     = height * width;
  const std::size_t capacity = (size + 15u) & ~(15u);

  if (this->capacity() == capacity)
  {
    ++this->d_nodes;
    return;
  }

  std::uint8_t *temp = new std::uint8_t[capacity]();
  if (width == this->width())
    this->fast_copy(this->p_data, temp);
  else
  {
    std::size_t dst = 0;
    for (std::size_t i = 0; i < this->height(); ++i, ++dst)
      for (std::size_t j = 0; j < this->width(); ++j, ++dst)
        temp[dst] = this->p_data[i * this->width() + j];
  }

  delete[] this->p_data;
  this->p_data = temp;
  this->d_nodes += 1;
}

void rpl::Graph::adjacent(const std::size_t &v, std::vector<std::size_t> &out) const
{
  out.clear();
  out.reserve(this->d_nodes);
  for (std::size_t i = 0; i < this->d_nodes; ++i)
    if (this->operator()(v, i)) out.emplace_back(i);
}