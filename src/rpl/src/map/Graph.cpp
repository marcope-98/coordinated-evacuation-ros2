#include "rpl/map/Graph.hpp"

// simd
#include <immintrin.h>

rpl::Graph::Graph(const std::size_t &nodes) : d_nodes(nodes)
{
  const std::size_t c     = this->capacity();
  this->p_AdjacencyMatrix = new std::uint8_t[c]();
}

rpl::Graph::Graph(const Graph &other) : d_nodes(other.d_nodes)
{
  this->p_AdjacencyMatrix = new std::uint8_t[this->capacity()]();
  this->fast_copy(other);
}

rpl::Graph &rpl::Graph::operator=(const Graph &other)
{
  if (this->capacity() < other.capacity()) this->reallocate(other.capacity());
  this->d_nodes = other.d_nodes;
  this->fast_copy(other);
  return *this;
}

rpl::Graph &rpl::Graph::operator=(Graph &&other) noexcept
{
  std::swap(this->d_nodes, other.d_nodes);
  std::swap(this->p_AdjacencyMatrix, other.p_AdjacencyMatrix);
  return *this;
}

rpl::Graph::~Graph() { this->deallocate_all(); }

void rpl::Graph::add_edge(const std::size_t &node1, const std::size_t &node2)
{
  if (node1 == node2) return;
  this->add_halfedge(node1, node2);
  this->add_halfedge(node2, node1);
}

void rpl::Graph::add_halfedge(const std::size_t &src, const std::size_t &dst)
{
  std::size_t div_node = dst >> 3u;
  std::size_t rem_node = dst - (div_node << 3u);
  std::size_t index    = src * this->width() + div_node;
  this->p_AdjacencyMatrix[index] |= (0x80u >> rem_node);
}

void rpl::Graph::rm_node(const std::size_t &node)
{
  const std::size_t s = this->height();
  for (std::size_t i = 0; i < s; ++i)
    this->rm_edge(node, i);
}

void rpl::Graph::rm_edge(const std::size_t &node1, const std::size_t &node2)
{
  this->rm_halfedge(node1, node2);
  this->rm_halfedge(node2, node1);
}

void rpl::Graph::rm_halfedge(const std::size_t &node1, const std::size_t &node2)
{
  std::size_t div_node = node2 >> 3u;
  std::size_t rem_node = node2 - (div_node << 3u);
  std::size_t index    = node1 * this->width() + div_node;
  this->p_AdjacencyMatrix[index] &= ~(0x80u >> rem_node);
}

void rpl::Graph::deallocate_all()
{
  this->d_nodes = 0;
  delete[] this->p_AdjacencyMatrix;
  this->p_AdjacencyMatrix = nullptr;
}

void rpl::Graph::fast_copy(const Graph &other)
{
  const std::size_t c_over_16 = other.capacity() >> 4u;
  __m128i *         src       = (__m128i *)other.p_AdjacencyMatrix;
  __m128i *         dst       = (__m128i *)this->p_AdjacencyMatrix;

  for (std::size_t i = 0; i < c_over_16; ++i)
    _mm_storeu_si128(dst + i, _mm_loadu_si128(src + i));
}

void rpl::Graph::reallocate(const std::size_t &capacity)
{
  this->deallocate_all();
  this->p_AdjacencyMatrix = new std::uint8_t[capacity]();
}

bool rpl::Graph::operator()(const std::size_t &src, const std::size_t &dst) const
{
  std::size_t div_node = dst >> 3u;
  std::size_t rem_node = dst - (div_node << 3u);
  std::size_t index    = src * this->width() + div_node;
  return bool(this->p_AdjacencyMatrix[index] & (0x80u >> rem_node));
}

void rpl::Graph::neighborhood(const std::size_t &node, std::vector<std::size_t> &out) const
{
  const std::size_t nodes = this->height();

  out.clear();
  out.reserve(nodes);

  for (std::size_t i = 0; i < nodes; ++i)
    if (this->operator()(node, i))
      out.emplace_back(i);
}