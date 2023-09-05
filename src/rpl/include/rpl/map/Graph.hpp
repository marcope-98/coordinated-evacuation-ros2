#ifndef RPL_MAP_GRAPH_HPP_
#define RPL_MAP_GRAPH_HPP_

#include <cstdint>
#include <utility>
#include <vector>

namespace rpl
{
  struct Graph
  {
  public:
    // Constructors
    Graph() = default;
    explicit Graph(const std::size_t &N);
    Graph(const Graph &other) { *this = other; }
    Graph(Graph &&other) noexcept { *this = std::move(other); }
    ~Graph();

    // Operators
    Graph &       operator=(const Graph &other);
    Graph &       operator=(Graph &&other) noexcept;
    bool          operator()(const std::size_t &n1, const std::size_t &n2) const;
    std::uint8_t  operator[](const std::size_t &index) const { return this->p_data[index]; }
    std::uint8_t &operator[](const std::size_t &index) { return this->p_data[index]; }

    // Getters
    std::size_t width() const { return ((this->d_nodes + 7u) & ~(7u)) >> 3u; }
    std::size_t height() const { return this->d_nodes; }
    std::size_t size() const { return this->width() * this->height(); }
    std::size_t capacity() const { return (this->size() + 15u) & ~(15u); }

    // add
    void add_edge(const std::size_t &n1, const std::size_t &n2);
    void add_halfedge(const std::size_t &n1, const std::size_t &n2);

    // remove
    void rm_node(const std::size_t &node);
    void rm_edge(const std::size_t &n1, const std::size_t &n2);
    void rm_halfedge(const std::size_t &n1, const std::size_t &n2);
    // print
    void print() const;

    void add_node();
    void adjacent(const std::size_t &v, std::vector<std::size_t> &out) const;

  private:
    std::size_t   d_nodes{0};
    std::uint8_t *p_data{nullptr};

  private:
    void deallocate_all();
    void reallocate(const std::size_t &capacity);
    void fast_copy(const std::uint8_t *const src, std::uint8_t *const dst);
  };
} // namespace rpl

#endif // RPL_MAP_GRAPH_HPP_