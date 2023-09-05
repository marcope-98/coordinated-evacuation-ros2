#ifndef RPL_MAP_TABLE_HPP_
#define RPL_MAP_TABLE_HPP_

#include <cstdint>
#include <vector>

#include "rpl/types.hpp"

#include "rpl/internal/geometry.hpp"

namespace rpl
{
  struct Table
  {
  public:
    // Constructors
    Table() = default;
    explicit Table(const std::size_t &N);
    Table(const Table &other) { *this = other; }
    Table(Table &&other) noexcept { *this = std::move(other); };
    ~Table();

    // Manipulators
    Table &operator=(const Table &other);
    Table &operator=(Table &&other) noexcept;

    // Getters
    std::size_t next(const std::size_t &idx) const { return (std::size_t)this->p_next[idx]; }
    std::size_t prev(const std::size_t &idx) const { return (std::size_t)this->p_prev[idx]; }
    Point       point(const std::size_t &idx) const { return {this->p_x[idx], this->p_y[idx]}; }
    std::size_t size() const { return this->d_size; }
    std::size_t capacity() const { return this->d_capacity; }

    // Methods
    void reserve(const std::size_t &N);
    void emplace_back(const Point &point);
    void emplace_back(const Polygon &polygon);
    void emplace_back(const Point &point, const std::uint32_t &prev, const std::uint32_t &next);

    void angular_sort(const std::size_t &v, std::vector<std::size_t> &sorted_vertices) const;

    std::int32_t orientation(const std::size_t &p0, const std::size_t &p1, const std::size_t &p2) const;
    float        intersection(const std::size_t &p0, const std::size_t &p1, const std::size_t &q0, const std::size_t &q1) const;
    float        halfplane_intersection(const std::size_t &origin, const std::size_t &p, const std::size_t &q) const;
    bool         intersects(const std::size_t &p0, const std::size_t &p1, const std::size_t &q0, const std::size_t &q1) const;
    bool         polygon_search(const std::size_t &v, const std::size_t &wi) const;
    bool         in_polygon(const std::size_t &v, const std::size_t &wi) const;

    void print() const;

  private:
    void        reallocate(const std::size_t &size);
    void        deallocate_all();
    std::size_t align_up(const std::size_t &value) const { return (value + 3u) & ~(3u); }

  private:
    std::size_t    d_size{0};
    std::size_t    d_capacity{0};
    std::uint32_t *p_prev{nullptr};
    std::uint32_t *p_next{nullptr};
    float *        p_x{nullptr};
    float *        p_y{nullptr};
  };

  inline float Table::intersection(const std::size_t &p0, const std::size_t &p1, const std::size_t &q0, const std::size_t &q1) const
  {
    return geometry::segment_segment_intersection(this->point(p0), this->point(p1), this->point(q0), this->point(q1));
  }

  inline float Table::halfplane_intersection(const std::size_t &origin, const std::size_t &p, const std::size_t &q) const
  {
    return geometry::halfplane_intersection(this->point(origin), this->point(p), this->point(q));
  }

  inline bool Table::intersects(const std::size_t &p0, const std::size_t &p1, const std::size_t &q0, const std::size_t &q1) const
  {
    return geometry::intersects(this->point(p0), this->point(p1), this->point(q0), this->point(q1));
  }

  inline std::int32_t Table::orientation(const std::size_t &p0, const std::size_t &p1, const std::size_t &p2) const
  {
    return geometry::orientation(this->point(p0), this->point(p1), this->point(p2));
  }

} // namespace rpl

#endif