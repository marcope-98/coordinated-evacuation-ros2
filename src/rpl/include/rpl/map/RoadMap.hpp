#ifndef RPL_MAP_ROADMAP_HPP_
#define RPL_MAP_ROADMAP_HPP_
// stl
#include <cstdint>
#include <vector>
// rpl
#include "rpl/types.hpp"
// rpl/map
#include "rpl/map/AVLTree.hpp"
#include "rpl/map/Graph.hpp"

namespace rpl
{
  struct RoadMap
  {
  private:
    struct Segment
    {
      std::size_t prev = 0;
      std::size_t next = 0;
    };

    AVLTree *avltree  = nullptr;
    Graph *  graph    = nullptr;
    Point *  points   = nullptr;
    Segment *segments = nullptr;

  public:
    RoadMap() = default;
    RoadMap(const std::vector<Polygon> &obstacles,
            const std::vector<Point> &  gates);
    RoadMap(const RoadMap &other) { *this = other; }
    RoadMap(RoadMap &&other) noexcept { *this = std::move(other); }
    ~RoadMap();

    RoadMap &operator=(const RoadMap &other);
    RoadMap &operator=(RoadMap &&other) noexcept;

  private:
    // call stack of visibiltiy graph construction
    void compute_visibility_graph();
    void visible_vertices(const std::size_t &v, std::vector<std::size_t> &W);
    void angular_sort(const std::size_t &v, std::vector<std::size_t> &sorted_vertices) const;
    bool visible(const std::size_t &v, const std::size_t &wi) const;
    void rm_out_of_bounds_nodes(const Polygon &border);

    // rpl::geometry wrrappers
    float        get_intersection(const std::size_t &p0, const std::size_t &p1, const std::size_t &p2, const std::size_t &p3) const;
    float        get_halfplane_intersection(const std::size_t &v, const std::size_t &edge1, const std::size_t &edge2);
    std::int32_t orientation(const std::size_t &p0, const std::size_t &p1, const std::size_t &p2) const;
    bool         intersects(const std::size_t &p0, const std::size_t &p1, const std::size_t &p2, const std::size_t &p3) const;

    // memory management
    void deallocate_all();

    // AVLTree management
    void AVLTree_initialize(const std::size_t &v);
    void AVLTree_update_keys(const std::size_t &v, const std::size_t &wi);
    void AVLTree_update_tree(const std::size_t &v, const std::size_t &wi);

    // utils for polygon processing
    std::size_t count_polygon_vertices(const std::vector<Polygon> &obstacles) const;
    bool        in_polygon(const std::size_t &v, const std::size_t &wi) const;
    bool        polygon_search(const std::size_t &v, const std::size_t &wi) const;
  };
} // namespace rpl

#endif // RPL_MAP_ROADMAP_HPP_