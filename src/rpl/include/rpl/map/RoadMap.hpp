#ifndef RPL_MAP_ROADMAP_HPP_
#define RPL_MAP_ROADMAP_HPP_

#include <cstdint>
#include <vector>

#include "rpl/types.hpp"

#include "rpl/map/AVLTree.hpp"
#include "rpl/map/Graph.hpp"
#include "rpl/map/Table.hpp"

namespace rpl
{
  struct RoadMap
  {
  public:
    struct Candidate
    {
      std::size_t ref_index;
      Pose        pose;
      float       min_range;
      float       max_range;
    };

    RoadMap() = default;
    RoadMap(const std::vector<Polygon> &obstacles, const Point &goal);
    RoadMap(const RoadMap &other) { *this = other; }
    RoadMap(RoadMap &&other) noexcept { *this = std::move(other); }
    ~RoadMap() = default;

    RoadMap &operator=(const RoadMap &other)
    {
      this->d_graph     = other.d_graph;
      this->d_table     = other.d_table;
      this->d_has_start = other.d_has_start;
      return *this;
    }

    RoadMap &operator=(RoadMap &&other) noexcept
    {
      this->d_graph     = std::move(other.d_graph);
      this->d_table     = std::move(other.d_table);
      this->d_has_start = std::move(other.d_has_start); // smh
      return *this;
    }

    Graph &graph() { return this->d_graph; }
    Graph  graph() const { return this->d_graph; }
    Table &table() { return this->d_table; }
    Table  table() const { return this->d_table; }

    void                   execute();
    std::vector<Candidate> dijkstra(const Pose &start_point);
    void                   remove_out_of_bounds(const Polygon &border);
    void                   print() const;

  private:
    void execute_single();
    void visible_vertices(const std::size_t &v, std::vector<std::size_t> &W);

    void AVLTree_initialize(const std::size_t &v);
    void AVLTree_update_keys(const std::size_t &v, const std::size_t &wi);
    void AVLTree_update_tree(const std::size_t &v, const std::size_t &wi);
    bool AVLTree_evaluate(const std::size_t &v, const std::size_t &wi) const;

  private:
    Graph   d_graph;
    Table   d_table;
    AVLTree d_avltree;

    bool d_has_start{false};
  };
} // namespace rpl
#endif // RPL_MAP_ROADMAP_HPP_