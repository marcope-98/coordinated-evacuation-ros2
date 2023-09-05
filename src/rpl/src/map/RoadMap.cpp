#include "rpl/map/RoadMap.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <utility>

#include "rpl/internal/geometry.hpp"
#include "rpl/internal/utils.hpp"

rpl::RoadMap::RoadMap(const std::vector<Polygon> &obstacles, const Point &goal) : RoadMap()
{
  std::size_t n_nodes{1}; // (goal)
  for (const auto &polygon : obstacles)
    n_nodes += polygon.size();

  this->d_graph = std::move(Graph(n_nodes));
  this->d_table.reserve(n_nodes);

  for (const auto &polygon : obstacles)
    this->d_table.emplace_back(polygon);
  this->d_table.emplace_back(goal);
}

void rpl::RoadMap::execute()
{
  const std::size_t vertices = this->d_graph.height();
  // for all vertices v \in V
  std::vector<std::size_t> W;
  W.reserve(vertices);

  for (std::size_t v = 0; v < vertices; ++v)
  {
    // do W <- VisibleVertices(v, S)
    this->visible_vertices(v, W);
    //    For every vertex w \in W, add the arc (v, w) to E
    for (const auto &w : W)
      this->d_graph.add_halfedge(v, w);
  }
}

void rpl::RoadMap::execute_single()
{
  const std::size_t vertices = this->d_graph.height();
  const std::size_t v        = vertices - 1;

  std::vector<std::size_t> W;
  W.reserve(vertices);

  this->visible_vertices(v, W);
  for (const auto &w : W)
    this->d_graph.add_edge(v, w);
}

void rpl::RoadMap::visible_vertices(const std::size_t &v, std::vector<std::size_t> &W)
{
  const std::size_t        vertices = this->d_graph.height();
  std::vector<std::size_t> out;

  /*
   sort the obstacle vertices according to the counterclockwise angle that the half-line
    from p to each vertex makes with the positive x-axis. In case of 
    ties, vertices closer to p should come before vertices farther from p.
    Let w1,...wn be the sorted list.
  */
  this->d_table.angular_sort(v, out);
  this->d_avltree.resize(vertices);

  /*
  Let rho be the half-line parallel to the positive x-axis starting at p. Find the obstacle edges
   that are properly intersected by rho, and store them in a balanced search tree T in the order 
   in which they are intersected by rho.
  */
  W.clear();
  this->AVLTree_initialize(v);

  // skip last vertices (which is v)
  for (std::size_t i = 0; i < vertices - 1; ++i)
  {
    std::size_t wi = out[i];
    this->AVLTree_update_keys(v, wi);
    if (this->AVLTree_evaluate(v, wi)) W.emplace_back(wi);
    this->AVLTree_update_tree(v, wi);
  }
}

void rpl::RoadMap::AVLTree_initialize(const std::size_t &v)
{
  this->d_avltree.reset();
  const std::size_t size = this->d_graph.height();
  float             intersection;
  for (std::size_t i = 0; i < size; ++i)
  {
    intersection = this->d_table.halfplane_intersection(v, i, this->d_table.next(i));
    if (ops::cmplt_f32(intersection, 1.f)) this->d_avltree.insert(i, intersection);
  }
}

void rpl::RoadMap::AVLTree_update_keys(const std::size_t &v, const std::size_t &wi)
{
  const std::size_t size = this->d_avltree.size();
  for (std::size_t i = 0; i < size; ++i)
    if (this->d_avltree.is_set(i))
      this->d_avltree[i] = this->d_table.intersection(v, wi, i, this->d_table.next(i));

  this->d_avltree.update_minseg();
}

void rpl::RoadMap::AVLTree_update_tree(const std::size_t &v, const std::size_t &wi)
{
  const std::size_t prev   = this->d_table.prev(wi);
  const std::size_t next   = this->d_table.next(wi);
  const float       length = 1.f;
  if (this->d_table.orientation(v, wi, prev) == -1)
    this->d_avltree.insert(prev, length);
  else
    this->d_avltree.remove(prev);

  if (this->d_table.orientation(v, wi, next) == -1)
    this->d_avltree.insert(wi, length);
  else
    this->d_avltree.remove(wi);
}

bool rpl::RoadMap::AVLTree_evaluate(const std::size_t &v, const std::size_t &wi) const
{
  if (v == wi) return false;
  if (this->d_table.in_polygon(v, wi)) return false;
  return !(ops::cmplt_f32(this->d_avltree[this->d_avltree.minseg()], 1.f));
}

void rpl::RoadMap::remove_out_of_bounds(const Polygon &border)
{
  const std::size_t n_points = this->d_graph.height();
  // NOTE: exclude goal
  for (std::size_t i = 0; i < n_points - 1; ++i)
    if (!geometry::point_in_polygon(this->d_table.point(i), border))
      this->d_graph.rm_node(i);
}

std::vector<rpl::RoadMap::Candidate> rpl::RoadMap::dijkstra(const Pose &start_pose)
{
  if (!this->d_has_start)
  {
    this->d_graph.add_node();
    this->d_table.emplace_back(start_pose.point());
    this->execute_single();

    this->d_has_start = true;
  }

  // HACK: we are exploiting the layout of tpl::Table: N obstacles vertices + 1 goal + 1 start
  const std::size_t start = this->d_table.size() - 1;
  const std::size_t goal  = start - 1;

  // initialize dijkstra algorithm
  const std::size_t                                  n_nodes = this->d_graph.height();
  std::priority_queue<std::pair<float, std::size_t>> queue;
  std::vector<float>                                 dist(n_nodes, 1e9);
  std::vector<std::size_t>                           prev(n_nodes, n_nodes);
  std::vector<bool>                                  visited(n_nodes, false);

  dist[start] = 0.f;
  queue.push({0.f, start});

  std::size_t              a;
  std::vector<std::size_t> neighbors;
  float                    update;
  // dijkstra algorithm

  while (!queue.empty())
  {
    a = queue.top().second;
    queue.pop();
    if (visited[a]) continue;
    visited[a] = true;
    this->d_graph.adjacent(a, neighbors);
    for (const auto &b : neighbors)
    {
      update = dist[a] + (this->d_table.point(b) - this->d_table.point(a)).norm();
      if (update < dist[b])
      {
        dist[b] = update;
        prev[b] = a;
        queue.push({-update, b});
      }
    }
  }
  // reverse path
  std::vector<Candidate> solution;
  std::size_t            temp = goal;
  solution.emplace_back(Candidate{goal, Pose{this->d_table.point(temp), 0.f}, 0.f, 0.f});
  Point next_point, prev_point;
  float min_range, max_range;

  while (temp != start)
  {
    if (temp == n_nodes)
    {
      solution.clear();
      return solution;
    }
    temp       = prev[temp];
    next_point = this->d_table.point(this->d_table.next(temp)) - this->d_table.point(temp);
    prev_point = this->d_table.point(this->d_table.prev(temp)) - this->d_table.point(temp);
    min_range  = utils::mod2pi(atan2f(next_point.y(), next_point.x()));
    max_range  = utils::mod2pi(atan2f(prev_point.y(), prev_point.x()));

    solution.emplace_back(Candidate{temp, Pose{this->d_table.point(temp), 0.f}, min_range, max_range});
  }

  std::reverse(solution.begin(), solution.end());
  solution[0] = Candidate{start, start_pose, 0.f, 0.f};
  return solution;
}

void rpl::RoadMap::print() const
{
  this->d_table.print();
  std::cerr << "\n";
  this->d_graph.print();
}
