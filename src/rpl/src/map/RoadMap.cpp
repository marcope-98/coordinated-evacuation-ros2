#include "rpl/map/RoadMap.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

#include "rpl/internal/geometry.hpp"
#include "rpl/internal/rplintrin.hpp"
#include "rpl/internal/utils.hpp"

rpl::RoadMap::RoadMap(const std::vector<Polygon> &obstacles,
                      const std::vector<Point> &  gates) : RoadMap()
{
  // count number of nodes
  std::size_t n_vertices = this->count_polygon_vertices(obstacles);
  std::size_t n_gates    = gates.size();
  std::size_t n_nodes    = n_vertices + n_gates;

  // initialize member variables
  this->avltree  = new AVLTree(n_nodes);
  this->graph    = new Graph(n_nodes);
  this->points   = new Point[n_nodes];
  this->segments = new Segment[n_nodes];

  // process obstacles
  std::size_t i = 0;
  std::size_t first, second;
  for (const auto &polygon : obstacles)
  {
    first  = i;
    second = i + polygon.size() - 1;
    for (const auto &point : polygon)
    {
      this->points[i]   = point;
      this->segments[i] = {i - 1, i + 1};
      ++i;
    }
    this->segments[first].prev  = second;
    this->segments[second].next = first;
  }

  // process gates
  for (const auto &point : gates)
  {
    this->points[i]   = point;
    this->segments[i] = {i, i};
    ++i;
  }
}

rpl::RoadMap::~RoadMap()
{
  this->deallocate_all();
}

void rpl::RoadMap::deallocate_all()
{
  delete this->graph;
  delete this->avltree;
  delete[] this->points;
  delete[] this->segments;
}

std::size_t rpl::RoadMap::count_polygon_vertices(const std::vector<Polygon> &obstacles) const
{
  std::size_t res = 0;
  for (const auto &polygon : obstacles)
    res += polygon.size();
  return res;
}

void rpl::RoadMap::compute_visibility_graph()
{
  const std::size_t        vertices = this->graph->height();
  std::vector<std::size_t> W;
  W.reserve(vertices);
  // for all vertices v \in V
  for (std::size_t v = 0; v < vertices; ++v)
  {
    // do W <- VisibleVertices(v, S)
    this->visible_vertices(v, W);
    //    For every vertex w \in W, add the arc (v, w) to E
    for (const auto &w : W)
      this->graph->add_halfedge(v, w);
  }
}

void rpl::RoadMap::visible_vertices(const std::size_t &v, std::vector<std::size_t> &W)
{
  const std::size_t        vertices = this->graph->height();
  std::vector<std::size_t> out;
  /*
   sort the obstacle vertices according to the counterclockwise angle that the half-line
    from p to each vertex makes with the positive x-axis. In case of 
    ties, vertices closer to p should come before vertices farther from p.
    Let w1,...wn be the sorted list.
  */
  this->angular_sort(v, out);

  /*
  Let rho be the half-line parallel to the positive x-axis starting at p. Find the obstacle edges
   that are properly intersected by rho, and store them in a balanced search tree T in the order 
   in which they are intersected by rho.
  */
  this->AVLTree_initialize(v);
  W.clear();
  std::size_t wi;
  // skip last vertices (which is v)
  for (std::size_t i = 0; i < vertices - 1; ++i)
  {
    wi = out[i];
    this->AVLTree_update_keys(v, wi);
    if (this->visible(v, wi))
      W.emplace_back(wi);
    this->AVLTree_update_tree(v, wi);
  }
}

void rpl::RoadMap::angular_sort(const std::size_t &v, std::vector<std::size_t> &sorted_vertices) const
{
  const std::size_t  vertices = this->graph->height();
  std::vector<float> theta;
  std::vector<float> rho;
  theta.reserve(vertices);
  rho.reserve(vertices);

  float x_ref = this->points[v].x;
  float y_ref = this->points[v].y;
  float dx, dy;
  for (std::size_t i = 0; i < vertices; ++i)
  {
    dx = this->points[i].x - x_ref;
    dy = this->points[i].y - y_ref;
    rho.emplace_back(dx * dx + dy * dy);
    theta.emplace_back(utils::mod2pi(atan2(dy, dx)));
  }
  theta[v] = 8.F;

  // actual angular sort
  sorted_vertices.resize(vertices);
  std::iota(sorted_vertices.begin(), sorted_vertices.end(), 0);
  std::sort(sorted_vertices.begin(), sorted_vertices.end(), [&](const std::size_t &a, const std::size_t &b)
            { return (theta[a] < theta[b]) ||
                     ((theta[a] == theta[b]) && (rho[a] < rho[b])); });
}

bool rpl::RoadMap::visible(const std::size_t &v, const std::size_t &wi) const
{
  if (v == wi) return false;
  // if l does not lie between the two edges incident on v (the sweep line does not intersect the interior of the obstacle at v)
  if (this->in_polygon(v, wi)) return false;
  // if the line segment vwi does not intersect the closest edge in S
  std::size_t minseg = this->avltree->minseg;
  return !(ops::cmplt_f32(this->avltree->keys[minseg], 1.F));
}

void rpl::RoadMap::AVLTree_initialize(const std::size_t &v)
{
  this->avltree->reset();
  const std::size_t size = this->graph->height();
  float             intersection_point;
  std::size_t       j;
  for (std::size_t i = 0; i < size; ++i)
  {
    j                  = this->segments[i].next;
    intersection_point = this->get_halfplane_intersection(v, i, j);
    if (ops::cmplt_f32(intersection_point, 1.f))
      this->avltree->insert(i, intersection_point);
  }
}

void rpl::RoadMap::AVLTree_update_keys(const std::size_t &v, const std::size_t &wi)
{
  Segment     segment;
  float       intersection;
  std::size_t s = this->avltree->size;

  for (std::size_t node = 0; node < s; ++node)
  {
    if (this->avltree->is_set(node))
    {
      segment      = this->segments[node];
      intersection = this->get_intersection(v, wi, node, segment.next);
      this->avltree->update(node, intersection);
    }
  }
  this->avltree->update_minseg();
}

void rpl::RoadMap::AVLTree_update_tree(const std::size_t &v, const std::size_t &wi)
{
  Segment ab     = this->segments[wi];
  float   length = 1.f;

  if (this->orientation(v, wi, ab.prev) == -1)
    this->avltree->insert(ab.prev, length);
  else
    this->avltree->remove(ab.prev);

  if (this->orientation(v, wi, ab.next) == -1)
    this->avltree->insert(wi, length);
  else
    this->avltree->remove(wi);
}

bool rpl::RoadMap::in_polygon(const std::size_t &v, const std::size_t &wi) const
{
  Segment seg = this->segments[wi];
  // 2 cases: v and wi belong to the same polygon, v and wi do not belong to the same polygon

  // TODO: is there a better way of doing this?
  if (this->polygon_search(v, wi))
  {
    if (seg.prev == v || seg.next == v) return false;
    bool one_left = false, one_right = false;
    for (std::size_t edge = seg.next; edge != v; edge = this->segments[edge].next)
    {
      if (this->orientation(wi, v, edge) == 1)
      {
        one_right = true;
        break;
      }
    }

    for (std::size_t edge = seg.prev; edge != v; edge = this->segments[edge].prev)
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

bool rpl::RoadMap::polygon_search(const std::size_t &v, const std::size_t &wi) const
{
  std::size_t vertex = this->segments[wi].next;
  while (vertex != wi)
  {
    if (vertex == v) return true;
    vertex = this->segments[vertex].next;
  }

  return false;
}

void rpl::RoadMap::rm_out_of_bounds_nodes(const Polygon &border)
{
  const std::size_t n_points = this->graph->height();
  Point             point;

  for (std::size_t i = 0; i < n_points; ++i)
  {
    point = this->points[i];
    if (!geometry::point_in_polygon(point, border))
      this->graph->rm_node(i);
  }
}

float rpl::RoadMap::get_intersection(const std::size_t &p0, const std::size_t &p1, const std::size_t &p2, const std::size_t &p3) const
{
  Point x0 = this->points[p0];
  Point x1 = this->points[p1];
  Point y0 = this->points[p2];
  Point y1 = this->points[p3];
  return geometry::segment_segment_intersection({x0, x1}, {y0, y1});
}

float rpl::RoadMap::get_halfplane_intersection(const std::size_t &v, const std::size_t &edge1, const std::size_t &edge2)
{
  Point x0 = this->points[v];
  Point y0 = this->points[edge1];
  Point y1 = this->points[edge2];
  return geometry::halfplane_intersection(x0, {y0, y1});
}

std::int32_t rpl::RoadMap::orientation(const std::size_t &p0, const std::size_t &p1, const std::size_t &p2) const
{
  Point x0 = this->points[p0];
  Point x1 = this->points[p1];
  Point x2 = this->points[p2];
  return geometry::orientation(x0, x1, x2);
}

bool rpl::RoadMap::intersects(const std::size_t &p0, const std::size_t &p1, const std::size_t &p2, const std::size_t &p3) const
{
  Point x0 = this->points[p0];
  Point x1 = this->points[p1];
  Point y0 = this->points[p2];
  Point y1 = this->points[p3];
  return geometry::intersects({x0, x1}, {y0, y1});
}

rpl::RoadMap &rpl::RoadMap::operator=(const RoadMap &other)
{
  *(this->graph)      = *(other.graph);
  std::size_t n_nodes = this->graph->height();

  //
  this->avltree = new AVLTree(n_nodes);

  this->points   = new Point[n_nodes];
  this->segments = new Segment[n_nodes];

  std::copy(other.points, other.points + n_nodes, this->points);
  std::copy(other.segments, other.segments + n_nodes, this->segments);
}

rpl::RoadMap &rpl::RoadMap::operator=(RoadMap &&other) noexcept
{
  std::swap(this->avltree, other.avltree);
  std::swap(this->graph, other.graph);
  std::swap(this->points, other.points);
  std::swap(this->segments, other.segments);
  return *this;
}