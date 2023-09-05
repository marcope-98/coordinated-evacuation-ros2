#ifndef RPL_INTERNAL_POINT_HPP_
#define RPL_INTERNAL_POINT_HPP_

#include <cmath>
#include <utility>
namespace rpl
{
  struct Point
  {
    Point() = default;
    Point(const float &x, const float &y) : d_x{x}, d_y{y} {}
    Point(const Point &other) { *this = other; }
    Point(Point &&other) noexcept { *this = std::move(other); }
    ~Point() = default;

    Point &operator=(const Point &other)
    {
      this->d_x = other.d_x;
      this->d_y = other.d_y;
      return *this;
    }

    Point &operator=(Point &&other) noexcept
    {
      std::swap(this->d_x, other.d_x);
      std::swap(this->d_y, other.d_y);
      return *this;
    }

    float  x() const { return this->d_x; }
    float &x() { return this->d_x; }
    float  y() const { return this->d_y; }
    float &y() { return this->d_y; }

    Point operator+() const { return *this; }
    Point operator-() const { return Point{-this->d_x, -this->d_y}; }

    Point operator+(const float &rhs) const { return Point{this->d_x + rhs, this->d_y + rhs}; }
    Point operator-(const float &rhs) const { return Point{this->d_x - rhs, this->d_y - rhs}; }
    Point operator*(const float &rhs) const { return Point{this->d_x * rhs, this->d_y * rhs}; }
    Point operator/(const float &rhs) const { return Point{this->d_x / rhs, this->d_y / rhs}; }

    Point operator+(const Point &rhs) const { return Point{this->d_x + rhs.d_x, this->d_y + rhs.d_y}; }
    Point operator-(const Point &rhs) const { return Point{this->d_x - rhs.d_x, this->d_y - rhs.d_y}; }
    Point operator*(const Point &rhs) const { return Point{this->d_x * rhs.d_x, this->d_y * rhs.d_y}; }
    Point operator/(const Point &rhs) const { return Point{this->d_x / rhs.d_x, this->d_y / rhs.d_y}; }

    Point &operator+=(const Point &rhs)
    {
      this->d_x += rhs.d_x;
      this->d_y += rhs.d_y;
      return *this;
    }

    Point &operator-=(const Point &rhs)
    {
      this->d_x -= rhs.d_x;
      this->d_y -= rhs.d_y;
      return *this;
    }

    Point &operator*=(const Point &rhs)
    {
      this->d_x *= rhs.d_x;
      this->d_y *= rhs.d_y;
      return *this;
    }

    Point &operator/=(const Point &rhs)
    {
      this->d_x /= rhs.d_x;
      this->d_y /= rhs.d_y;
      return *this;
    }

    float norm() const { return sqrtf(this->norm2()); }
    float norm2() const { return this->d_x * this->d_x + this->d_y * this->d_y; }

  private:
    float d_x{0.f};
    float d_y{0.f};
  };

  inline Point operator+(const float &scalar, const Point &point) { return Point{scalar + point.x(), scalar + point.y()}; }
  inline Point operator-(const float &scalar, const Point &point) { return Point{scalar - point.x(), scalar - point.y()}; }
  inline Point operator*(const float &scalar, const Point &point) { return Point{scalar * point.x(), scalar * point.y()}; }
  inline Point operator/(const float &scalar, const Point &point) { return Point{scalar / point.x(), scalar / point.y()}; }

} // namespace rpl

#endif