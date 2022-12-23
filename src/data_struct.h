#ifndef DATA_STRUCT_H_
#define DATA_STRUCT_H_

#include <cmath>

namespace eric {

extern const double kMathEpsilon;

template <typename T>
const T operator+(const T& lhs, const T& rhs) {
  return T(lhs) += rhs;
}

template <typename T>
const T operator-(const T& lhs, const T& rhs) {
  return T(lhs) -= rhs;
}

template <typename T>
const T operator*(const T& Point, const double ratio) {
  return T(Point) *= ratio;
}

template <typename T>
const T operator*(const double ratio, const T& Point) {
  return Point * ratio;
}

template <typename T>
bool operator!=(const T& lhs, const T& rhs) {
  return !(lhs == rhs);
}

class Point {
 public:
  // Constructor returning the zero
  Point() = default;

  // Constructor which takes x-y-coordinates.
  Point(const double x, const double y) : x_(x), y_(y) {}

  // Getter for x component
  double x() const { return x_; }

  // Setter for x component
  void set_x(const double x) { x_ = x; }

  // Getter for y component
  double y() const { return y_; }

  // Setter for y component
  void set_y(const double y) { y_ = y; }

  // get vertical vector
  Point Vert() const { return Point(-y_, x_); }

  // get opposite vector
  Point Negate() const { return Point(-x_, -y_); }

  // Returns the "cross" product between these two Vec2d (non-standard).
  double CrossProd(const Point& other) const;

  // Returns the inner product between these two Vec2d.
  double InnerProd(const Point& other) const;

  // Sums another Point to the current one
  Point& operator+=(const Point& other);

  // Subtracts another Point to the current one
  Point& operator-=(const Point& other);

  // Multiplies this Point by a scalar
  Point& operator*=(double ratio);

  // Compares two Point
  bool operator==(const Point& other) const;

 protected:
  double x_ = 0.0;

  double y_ = 0.0;
};

}  // namespace eric

#endif  // DATA_STRUCT_H_ 
