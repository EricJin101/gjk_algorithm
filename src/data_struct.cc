#include "data_struct.h"

namespace eric {

const double kMathEpsilon = 1e-5;

/*
 * Point
 */
Point& Point::operator+=(const Point& other) {
  x_ += other.x_;
  y_ += other.y_;
  return *this;
}

Point& Point::operator-=(const Point& other) {
  x_ -= other.x_;
  y_ -= other.y_;
  return *this;
}

Point& Point::operator*=(const double ratio) {
  x_ *= ratio;
  y_ *= ratio;
  return *this;
}

bool Point::operator==(const Point& other) const {
  return std::fabs(x_ - other.x()) < kMathEpsilon &&
         std::fabs(y_ - other.y()) < kMathEpsilon;
}

double Point::CrossProd(const Point& other) const {
  return x_ * other.y() - y_ * other.x();
}

double Point::InnerProd(const Point& other) const {
  return x_ * other.x() + y_ * other.y();
}

}  // namespace eric
