#ifndef GJK_COLLISION_DETECT_H_
#define GJK_COLLISION_DETECT_H_

#include <memory>

#include <limits.h>
#include <algorithm>
#include "cmath"
#include "data_struct.cc"
#include "iostream"
#include "vector"
using namespace std;
namespace eric {
namespace collision_detect {
struct point {
  double x;
  double y;
};
typedef vector<point> Polygon;
Polygon polygon1;
Polygon polygon2;
Polygon minkowski_diff;
Polygon Simplex;
point direction{};
bool collision_occurs{false};
string collision_method;
point negative_vector(point& pt) {
  pt.x = -pt.x;
  pt.y = -pt.y;
  return pt;
}
point vector_minus(point pt1, point pt2) {  // pt1 - pt2
  return {pt1.x - pt2.x, pt1.y - pt2.y};
}
bool point_in_vector(Polygon polygon,
                     point& p1) {  // 判断点在不在多边形顶点中（vector）
  for (int i{0}; i < polygon.size(); ++i) {
    if (polygon[i].x == p1.x && polygon[i].y == p1.y) {
      return true;
    }
  }
  return false;
}

void polygon_minus(Polygon& poly1, Polygon& poly2) {  // polygon minus
  for (int p1{0}; p1 < poly1.size(); ++p1) {
    for (int p2{0}; p2 < poly2.size(); ++p2) {
      point tem_p{};
      tem_p.x = poly1[p1].x - poly2[p2].x;
      tem_p.y = poly1[p1].y - poly2[p2].y;
      if (!point_in_vector(minkowski_diff, tem_p)) {
        minkowski_diff.push_back(tem_p);
      }
    }
  }
}

class TestifyTriangle {
 public:
 private:
};

class GJK {
 public:
  GJK(){};
  bool Init(const std::vector<Point>& poly1, const std::vector<Point>& poly2);
  /**
   * @breif: 给两定多边形, 判断是否相交
   * */
  bool Check();

 private:
  /**
   * @breif: 求mincov diff
   * */
  void GetMincovDiff();
  Point GetFarthestinDirection();
  bool OriginContained();

  Point CrossProduct(const Point& v1, const Point& v2, const Point& v3);

 private:
  std::vector<Point> polygon1_;
  std::vector<Point> polygon2_;
  std::vector<Point> simplex_;

  Point direction_{1, 0};

  std::vector<Point> minkowski_diff_;  // 敏可夫差
};

}  // namespace collision_detect
}  // namespace eric

#endif  // GJK_COLLISION_DETECT_H_
