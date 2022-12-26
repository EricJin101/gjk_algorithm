#ifndef GJK_COLLISION_DETECT_H_
#define GJK_COLLISION_DETECT_H_

#include <memory>

#include <limits.h>
#include <algorithm>
#include "cmath"
#include "data_struct.h"
#include "iostream"
#include "vector"
using namespace std;
namespace eric {
namespace collision_detect {
// struct point {
//   double x;
//   double y;
// };
// typedef vector<point> Polygon;
// Polygon polygon1;
// Polygon polygon2;
// Polygon minkowski_diff;
// Polygon Simplex;
// point direction{};
// bool collision_occurs{false};
// string collision_method;

class TestifyHalfLine {
 public:
  TestifyHalfLine(){};

  bool Init(const std::vector<Point>& poly1, const std::vector<Point>& poly2);

  /*
   * @brief: 射线法
   * */
  bool Check();

 private:
  void GetMinkowskiDiff();

  bool HalfLineMethod(std::vector<Point> convex_polygon);

  size_t FindNextEdge(std::vector<Point> convex_polygon, size_t now,
                      double nowK, std::vector<Point> convex_edge, bool flag);

  double ComputeK(const Point& p1, const Point& p2);

 private:
  std::string name_ = "half line method";
  std::vector<Point> polygon1_;
  std::vector<Point> polygon2_;
  std::vector<Point> minkowski_diff_;  // 闵可夫斯基差
};

class TestifyTriangle {
 public:
  TestifyTriangle(){};
  bool Init(const std::vector<Point>& poly1, const std::vector<Point>& poly2);

  /**
   * 在minkowski差中任意选择三条边，组成三条三角形，判断原点在不在其内
   * */
  bool Check();

 private:
  void GetMinkowskiDiff();

  /**
   * @brief: 是否为三角形
   * */
  double IsTriangle(double x1, double x2, double x3, double y1, double y2,
                    double y3);

  /**
   * 判断x,y 在不在由(x1,y1), (x2,y2), (x3,y3)组成的三角形内
   * */
  bool PointInTriangle(double x, double y, double x1, double y1, double x2,
                       double y2, double x3, double y3);

 private:
  std::string name_ = "triangle method";
  std::vector<Point> polygon1_;
  std::vector<Point> polygon2_;
  std::vector<Point> minkowski_diff_;  // 闵可夫斯基差
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
   * @breif: 求minkowski差,两个多边形顶点求差, 并去除相同点, 填入minkowski_diff
   *         如 (4, 11) - (5, 7) = (-1, 4)
   *            (11, 11) - (12, 7) = (-1, 4)
   *            这两个点的minkowski差相同
   * */
  void GetMinkowskiDiff();

  /**
   * @breif: 从闵可夫斯基差中选择最远点
   * (从Shape1的direction方向, 和Shape2的 - direction方向)
   *
   * */
  Point GetFarthestinDirection();

  /**
   * @breif: 判断是否包含原点
   * */
  bool OriginContained();

  /**(AC x AB) x AB = AB .x (AB .x AC) - AC .x (AB .x AB)
   * 在C点对侧的AB的垂向量， .x表示点乘； x代表叉乘
   * 返回向量叉积*/
  Point CrossProduct(const Point& v1, const Point& v2, const Point& v3);

 private:
  std::string name_ = "gjk";
  std::vector<Point> polygon1_;
  std::vector<Point> polygon2_;
  std::vector<Point> simplex_;

  Point direction_{1, 0};  // 可以任意选一个方向

  std::vector<Point> minkowski_diff_;  // 闵可夫斯基差
};

}  // namespace collision_detect
}  // namespace eric

#endif  // GJK_COLLISION_DETECT_H_
