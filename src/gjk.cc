#include "gjk.h"

namespace eric {
namespace collision_detect {
void method_define(string& method) { collision_method = method; }

double computeKa(point p1, point p2) {
  double disX = p2.x - p1.x;
  double disY = p2.y - p1.y;
  if (disX == 0) {
    return disY > 0 ? INT_MAX : INT_MIN;
  }
  return disY / disX;
}
int findNextEdge(const Polygon& convex_polygon, int now, double& nowK,
                 Polygon& convex_edge, bool& flag) {
  /**
   * flag用于判断是不是找到横坐标最大的点，找到最大点之后再往回找
   * **/
  if (flag) {
    int max = now + 1;
    double maxK = computeKa(convex_polygon[now], convex_polygon[max]);
    for (unsigned i = now + 2; i < convex_polygon.size(); i++) {
      double k = computeKa(convex_polygon[now], convex_polygon[i]);
      if (k > maxK && k <= nowK) {
        max = i;
        maxK = k;
      }
    }
    nowK = maxK;
    return max;
  } else {
    int max = now - 1;
    double maxK = computeKa(convex_polygon[max], convex_polygon[now]);
    for (int i = now - 2; i >= 0; i--) {
      double k = computeKa(convex_polygon[i], convex_polygon[now]);
      if (k > maxK && k <= nowK) {
        max = i;
        maxK = k;
      }
    }
    nowK = maxK;
    return max;
  }
}
bool half_line_method(Polygon& convex_edge) {
  int nCross = 0;  //多少条相交线
  for (int i{0}; i < convex_edge.size() - 1; i++) {
    point p1 = convex_edge[i];
    point p2 =
        convex_edge[(i + 1) % convex_edge.size()];  // 最后一个点与第一个点连线
    if (p1.y == p2.y)
      continue;
    if (0 < min<double>(p1.y, p2.y))
      continue;  //在两个点下面
    if (0 >= max<double>(p1.y, p2.y))
      continue;  //在两个点上面
    double x =
        (double)(0 - p1.y) * (double)(p2.x - p1.x) / (double)(p2.y - p1.y) +
        p1.x;  // 求交点的x坐标
    if (x > 0) {
      nCross++;  // 只统计p1p2与p向右射线的交点
    }
  }
  return (nCross % 2 == 1);  //交点为偶数，点在多边形之外
}
bool convex_method(Polygon& poly1, Polygon& poly2) {
  polygon_minus(poly1, poly2);
  Polygon convex_polygon;
  convex_polygon = minkowski_diff;  //两个矩阵相减之后所有顶点
  sort(convex_polygon.begin(), convex_polygon.end(),
       [](const point& p1, const point& p2) {
         return p1.y > p2.y;
       });  //坐标系按照y从大到小排列、algorithm
  stable_sort(
      convex_polygon.begin(), convex_polygon.end(),
      [](const point& p1, const point& p2) { return p1.x < p2.x; });  //左到右
  Polygon convex_edge;  //凸多边形的各个顶点
  int now = 0;  //在所有顶点中的位置
  double nowK = INT_MAX;  //下一个边界点,在convex_polygon中的id
  point next_point{};
  next_point = convex_polygon[now];
  bool flag{true};
  now = findNextEdge(convex_polygon, now, nowK, convex_edge, flag);
  convex_edge.push_back(next_point);  //先将第一个点加在边界点中，最左边一个点
  while (0 != now) {
    if (now == convex_polygon.size() - 1) {
      flag = false;
      nowK = INT_MAX;
    }
    convex_edge.push_back(convex_polygon[now]);
    now = findNextEdge(convex_polygon, now, nowK, convex_edge, flag);
  }
  for (int jj{0}; jj < convex_edge.size(); ++jj) {
    cout << convex_edge[jj].x << "," << convex_edge[jj].y << endl;
  }
  if (half_line_method(convex_edge)) {
    cout << "in side" << endl;
  }
}

void collisionDetection(Polygon& poly1, Polygon& poly2, string method_select) {
  std::unique_ptr<GJK> gjk_ = nullptr;
  gjk_ = std::make_unique<GJK>();

  std::unique_ptr<TestifyTriangle> test_triangle_ = nullptr;
  test_triangle_ = std::make_unique<TestifyTriangle>();

  std::unique_ptr<TestifyHalfLine> test_halfline_ = nullptr;
  test_halfline_ = std::make_unique<TestifyHalfLine>();
  cout << "using " << method_select << endl;
  Point tpt;
  std::vector<Point> po1, po2;
  for (auto p : poly1) {
    tpt.set_x(p.x);
    tpt.set_y(p.y);
    po1.emplace_back(tpt);
  }
  for (auto p : poly2) {
    tpt.set_x(p.x);
    tpt.set_y(p.y);
    po2.emplace_back(tpt);
  }
  if (method_select == "convex") {
    test_halfline_->Init(po1, po2);
    test_halfline_->Check();
  } else if (method_select == "triangle") {
    test_triangle_->Init(po1, po2);
    test_triangle_->Check();
  } else if (method_select == "gjk") {
    gjk_->Init(po1, po2);
    gjk_->Check();
  }
}

bool TestifyHalfLine::Init(const std::vector<Point>& poly1,
                           const std::vector<Point>& poly2) {
  if (poly1.size() < 3 && poly2.size() < 3) {
    return false;
  }
  polygon1_ = poly1;
  polygon2_ = poly2;
  return true;
}

void TestifyHalfLine::GetMinkowskiDiff() {
  auto inMinkowski = [](std::vector<Point>& poly, Point pt) -> bool {
    for (auto pp : poly) {
      if (pp == pt) {
        return true;
      }
    }
    return false;
  };
  for (const auto& p1 : polygon1_) {
    for (const auto& p2 : polygon2_) {
      Point tp{p1.x() - p2.x(), p1.y() - p2.y()};
      if (!inMinkowski(minkowski_diff_, tp)) {
        minkowski_diff_.emplace_back(tp);
      }
    }
  }
}

bool TestifyHalfLine::Check() {
  GetMinkowskiDiff();
  std::vector<Point> convex_polygon = minkowski_diff_;
  for (auto p : convex_polygon) {
    cout << p.x() << ", " << p.y() << endl;
  }
  sort(convex_polygon.begin(), convex_polygon.end(),
       [](const Point& p1, const Point& p2) { return p1.y() > p2.y(); });
  //坐标系按照y从大到小排列、algorithm
  stable_sort(convex_polygon.begin(), convex_polygon.end(),
              [](const Point& p1, const Point& p2) { return p1.x() < p2.x(); });
  //左到右
  std::vector<Point> convex_edge;
  int now = 0;  //在所有顶点中的位置
  double nowK = std::numeric_limits<double>::max();
  //下一个边界点,在convex_polygon中的id
  Point next_point;
  next_point = convex_polygon[now];
  bool flag{true};
  now = FindNextEdge(convex_polygon, now, nowK, convex_edge, flag);
  convex_edge.emplace_back(next_point);
  while (0 != now) {
    if (now == convex_polygon.size() - 1) {
      flag = false;
      nowK = INT_MAX;
    }
    convex_edge.emplace_back(convex_polygon[now]);
    now = FindNextEdge(convex_polygon, now, nowK, convex_edge, flag);
  }
  if (HalfLineMethod(convex_edge)) {
    cout << "halfline in side" << endl;
    return true;
  }

  cout << "half_line_method out" << endl;
  return false;
}

bool TestifyHalfLine::HalfLineMethod(std::vector<Point> convex_edge) {
  int nCross = 0;  //多少条相交线
  for (int i{0}; i < convex_edge.size() - 1; i++) {
    Point p1 = convex_edge[i];
    Point p2 =
        convex_edge[(i + 1) % convex_edge.size()];  // 最后一个点与第一个点连线
    if (p1.y() == p2.y())
      continue;
    if (0 < min<double>(p1.y(), p2.y()))
      continue;  //在两个点下面
    if (0 >= max<double>(p1.y(), p2.y()))
      continue;  //在两个点上面
    double x = (double)(0 - p1.y()) * (double)(p2.x() - p1.x()) /
                   (double)(p2.y() - p1.y()) +
               p1.x();  // 求交点的x坐标
    if (x > 0) {
      nCross++;  // 只统计p1p2与p向右射线的交点
    }
  }
  return (nCross % 2 == 1);  //交点为偶数，点在多边形之外
}

double TestifyHalfLine::ComputeK(const Point& p1, const Point& p2) {
  double disX = p2.x() - p1.x();
  double disY = p2.y() - p1.y();
  if (disX == 0) {
    return disY > 0 ? INT_MAX : INT_MIN;
  }
  return disY / disX;
}

size_t TestifyHalfLine::FindNextEdge(std::vector<Point> convex_polygon,
                                     size_t now, double nowK,
                                     std::vector<Point> convex_edge,
                                     bool flag) {
  /**
   * flag用于判断是不是找到横坐标最大的点，找到最大点之后再往回找
   * **/
  if (flag) {
    int max = now + 1;
    double maxK = ComputeK(convex_polygon[now], convex_polygon[max]);
    for (unsigned i = now + 2; i < convex_polygon.size(); i++) {
      double k = ComputeK(convex_polygon[now], convex_polygon[i]);
      if (k > maxK && k <= nowK) {
        max = i;
        maxK = k;
      }
    }
    nowK = maxK;
    return max;
  } else {
    int max = now - 1;
    double maxK = ComputeK(convex_polygon[max], convex_polygon[now]);
    for (int i = now - 2; i >= 0; i--) {
      double k = ComputeK(convex_polygon[i], convex_polygon[now]);
      if (k > maxK && k <= nowK) {
        max = i;
        maxK = k;
      }
    }
    nowK = maxK;
    return max;
  }
}

bool TestifyTriangle::Init(const std::vector<Point>& poly1,
                           const std::vector<Point>& poly2) {
  // 不是多边形
  if (poly1.size() < 3 && poly2.size() < 3) {
    return false;
  }
  polygon1_ = poly1;
  polygon2_ = poly2;
  // TODO: 在一条直线?
  return true;
}

bool TestifyTriangle::Check() {
  GetMinkowskiDiff();
  for (int i{0}; i < minkowski_diff_.size(); ++i) {
    for (int j{0}; j < minkowski_diff_.size(); ++j) {
      for (int k{0}; k < minkowski_diff_.size(); ++k) {
        if (i == j || i == k || j == k) {
          continue;
        }
        bool in_side(false);
        in_side = PointInTriangle(
            0, 0, minkowski_diff_[i].x(), minkowski_diff_[i].y(),
            minkowski_diff_[j].x(), minkowski_diff_[j].y(),
            minkowski_diff_[k].x(), minkowski_diff_[k].y());
        if (in_side) {
          cout << "test triangle in side.\n" << endl;
          return true;
        }
      }
    }
  }
  return false;
}

void TestifyTriangle::GetMinkowskiDiff() {
  auto inMinkowski = [](std::vector<Point>& poly, Point pt) -> bool {
    for (auto pp : poly) {
      if (pp == pt) {
        return true;
      }
    }
    return false;
  };
  for (const auto& p1 : polygon1_) {
    for (const auto& p2 : polygon2_) {
      Point tp{p1.x() - p2.x(), p1.y() - p2.y()};
      if (!inMinkowski(minkowski_diff_, tp)) {
        minkowski_diff_.emplace_back(tp);
      }
    }
  }
}

double TestifyTriangle::IsTriangle(double x1, double y1, double x2, double y2,
                                   double x3, double y3) {
  return abs((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0);
}

bool TestifyTriangle::PointInTriangle(double x, double y, double x1, double y1,
                                      double x2, double y2, double x3,
                                      double y3) {
  double ABC = IsTriangle(x1, y1, x2, y2, x3, y3);
  double PBC = IsTriangle(x, y, x2, y2, x3, y3);
  double PAC = IsTriangle(x1, y1, x, y, x3, y3);
  double PAB = IsTriangle(x1, y1, x2, y2, x, y);
  return (ABC == PBC + PAC + PAB);
}

bool GJK::Init(const std::vector<Point>& poly1,
               const std::vector<Point>& poly2) {
  // 不是多边形
  if (poly1.size() < 3 && poly2.size() < 3) {
    return false;
  }
  polygon1_ = poly1;
  polygon2_ = poly2;
  // TODO: 在一条直线?
  return true;
}

void GJK::GetMinkowskiDiff() {
  auto inMinkowski = [](std::vector<Point>& poly, Point pt) -> bool {
    for (auto pp : poly) {
      if (pp == pt) {
        return true;
      }
    }
    return false;
  };

  cout << "         ^*         " << endl;
  cout << "        { }         " << endl;
  cout << "       {°  }        " << endl;
  cout << "      {    :}       " << endl;
  cout << "        | |         " << endl;
  cout << "  merry christmas!  " << endl;
  cout << "                  2022-12-23  " << endl;

  for (const auto& p1 : polygon1_) {
    for (const auto& p2 : polygon2_) {
      Point tp{p1.x() - p2.x(), p1.y() - p2.y()};
      if (!inMinkowski(minkowski_diff_, tp)) {
        minkowski_diff_.emplace_back(tp);
      }
    }
  }
}

bool GJK::Check() {
  GetMinkowskiDiff();
  simplex_.emplace_back(GetFarthestinDirection());
  direction_ = direction_.Negate();
  while (true) {
    simplex_.emplace_back(GetFarthestinDirection());
    if (simplex_.back().InnerProd(direction_) < 0.0) {
      // 在direction的投影距离为负数
      // simplex最后一个点是direction方向的最远点
      // 最远点都在direction的"后面"
      cout << "not inside" << endl;
      return false;
    } else {
      if (OriginContained()) {
        cout << "wow in side." << endl;
        return true;
      }
    }
  }

  return false;
}

Point GJK::GetFarthestinDirection() {
  double x_min = -std::numeric_limits<double>::max();
  Point far_pt;
  for (const auto& pt : minkowski_diff_) {
    double inner_pro = pt.InnerProd(direction_);
    if (inner_pro > x_min) {
      x_min = inner_pro;
      far_pt = pt;
    }
  }
  return far_pt;
}

bool GJK::OriginContained() {
  if (simplex_.empty()) {
    return false;
  }
  Point a = simplex_.back();
  if (3 == simplex_.size()) {
    Point b = simplex_.at(1);
    Point c = simplex_.at(0);
    Point ab = b - a;
    Point ac = c - a;
    Point abPrep = CrossProduct(ac, ab, ab);
    Point acPrep = CrossProduct(ab, ac, ac);
    if ((abPrep.x() * (-a.x()) + abPrep.y() * (-a.y())) > 0.0) {
      simplex_.erase(simplex_.begin());
      direction_.set_x(abPrep.x());
      direction_.set_y(abPrep.y());
      return false;
    } else {
      if ((acPrep.x() * (-a.x()) + acPrep.y() * (-a.y())) > 0.0) {
        simplex_.erase(simplex_.begin() + 1);
        direction_.set_x(acPrep.x());
        direction_.set_y(acPrep.y());
        return false;
      }
      return true;
    }
  } else {
    // 单纯形只有两个点时, 更新direction
    Point b = simplex_.front();
    Point ab = b - a;
    Point abPrep = CrossProduct(ab, a.Negate(), ab);
    direction_ = abPrep;
    return false;
  }
}

Point GJK::CrossProduct(const Point& v1, const Point& v2, const Point& v3) {
  return {(v3.x() * v1.x() + v3.y() * v1.y()) * v2.x() -
              (v3.x() * v2.x() + v3.y() * v2.y()) * v1.x(),
          (v3.x() * v1.x() + v3.y() * v1.y()) * v2.y() -
              (v3.x() * v2.x() + v3.y() * v2.y()) * v1.y()};
}

}  // namespace collision_detect
}  // namespace eric
