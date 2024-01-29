#include "gjk.h"

namespace eric {
namespace collision_detect {

bool TestifyHalfLine::Init(const std::vector<Point>& poly1,
                           const std::vector<Point>& poly2) {
  cout << "using " << name_ << endl;
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
  // 坐标系按照y从大到小排列、algorithm
  stable_sort(convex_polygon.begin(), convex_polygon.end(),
              [](const Point& p1, const Point& p2) { return p1.x() < p2.x(); });
  // 左到右
  std::vector<Point> convex_edge;
  int now = 0;  // 在所有顶点中的位置
  double nowK = std::numeric_limits<double>::max();
  // 下一个边界点,在convex_polygon中的id
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
  int nCross = 0;  // 多少条相交线
  for (int i{0}; i < convex_edge.size() - 1; i++) {
    Point p1 = convex_edge[i];
    Point p2 =
        convex_edge[(i + 1) % convex_edge.size()];  // 最后一个点与第一个点连线
    if (p1.y() == p2.y())
      continue;
    if (0 < min<double>(p1.y(), p2.y()))
      continue;  // 在两个点下面
    if (0 >= max<double>(p1.y(), p2.y()))
      continue;  // 在两个点上面
    double x = (double)(0 - p1.y()) * (double)(p2.x() - p1.x()) /
                   (double)(p2.y() - p1.y()) +
               p1.x();  // 求交点的x坐标
    if (x > 0) {
      nCross++;  // 只统计p1p2与p向右射线的交点
    }
  }
  return (nCross % 2 == 1);  // 交点为偶数，点在多边形之外
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
  cout << "using " << name_ << endl;
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
  cout << "using " << name_ << endl;
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
      // 全都在direction的正方向上，分离轴直接分开了
      // 分离轴为与direciton垂直且经过原点的直线
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
    // 单纯形只有两个点时, 更新direction两点垂线
    Point b = simplex_.front();
    Point ab = b - a;
    Point abPrep = CrossProduct(ab, a.Negate(), ab);
    // ab x -a 右手定则 得到ad
    // ad x ab 右手定则，得到的ae是与ab垂直且朝向原点方向
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
