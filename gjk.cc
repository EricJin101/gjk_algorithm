#include "gjk.h"
namespace eric {
namespace collision_detect {
void method_define(string& method) { collision_method = method; }

double isTriangleOrArea(double x1, double y1, double x2, double y2, double x3,
                        double y3) {
  /**
   * 是否为三角形**/
  return abs((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0);
}
bool point_in_triangle(double x, double y, double x1, double y1, double x2,
                       double y2, double x3, double y3) {
  /**
   * 判断x,y 在不在由(x1,y1), (x2,y2), (x3,y3)组成的三角形内**/
  double ABC = isTriangleOrArea(x1, y1, x2, y2, x3, y3);  // 是否为三角形
  double PBC = isTriangleOrArea(x, y, x2, y2, x3, y3);
  double PAC = isTriangleOrArea(x1, y1, x, y, x3, y3);
  double PAB = isTriangleOrArea(x1, y1, x2, y2, x, y);
  return (ABC == PBC + PAC + PAB);
}
bool triangle_method() {
  /**
   * 选择三条边，组成三条三角形，判断原点在不在其内**/
  for (int i{0}; i < minkowski_diff.size(); ++i) {
    for (int j{i + 1}; j < minkowski_diff.size() - 1; ++j) {
      for (int k{j + 1}; k < minkowski_diff.size() - 2; ++k) {
        bool in_side(false);
        in_side = point_in_triangle(
            0, 0, minkowski_diff[i].x, minkowski_diff[i].y, minkowski_diff[j].x,
            minkowski_diff[j].y, minkowski_diff[k].x, minkowski_diff[k].y);
        if (in_side) {
          cout << minkowski_diff[i].x << "," << minkowski_diff[i].y << endl;
          cout << minkowski_diff[j].x << "," << minkowski_diff[j].y << endl;
          cout << minkowski_diff[k].x << "," << minkowski_diff[k].y << endl;
          cout << "in side.\n" << endl;
          return true;
        }
      }
    }
  }
  return false;
}
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
void getFarthestPointInDirection(point direction, point& pt) {
  /**
   * direction 为向量方向
   * pt正方向上最远端*/
  double x_min = INT_MIN;
  int idx_max{0};
  for (int i{0}; i < minkowski_diff.size(); ++i) {
    double dot_product =
        minkowski_diff[i].x * direction.x + minkowski_diff[i].y * direction.y;
    if (dot_product > x_min) {
      x_min = dot_product;
      idx_max = i;
    }
  }
  pt.x = minkowski_diff[idx_max].x;
  pt.y = minkowski_diff[idx_max].y;
}
point support(point direction) {
  point pt{};
  getFarthestPointInDirection(direction, pt);
  return pt;
}
point crossProduct(point vector1, point vector2, point vector3) {
  /**(AC x AB) x AB = AB .x (AB .x AC) - AC .x (AB .x AB)
   * 在C点对侧的AB的垂向量， .x表示点乘； x代表叉乘
   * 返回向量叉积*/
  return {(vector3.x * vector1.x + vector3.y * vector1.y) * vector2.x -
              (vector3.x * vector2.x + vector3.y * vector2.y) * vector1.x,
          (vector3.x * vector1.x + vector3.y * vector1.y) * vector2.y -
              (vector3.x * vector2.x + vector3.y * vector2.y) * vector1.y};
}
bool containedOrigin() {  //判断是否包含原点
  if (Simplex.empty()) {
    return false;
  }
  point a = Simplex.back();
  if (Simplex.size() == 3) {
    point b = Simplex.at(1);
    point c = Simplex.at(0);
    point ab = vector_minus(b, a);
    point ac = vector_minus(c, a);
    point abPrep = crossProduct(ac, ab, ab);
    point acPrep = crossProduct(ab, ac, ac);
    if ((abPrep.x * (-a.x) + abPrep.y * (-a.y)) > 0.0) {
      Simplex.erase(Simplex.begin());
      direction.x = abPrep.x;
      direction.y = abPrep.y;
      return false;
    } else {
      if ((acPrep.x * (-a.x) + acPrep.y * (-a.y)) > 0.0) {
        Simplex.erase(Simplex.begin() + 1);
        direction.x = acPrep.x;
        direction.y = acPrep.y;
        return false;
      } else {
        return true;
      }
    }
  } else {
    point b = Simplex.front();
    point ab = vector_minus(b, a);
    point abPrep = crossProduct(ab, negative_vector(a), ab);
    // 更新direction
    direction.x = abPrep.x;
    direction.y = abPrep.y;
    return false;
  }
}
bool gjk_method(Polygon& poly1, Polygon& poly2) {
  //从minkowski中选，然后最大方向的两个点，垂直方向再选两个最大点
  direction.x = 1;
  direction.y = 0;
  Simplex.push_back(support(direction));  //添加一个点
  negative_vector(direction);
  while (true) {
    Simplex.push_back(support(direction));
    if (Simplex.back().x * direction.x + Simplex.back().y * direction.y < 0.0) {
      return false;
    } else {
      if (containedOrigin()) {
        cout << "in side." << endl;
        return true;
      }
    }
  }
}
void collisionDetection(Polygon& poly1, Polygon& poly2, string method_select) {
  polygon_minus(poly1, poly2);
  cout << method_select << endl;
  if (method_select == "convex") {
    convex_method(poly1, poly2);
  } else if (method_select == "triangle") {
    bool triangle_result = triangle_method();
  } else if (method_select == "gjk") {
    gjk_method(poly1, poly2);
  }
}

}  // namespace collision_detect
}  // namespace eric
