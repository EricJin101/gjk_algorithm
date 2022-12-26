#include "src/gjk.h"

using namespace eric;

int main() {
  cout << "多边形已经输入完成。" << endl;
  // eric::collision_detect::point temp{};
  // temp.x = 4;
  // temp.y = 5;
  // eric::collision_detect::polygon1.push_back(temp);
  // temp.x = 8;
  // temp.y = 5;
  // eric::collision_detect::polygon1.push_back(temp);
  // temp.x = 4;
  // temp.y = 2;
  // eric::collision_detect::polygon1.push_back(temp);
  // temp.x = 8;
  // temp.y = 2;
  // eric::collision_detect::polygon1.push_back(temp);

  // temp.x = 1;
  // temp.y = 3;
  // eric::collision_detect::polygon2.push_back(temp);
  // temp.x = 5;
  // temp.y = 3;
  // eric::collision_detect::polygon2.push_back(temp);
  // temp.x = 1;
  // temp.y = 0;
  // eric::collision_detect::polygon2.push_back(temp);
  // // temp.x = 7;
  // // temp.y = 3;
  // // polygon2.push_back(temp);
  // // method_define((string&)"triangle");
  // eric::collision_detect::collisionDetection(eric::collision_detect::polygon1,
  // eric::collision_detect::polygon2, "convex");
  // eric::collision_detect::collisionDetection(eric::collision_detect::polygon1,
  // eric::collision_detect::polygon2, "triangle");
  // eric::collision_detect::collisionDetection(eric::collision_detect::polygon1,
  // eric::collision_detect::polygon2, "gjk");
  std::unique_ptr<collision_detect::GJK> gjk_ = nullptr;
  gjk_ = std::make_unique<collision_detect::GJK>();

  std::unique_ptr<collision_detect::TestifyTriangle> test_triangle_ = nullptr;
  test_triangle_ = std::make_unique<collision_detect::TestifyTriangle>();

  std::unique_ptr<collision_detect::TestifyHalfLine> test_halfline_ = nullptr;
  test_halfline_ = std::make_unique<collision_detect::TestifyHalfLine>();
  std::vector<Point> po1, po2;
  Point tpt;
  tpt = {4.0, 5.0};
  po1.emplace_back(tpt);
  tpt = {8.0, 5.0};
  po1.emplace_back(tpt);
  tpt = {4.0, 2.0};
  po1.emplace_back(tpt);
  tpt = {8.0, 2.0};
  po1.emplace_back(tpt);

  tpt = {1.0, 3.0};
  po2.emplace_back(tpt);
  tpt = {5.0, 3.0};
  po2.emplace_back(tpt);
  tpt = {1.0, 0.0};
  po2.emplace_back(tpt);

  test_halfline_->Init(po1, po2);
  test_halfline_->Check();
  test_triangle_->Init(po1, po2);
  test_triangle_->Check();
  gjk_->Init(po1, po2);
  gjk_->Check();

  return 0;
}
