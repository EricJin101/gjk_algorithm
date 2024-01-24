#include "src/gjk.h"

using namespace eric;

int main() {
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
  cout << "多边形已经输入完成。" << endl;

  test_halfline_->Init(po1, po2);
  test_halfline_->Check();

  test_triangle_->Init(po1, po2);
  test_triangle_->Check();

  gjk_->Init(po1, po2);
  gjk_->Check();

  return 0;
}
