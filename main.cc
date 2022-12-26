#include "src/gjk.cc"
using namespace eric::collision_detect;

int main() {
  cout << "多边形已经输入完成。" << endl;
  eric::collision_detect::point temp{};
  temp.x = 4;
  temp.y = 5;
  eric::collision_detect::polygon1.push_back(temp);
  temp.x = 8;
  temp.y = 5;
  eric::collision_detect::polygon1.push_back(temp);
  temp.x = 4;
  temp.y = 2;
  eric::collision_detect::polygon1.push_back(temp);
  temp.x = 8;
  temp.y = 2;
  eric::collision_detect::polygon1.push_back(temp);

  temp.x = 1;
  temp.y = 3;
  eric::collision_detect::polygon2.push_back(temp);
  temp.x = 5;
  temp.y = 3;
  eric::collision_detect::polygon2.push_back(temp);
  temp.x = 1;
  temp.y = 0;
  eric::collision_detect::polygon2.push_back(temp);
  // temp.x = 7;
  // temp.y = 3;
  // polygon2.push_back(temp);
  // method_define((string&)"triangle");
  eric::collision_detect::collisionDetection(eric::collision_detect::polygon1, eric::collision_detect::polygon2, "convex");
  eric::collision_detect::collisionDetection(eric::collision_detect::polygon1, eric::collision_detect::polygon2, "triangle");
  eric::collision_detect::collisionDetection(eric::collision_detect::polygon1, eric::collision_detect::polygon2, "gjk");

  return 0;
}
