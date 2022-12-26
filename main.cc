#include "src/gjk.cc"

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
    temp.x = 3;
    temp.y = 3;
    eric::collision_detect::polygon2.push_back(temp);
    temp.x = 1;
    temp.y = 0;
    eric::collision_detect::polygon2.push_back(temp);
    // temp.x = 7;
    // temp.y = 3;
    // eric::collision_detect::polygon2.push_back(temp);
//    eric::collision_detect::method_define((string &) "triangle");
//    eric::collision_detect::collision_detect(collision::polygon1, collision::polygon2);
    eric::collision_detect::collisionDetection(eric::collision_detect::polygon1,
                                               eric::collision_detect::polygon2,
                                               "gjk");

    return 0;
}
