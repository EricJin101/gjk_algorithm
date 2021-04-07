#include "collision_detect.cc"

int main() {
    cout << "多边形已经输入完成。" << endl;
    // 22号没有提交
    eric::collision_detect::point temp{};
    temp.x = 4;
    temp.y = 11;
    eric::collision_detect::polygon1.push_back(temp);
    temp.x = 9;
    temp.y = 9;
    eric::collision_detect::polygon1.push_back(temp);
    temp.x = 4;
    temp.y = 5;
    eric::collision_detect::polygon1.push_back(temp);

    temp.x = 5;
    temp.y = 7;
    eric::collision_detect::polygon2.push_back(temp);
    temp.x = 12;
    temp.y = 7;
    eric::collision_detect::polygon2.push_back(temp);
    temp.x = 10;
    temp.y = 2;
    eric::collision_detect::polygon2.push_back(temp);
    temp.x = 7;
    temp.y = 3;
    eric::collision_detect::polygon2.push_back(temp);
//    eric::collision_detect::method_define((string &) "triangle");
//    eric::collision_detect::collision_detect(collision::polygon1, collision::polygon2);
    eric::collision_detect::collisionDetection(eric::collision_detect::polygon1,
                                               eric::collision_detect::polygon2,
                                               "gjk");

    return 0;
}
