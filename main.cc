#include "collision_detect.cc"

int main() {
    bool result;
    result = eric::collision_detect::point_to_rectangle(0, 0, -1, -1, 2, 2);
    cout << result << endl;
    bool r;
    r = eric::collision_detect::point_in_triangle(0, 0, 1, 1, 2, 2, -1, -2);
    cout << r << endl;
    int read_finish{2};
//    while (read_finish > 0) {
//        eric::collision_detect::point tem_p{};
//        cout << "输入x(input x): " << endl;
//        cin >> tem_p.x;
//        cout << "输入y(input y)" << endl;
//        cin >> tem_p.y;
//        cout << "多边形输入完成（0:完成，1:输入1，2:输入2）：" << endl;
//        cin >> read_finish;
//        if (read_finish > 1) {
//            eric::collision_detect::polygon1.push_back(tem_p);
//        } else {
//            eric::collision_detect::polygon2.push_back(tem_p);
//        }
//    }

        cout << "多边形已经输入完成。" << endl;
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

        eric::collision_detect::find_new_ploy(eric::collision_detect::polygon1,
                                              eric::collision_detect::polygon2);

    return 0;
}