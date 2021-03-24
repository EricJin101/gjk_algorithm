#include "collision_detect.cc"

int main()
{
    bool result;
    result = eric::collision_detect::point_to_rectangle(0, 0, -1, -1, 2, 2);
    cout << result << endl;
    bool r;
    r = eric::collision_detect::point_in_triangle(0,0, 1,1,2,2,-1,-2);
    cout << r << endl;
    eric::collision_detect::Point ploy1;

}