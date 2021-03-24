// Copyright (c) 2021 Trunk.Tech. All rights reserved.
//

#include "collision_detect.h"
namespace eric{
    namespace collision_detect{
        bool point_to_rectangle(int x1, int y1, int x2, int y2, int w, int h)
        {
            // standard rectangle
            cout << "first function" << endl;
            return x1 >= x2 && x1 <= x2 + w && y1 >= y2 && y1 <= y2 + h;
        }
        bool point_to_rectangle(double x1, double y1, double x2, double y2, double w, double h)
        {
            // standard rectangle
            cout << "second function" << endl;
            return x1 >= x2 && x1 <= x2 + w && y1 >= y2 && y1 <= y2 + h;
        }
        double isTriangleOrArea(int x1, int y1, int x2, int y2, int x3, int y3)
        {
            return abs((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0);
        }
        bool point_in_triangle(int x, int y, int x1, int y1, int x2, int y2, int x3, int y3)
        {
            double ABC = isTriangleOrArea(x1,y1,x2,y2,x3,y3);// 是否为三角形
            double PBC = isTriangleOrArea(x,y,x2,y2,x3,y3);
            double PAC = isTriangleOrArea(x1,y1,x,y,x3,y3);
            double PAB = isTriangleOrArea(x1,y1,x2,y2,x,y);
            return (ABC == PBC + PAC + PAB);
        }

        void find_new_ploy()
        {

        }

    }
}