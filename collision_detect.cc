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
        double isTriangleOrArea(double x1, double y1, double x2, double y2, double x3, double y3)
        {
            return abs((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0);
        }
        bool point_in_triangle(double x, double y, double x1, double y1,
                double x2, double y2, double x3, double y3)
        {
            //judge point in triangle or not
            double ABC = isTriangleOrArea(x1,y1,x2,y2,x3,y3);// 是否为三角形
            double PBC = isTriangleOrArea(x,y,x2,y2,x3,y3);
            double PAC = isTriangleOrArea(x1,y1,x,y,x3,y3);
            double PAB = isTriangleOrArea(x1,y1,x2,y2,x,y);
            return (ABC == PBC + PAC + PAB);
        }

        void find_new_ploy(Polygon& poly1, Polygon& poly2)
        {
            // 每3条判断一回是否在三角形内
            Polygon polygon_new;
            for (int p1{0}; p1 < poly1.size(); ++p1)
            {
                for (int p2{0}; p2 < poly2.size(); ++p2)
                {
                    point tem_p{};
                    tem_p.x = poly1[p1].x - poly2[p2].x;
                    tem_p.y = poly1[p1].y - poly2[p2].y;
                    polygon_new.push_back(tem_p);
                }
            }
            for (int i{0}; i < polygon_new.size(); ++i)
            {
                for (int j{i+1}; j < polygon_new.size() -1; ++j)
                {
                    for (int k{j+1}; k < polygon_new.size() - 2; ++k)
                    {
                        bool in_side(false);
                        in_side = point_in_triangle(0,0,polygon_new[i].x,polygon_new[i].y,
                                polygon_new[j].x,polygon_new[j].y,
                                polygon_new[k].x,polygon_new[k].y);
                        if (in_side)
                        {
                            cout << polygon_new[i].x << "," << polygon_new[i].y << endl;
                            cout << polygon_new[j].x << "," << polygon_new[j].y << endl;
                            cout << polygon_new[k].x << "," << polygon_new[k].y << endl;
                            cout << "in side.\n" << endl;
                        }
                    }
                }
            }
        }
        bool gjk_detect()
        {
            //
        }

    }
}