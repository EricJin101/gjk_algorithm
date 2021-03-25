// Copyright (c) 2021 Trunk.Tech. All rights reserved.
//

#include "collision_detect.h"
namespace eric{
    namespace collision_detect{
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

        bool find_new_ploy(Polygon& poly1, Polygon& poly2)
        {// 每3条判断一回是否在三角形内
            polygon_minus(poly1, poly2);
            for (int i{0}; i < minkowski_diff.size(); ++i)
            {
                for (int j{i+1}; j < minkowski_diff.size() -1; ++j)
                {
                    for (int k{j+1}; k < minkowski_diff.size() - 2; ++k)
                    {
                        bool in_side(false);
                        in_side = point_in_triangle(0,0,minkowski_diff[i].x,minkowski_diff[i].y,
                                                    minkowski_diff[j].x,minkowski_diff[j].y,
                                                    minkowski_diff[k].x,minkowski_diff[k].y);
                        if (in_side)
                        {
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

        bool gjk_detect(Polygon& poly1, Polygon& poly2)
        {
            //

        }

    }
}