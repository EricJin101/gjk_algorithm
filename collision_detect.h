// Copyright (c) 2021 Trunk.Tech. All rights reserved.
//

#ifndef GJK_COLLISION_DETECT_H
#define GJK_COLLISION_DETECT_H

#include "iostream"
#include "cmath"
#include "vector"
#include <algorithm>
#include <limits.h>
using namespace std;
namespace eric{
    namespace collision_detect{
        struct point
        {
            double x;
            double y;
        };
        typedef vector<point> Polygon;
        Polygon polygon1;
        Polygon polygon2;
        Polygon minkowski_diff;
        bool collision_occurs{false};
        string collision_method;

        bool point_in_vector(Polygon polygon, point& p1)
        {// 判断点在不在多边形中（vector）
            for (int i{0}; i < polygon.size(); ++i)
            {
                if (polygon[i].x == p1.x && polygon[i].y == p1.y)
                {
                    return true;
                }
            }
            return false;
        }

        void polygon_minus(Polygon& poly1, Polygon& poly2)
        {// polygon minus
            for (int p1{0}; p1 < poly1.size(); ++p1)
            {
                for (int p2{0}; p2 < poly2.size(); ++p2)
                {
                    point tem_p{};
                    tem_p.x = poly1[p1].x - poly2[p2].x;
                    tem_p.y = poly1[p1].y - poly2[p2].y;
                    if (!point_in_vector(minkowski_diff, tem_p))
                    {
                        minkowski_diff.push_back(tem_p);
                    }
                }
            }
        }


    }
}
#endif //GJK_COLLISION_DETECT_H
