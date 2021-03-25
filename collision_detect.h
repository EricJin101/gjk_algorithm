// Copyright (c) 2021 Trunk.Tech. All rights reserved.
//

#ifndef GJK_COLLISION_DETECT_H
#define GJK_COLLISION_DETECT_H

#include "iostream"
#include "cmath"
#include "vector"
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
        bool collision_occurs{false};
    }
}
#endif //GJK_COLLISION_DETECT_H
