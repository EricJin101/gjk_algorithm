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
        typedef vector<point> Point;
        Point tri;
    }
}
#endif //GJK_COLLISION_DETECT_H
