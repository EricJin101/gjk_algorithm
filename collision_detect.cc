// Copyright (c) 2021 Trunk.Tech. All rights reserved.
//

#include "collision_detect.h"
namespace eric{
    namespace collision_detect{
        void method_define(string& method)
        {
            collision_method = method;
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
        bool triangle_method()
        {
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
        double computeKa(point p1, point p2){
            double disX = p2.x - p1.x;
            double disY = p2.y - p1.y;
            if (disX == 0)
            {
                return disY > 0 ? INT_MAX : INT_MIN;
            }
            return disY / disX;
        }
        int findNextEdge(const Polygon& convex_polygon, int now, double &nowK, bool &flag)
        {
            /**
             * flag用于判断是不是找到横坐标最大的点，找到最大点之后再往回找
             * **/
        }
        bool convex_method(Polygon& poly1, Polygon& poly2)
        {
            polygon_minus(poly1, poly2);
            Polygon convex_polygon;
            convex_polygon = minkowski_diff;
            sort(convex_polygon.begin(), convex_polygon.end(),
                 [](const point &p1, const point &p2){
                     return p1.y > p2.y;
                 });//坐标系按照y从大到小排列、algorithm
            stable_sort(convex_polygon.begin(), convex_polygon.end(),
                        [](const point &p1, const point &p2){
                            return p1.x < p2.x;
                        });//左到右
            Polygon convex_edge;

        }

        bool gjk_method(Polygon& poly1, Polygon& poly2)
        {
            //
            // find a vector direction
        }

        void collisionDetection(Polygon& poly1, Polygon& poly2)
        {
            polygon_minus(poly1, poly2);
            bool triangle_result = triangle_method();
//            return result;
        }

    }
}