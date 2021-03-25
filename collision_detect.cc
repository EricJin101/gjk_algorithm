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
            /**
             * 是否为三角形**/
            return abs((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0);
        }
        bool point_in_triangle(double x, double y, double x1, double y1,
                double x2, double y2, double x3, double y3)
        {
            /**
             * 判断x,y 在不在由(x1,y1), (x2,y2), (x3,y3)组成的三角形内**/
            double ABC = isTriangleOrArea(x1,y1,x2,y2,x3,y3);// 是否为三角形
            double PBC = isTriangleOrArea(x,y,x2,y2,x3,y3);
            double PAC = isTriangleOrArea(x1,y1,x,y,x3,y3);
            double PAB = isTriangleOrArea(x1,y1,x2,y2,x,y);
            return (ABC == PBC + PAC + PAB);
        }
        bool triangle_method()
        {
            /**
             * 选择三条边，组成三条三角形，判断原点在不在其内**/
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
        bool in_edge(Polygon& convex_edge, point& p1)
        {
            /**
             * @param convex_edge是凸多边形的边界
             * @param p1是某一点
             * @return true代表p1在convex_edge中, false代表p1不在convex_edge中**/
            for (int i{0}; i < convex_edge.size(); ++i)
            {
                if (convex_edge[i].x == p1.x && convex_edge[i].y == p1.y)
                {
                    return true;
                }
            }
            return false;
        }
        void findNextEdge(const Polygon& convex_polygon, int now, int &nowK, Polygon& convex_edge)
        {
            /**
             * flag用于判断是不是找到横坐标最大的点，找到最大点之后再往回找
             * **/
             point temp{};
             temp = convex_polygon[now];
             if (!in_edge(convex_edge, temp))
             {//判断点在不在边界点中
                //now这个点不在边界里面，再判断找出最大的斜率
                double temp_k = 0;
                int i = 0;
                for (; i < convex_polygon.size(); ++i)
                {//遍历所有顶点
                    // 如果这个点不在边界点中，就计算与当前点的斜率
                    point calculate_point{};
                    calculate_point = convex_polygon[i];
                    if (in_edge(convex_edge, calculate_point))
                    {
                        continue;
                    }
                    double k = computeKa(temp, convex_polygon[i]);//求所有点的斜率k
                    if (k > temp_k)
                    {
                        temp_k = k;//寻找最大的斜率
                        nowK = i;//记录，下一个点在顶点vector中的位置
                    }
                }
                point final_point{};
                final_point = convex_polygon[nowK];
                if (!in_edge(convex_edge, final_point))
                {//最后斜率最大的点不在边界点中，就添加到边界点
                    convex_edge.push_back(convex_polygon[nowK]);
                }

             }
        }
        bool half_line_method(Polygon& convex_edge)
        {//原点位0,0
            int nCross = 0;
            for (int i{0}; i < convex_edge.size(); i++)
            {//找没一个坐标点
                Point p1 = convex_edge[i];
                Point p2 = convex_edge[(i + 1) % nCount];// 最后一个点与第一个点连线
                if (p1.y == p2.y)
                    continue;
                if (p.y < min<double>(p1.y, p2.y))
                    continue;
                if (p.y >= max<double>(p1.y, p2.y))
                    continue;
                double x = (double)(p.y - p1.y) * (double)(p2.x - p1.x) / (double)(p2.y - p1.y) + p1.x;// 求交点的x坐标
                if (x > p.x)
                {// 正方向点,如果在右边
                    nCross++;
                }
            }
            return (nCross % 2 == 1);// 交点为偶数，点在多边形之外
        }
        bool convex_method(Polygon& poly1, Polygon& poly2)
        {
            polygon_minus(poly1, poly2);
            Polygon convex_polygon;
            convex_polygon = minkowski_diff;//两个矩阵相减之后所有顶点
            sort(convex_polygon.begin(), convex_polygon.end(),
                 [](const point &p1, const point &p2){
                     return p1.y > p2.y;
                 });//坐标系按照y从大到小排列、algorithm
            stable_sort(convex_polygon.begin(), convex_polygon.end(),
                        [](const point &p1, const point &p2){
                            return p1.x < p2.x;
                        });//左到右
            Polygon convex_edge;//凸多边形的各个顶点
            int now = 0;//在所有顶点中的位置
            int nowK = 1;//下一个边界点,在convex_polygon中的id
            point next_point{};
            next_point = convex_polygon[now];
            convex_edge.push_back(next_point);//先将第一个点加在边界点中，最左边一个点
            // find next edge
            while (0 != nowK)
            {
                findNextEdge(convex_polygon, now, nowK, convex_edge);
            }

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