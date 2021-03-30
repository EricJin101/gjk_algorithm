# gjk_algorithm learning
- 2D algorithm

## Introduction
- GJK 简写为 Gilbert–Johnson–Keerthi
- GJK算法在很大程度上依赖于称为闵科夫斯基和的概念。
    + 闵科夫斯基和
        * 设有两个形状，这些形状的闵科夫斯基和就是shape1中的所有点加到shape2中的所有点上
            {得到的是另外一个更大更复杂的形状}：
## 头铁
### 想法1
- 根据排列求出定点之间的坐标差
- 将三三点之间用三角形连起来，求原点在不在其中

### 想法2
- 求出最大的凸矩形
    + 从横坐标最小的点，斜率为正无限开始，依次寻找下一个坐标点。
    + 求出该点与其它所有点的斜率，找到小于当前斜率的前提下能形成最大斜率且横坐标大于等于该点的点。
    + 直到找到横坐标最大的点为止。再从该点，斜率为正无限开始，依次寻找下一个坐标点。
    + 具体过程为，求出该点与其它所有点的斜率，找到小于当前斜率的前提下能形成最大斜率且横坐标小于等于该点的点。
    + 直到找到初始位置为止。
      
- 求原点在不在这个凸矩形中
    + 通过射线法判断原点是否在凸多边形中
        * 原点在凸多边形的顶点上或者边上
        * 点在多边形的延长线上
        * 点的射线与多边形相交于多边形的顶点
    



### gjk推荐方法
- 选择一个向量方向(当前为默认x轴正方向)
- 将所有点投影到该向量上，求出最远端的点
- 求反方向的最大点
- 垂直方向再次计算
- > 不理解好处在哪里
  > 比三角形那个理论上会快一些，因为gjk选择的是最远的（最大面积）
- 若找不到可咋整？
* gjk优点在于，选择最大三角形(simplex)，下一个点选择在靠近原点方向的凸多边形的点
* > 增加三角形的方向性
