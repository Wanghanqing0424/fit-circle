#ifndef __LIDAR__
#define __LIDAR__
#include "parameters.hpp"
struct Region
{
    vector<Point2f> points;
    int mark;//按照扫描顺序排列
};

struct Line
{
    vector<Point2f> points;
    float k,b;
};

struct myCircle
{
    struct Region *region;
    Point2f center;
    float diameter;
    float L0_error=0;
    float angle=0;//初始点到末尾点的夹角
};

class Lidar
{
public:
    Lidar();
    vector<Line> Lines;
    vector<myCircle> circles;
    vector<Point2f> lidar_points;
    PointCloud<PointXYZRGB>::Ptr cloud;
    vector<Region> regions;

    void extract_region(vector<float> sub_scan);
    void fit_line();
    void get_scan(vector<float> sub_scan);
    int calculate_radius();
private:
    myCircle fit_circle(struct Region &);
    float get_distance(float x1,float y1,float x2,float y2) const;
    //Point2f get_curvature_angle(vector<Point2f> points);
    void BreakPolyLine(vector<Point2f> points);
    Point2f get_line_kb(vector<Point2f> points);
    void select_lines();
    //拟合折线,返回直线的k,b和截止的点索引
    //bool Lidar::judge_line(vector<point2f> curvature_angle);
};


#endif 
