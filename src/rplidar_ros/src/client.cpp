#include"parameters.hpp"
#include"lidar.hpp"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#define RAD2DEG(x) ((x)*180./M_PI)
using namespace std;

vector<PointXYZ> breakPoint;
Lidar test_lidar;
visualization::CloudViewer viewer("viewer");
int viewer_update=0;
char line_char[20];
char sphere_char[20];
char text_char[20];
char distance_char[20];

char circle_char[20];
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);//放到这里没用
    /*
	viewer.setBackgroundColor(0, 0, 0);
	pcl::PointXYZ o;
	o.x = 1.0;
	o.y = 0;
	o.z = 0;
	viewer.addSphere(o, 0.25, "Sphere", 0);
	std::cout << "I only run once" << std::endl;
    */

}
char cc=0;
char startviewer=1;//可视化雷达图像
void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
    if (startviewer)
    {
        //viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
        // startviewer=0;
    }
    if (viewer_update)
    {
        //static unsigned count = 0;
        viewer.removeAllShapes();
        viewer_update = 0;

        for(int i=0;i<test_lidar.circles.size();i++)
        {
            PointXYZRGB temp_p;
            temp_p.x=test_lidar.circles[i].center.x;
            temp_p.y=test_lidar.circles[i].center.y;
            temp_p.z=0;
            temp_p.r=100;
            temp_p.g=0;
            temp_p.b=100;

       //     sprintf(sphere_char, "sphere%d", i);
            string sphere_str1(sphere_char);

            //viewer.addSphere<PointXYZRGB>(temp_p, test_lidar.circles[i].diameter/2, 0, 1, 0, sphere_str1);


          //  sprintf(text_char, "x:%.3f y:%.3f d:%.1f", temp_p.x, temp_p.y,1000*test_lidar.circles[i].diameter);
            string text_str(text_char);
            viewer.addText3D<PointXYZRGB>(text_str,temp_p , 0.03, 1, 1, 1, text_char);
        }
        /*
        for (int i = 0; i < test_lidar.Lines.size(); i++)//画出拟合的线段
        {
            float k = test_lidar.Lines[i].k;
            float b = test_lidar.Lines[i].b;
            Point2f startp = test_lidar.Lines[i].points.front();
            Point2f endp = test_lidar.Lines[i].points.back();
            float x1 = ((startp.y - b) * k + startp.x) / (k * k + 1);
            float x2 = ((endp.y - b) * k + endp.x) / (k * k + 1);
            float y1 = k * x1 + b;
            float y2 = k * x2 + b;

            PointXYZ temp1(x1 , y1 , 0);
            PointXYZ temp2(x2 , y2 , 0);//拟合的直线端点

            PointXYZ temp3(startp.x , startp.y , 0);
            PointXYZ temp4(endp.x , endp.y , 0);//转折点

            PointXYZ temp5((-b * k) / (k * k + 1) / 2, ((-b * k * k) / (k * k + 1) + b) / 2, 0);//到直线的垂直中心点
            float distance = abs(b / sqrt(k * k + 1));

            Line temp_line = test_lidar.Lines[i];
            sprintf(line_char, "line%d", i);
            string line_str(line_char);
            sprintf(sphere_char, "sphere%d", 2 * i);
            string sphere_str1(sphere_char);
            sprintf(sphere_char, "sphere%d", 2 * i + 1);
            string sphere_str2(sphere_char);
            sprintf(text_char, "text%d", i);
            sprintf(distance_char, "%.1f", distance);


            /*
            for(int j=0;j<temp_line.points.size();j++)
            {
                PointXYZRGB temp;
                temp.x=temp_line.points[j].x/3000+0.001;
                temp.y=temp_line.points[j].y/3000+0.001;
                temp.z=0;temp.r=255*(i%3==0);temp.g=(i%3==1);temp.b=(i%3==2);
                test_lidar.cloud->points.push_back(temp);
            }

            viewer.addLine<PointXYZ>(temp1, temp2, 255 * (i % 3 == 0), 255 * (i % 3 == 1), 255 * (i % 3 == 2),
                                     line_str);
            viewer.addSphere<PointXYZ>(temp3, 0.003, 255, 255, 255, sphere_str1);
            viewer.addSphere<PointXYZ>(temp4, 0.003, 255, 255, 255, sphere_str2);
            viewer.addText(distance_char, 10, 200 - 40 * i, 30, (i % 3 == 0), (i % 3 == 1), (i % 3 == 2), text_char, 0);
        }*/

    }
}


void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = scan->scan_time / scan->time_increment;
    //ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
    //ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
    //角分辨率0.25度,每次大约有1440个数据,数据从180度开始，也就是指向正后方开始顺时针转
    //std::cout<<count<<std::endl; 
   // cout<<"1"<<endl;
    vector<float> scan_temp(count,0);
    for(int i = 0; i < count; i++) {
        scan_temp[i]=scan->ranges[i];
        //float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        //ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
    }
    viewer_update = 1;
    test_lidar.extract_region(scan_temp);
    test_lidar.calculate_radius();
    viewer.showCloud(test_lidar.cloud);

    

}

int main(int argc, char **argv)
{
    //viewer.runOnVisualizationThreadOnce(viewerOneOff);//只在初始化调用一次
    viewer.runOnVisualizationThread(viewerPsycho);//不断调用
    ros::init(argc, argv, "rplidar_node_client");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

    ros::spin();

    return 0;
}
