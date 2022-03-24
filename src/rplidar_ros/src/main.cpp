#include"parameter.hpp"
#include"lidar.hpp"

mutex updateModelMutex;
/*
shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
        // --------------------------------------------
        // -----Open 3D viewer and add point cloud-----
        // --------------------------------------------
        shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
        //viewer->setCameraPosition(0, 0, 0, 0, 0, 0, 0, 0, -1);
        //viewer->setBackgroundColor(0, 0, 0);
        viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
        //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
        //viewer->initCameraParameters ();
        return (viewer);
}

void viewerRunner(shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
    }
} 

*/
vector<PointXYZ> breakPoint;
Lidar test_lidar;
visualization::CloudViewer viewer("viewer");
int viewer_update=0;
char line_char[20];
char sphere_char[20];
char text_char[20];
char distance_char[20];
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

            PointXYZ temp1(x1 / 3000, y1 / 3000, 0);
            PointXYZ temp2(x2 / 3000, y2 / 3000, 0);//拟合的直线端点

            PointXYZ temp3(startp.x / 3000, startp.y / 3000, 0);
            PointXYZ temp4(endp.x / 3000, endp.y / 3000, 0);//转折点

            PointXYZ temp5((-b * k) / (k * k + 1) / 6000, ((-b * k * k) / (k * k + 1) + b) / 6000, 0);//到直线的垂直中心点
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
            }*/

            viewer.addLine<PointXYZ>(temp1, temp2, 255 * (i % 3 == 0), 255 * (i % 3 == 1), 255 * (i % 3 == 2),
                                     line_str);
            viewer.addSphere<PointXYZ>(temp3, 0.003, 255, 255, 255, sphere_str1);
            viewer.addSphere<PointXYZ>(temp4, 0.003, 255, 255, 255, sphere_str2);
            viewer.addText(distance_char, 10, 200 - 40 * i, 30, (i % 3 == 0), (i % 3 == 1), (i % 3 == 2), text_char, 0);
        }

    }
}
//回调函数
void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    // num_readings = 扫描角 / 角度分辨率;根据雷达参数计算;
    // 创建一个存储激光雷达数据的数据空间
    float data[726]={};
    for(unsigned int i =0; i <726; ++i)
    {
        data[i]=scan->ranges[i];
    }
    test_lidar.extract_region(data);
    test_lidar.fit_line();
    viewer_update=1;
    viewer.showCloud(test_lidar.cloud);
    cout<<test_lidar.cloud->points.size()<<endl;

};

int main(int argc, char **argv)
{
    viewer.runOnVisualizationThreadOnce(viewerOneOff);
    viewer.runOnVisualizationThread(viewerPsycho);
    // ros打开激光雷达读取功能初始化
    ros::init(argc, argv, "laser_scan_publisher2");
    ros::NodeHandle n;
    // 连接回调函数，从/scan中订阅消息
    ros::Subscriber get_date = n.subscribe("/scan",681, chatterCallback);// 当/scan中积累681个数据的时候，调用上面回调函数
    ros::spin();
    return 0;
}
