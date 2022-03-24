#ifndef __PARA__
#define __PARA__
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <boost/thread/thread.hpp>

#include <boost/thread/thread.hpp>


#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/console/parse.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <fstream>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <cmath>
#include <thread>
#include <pthread.h>
#include <Eigen/Core>
// Eigen 几何模块
#include <Eigen/Geometry>
using namespace pcl;
using namespace cv;
using namespace std;
#define CIRCLE 2
#define LINE 1

#define missing_num 6//一段区域允许更丢的个数
#define maxDistance 4
#define minDistance 0.05
#define minGap 0.04//区域内相邻点的最大距离
#define minRegionpoints 10
#define queryNum 20
#define Angle_increment  0.25;//角分辨率
#define max_curvature 10
#define max_line_points 30
#define break_line_max_dis 100
#define radius_error 1
//#define Debug_region 0

#endif