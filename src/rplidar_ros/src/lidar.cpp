#include"lidar.hpp"
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
Lidar::Lidar()
{
    cloud=PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>);
}
void Lidar::get_scan(vector<float> sub_scan)//获取一帧
{
    double d_angle =CV_PI / 180 * Angle_increment;
    cloud->points.clear();

    for (int n = 0; n < sub_scan.size(); n++)
    {
        float distance = sub_scan[n];  //获取距离数据/
        //cout<<value<<endl;
        //cout<<sub_scan[n]<<endl;
        if (distance <= maxDistance && distance > minDistance)//当距离信息在一定范围时进行下面操作 注：1.当没有检测到信息时，data=0；2.激光雷达外壳可能有干扰，此时data<10;
        {
            double R = distance;

            double Theta = d_angle*(n);
            double j = R * cos(-Theta - CV_PI / 2);
            double i = R * sin(-Theta - CV_PI / 2);

            PointXYZRGB temp_point;
            temp_point.x=j;
            temp_point.y=i;
            temp_point.z=0;
            temp_point.r=255;
            temp_point.g=255;
            temp_point.b=255;
            cloud->points.push_back(temp_point);
        }
    }
}

float Lidar::get_distance(float x1,float y1,float x2,float y2) const
{
    return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}

Point2f Lidar::get_line_kb(vector<Point2f> points)
{
    cv::Vec4f line_para; 
	cv::fitLine(points, line_para, cv::DIST_L2, 0, 1e-2, 1e-2);
    float k = line_para[1] / line_para[0];
    float b = line_para[3]-k*line_para[2];
    //float temp=sqrt(k*k+1);

    vector<pair<float, int> > vPairs;
    for(int i=0;i<points.size();i++)
    {
        vPairs.push_back(make_pair(abs(k*points[i].x-points[i].y+b),i));
    }
    sort(vPairs.begin(),vPairs.end());

    vector<Point2f> temp_points;
    for(int i=0;i<int(points.size()*0.8);i++)//舍去离群点
    {
        temp_points.push_back(points[vPairs[i].second]);
    }

    cv::fitLine(temp_points, line_para, cv::DIST_L2, 0, 1e-2, 1e-2);//再拟合一遍
    k = line_para[1] / line_para[0];
    b = line_para[3]-k*line_para[2];

	return Point2f(k,b);
}
void Lidar::extract_region(vector<float> sub_scan_all)//分割区域
{
    double d_angle =CV_PI / 180 * Angle_increment;

    //cloud->points.clear();
    int missing_points=0;
    int region_points=0;
    int mark=0;
    vector<Point2f> temp;//装一段线
    Point2f last_point;
    regions.clear();
    lidar_points.clear();    
    //由于激光雷达的角度是从正前方开始扫一圈，为了防止正前方左右两边将物体轮廓分开,把左右90度拼接起来

    vector<float> sub_scan(sub_scan_all.begin()+int(3*1.0/4*sub_scan_all.size()),
    sub_scan_all.end()-1);
    sub_scan.insert(sub_scan.end(),sub_scan_all.begin(),
    sub_scan_all.begin()+int(1.0/4*sub_scan_all.size()));
    
    //struct timeval time1,time2;
   // int t0,t1,t2,t3;
    //需要一段区域足够长，中断点不多
    //gettimeofday(&time1,NULL);
    for (int n = 0; n <sub_scan.size() ; n++)
    {
        float distance = sub_scan[n];  //获取距离数据/
        //cout<<value<<endl;
        //cout<<sub_scan[n]<<endl;
        if (distance <= maxDistance && distance > minDistance)//当距离信息在一定范围时进行下面操作 注：1.当没有检测到信息时，data=0；2.激光雷达外壳可能有干扰，此时data<10;
        {
            double R = distance;
            double Theta = d_angle*(n);
            double j = R * cos(Theta);
            double i = R * sin(Theta);//y
            //cout<<i<<" "<<j<<endl;
            
            if(!temp.empty()&&!missing_points)//还没断的时候
            {
                last_point=temp.back();
                //cout<<get_distance(last_point.x,last_point.y,j,i)<<endl;
                if(get_distance(last_point.x,last_point.y,j,i)>minGap)//这一段第一次断
                {
                    missing_points++;
                }
                else
                {
                    temp.emplace_back(j,i);
                    region_points++;
                }
            }
            
            if(!temp.empty()&&missing_points>0)//断了
            {
                if(get_distance(last_point.x,last_point.y,j,i)>(missing_points+1)*minGap)
                {
                    missing_points++;
                }
                else
                {
                    missing_points=0;
                    region_points++;
                    temp.emplace_back(j,i);
                }
        
                if(missing_points>missing_num)
                {
                    if (region_points > minRegionpoints)//形成一段区域
                    {
                        Region temp_region;
                        temp_region.points = temp;
                        temp_region.mark = mark;
                        regions.push_back(temp_region);
                        missing_points=0;
                        temp.clear();
                        mark++;
                    }
                    region_points=0;
                    missing_points=0;
                }
            }
                
            if(temp.empty())
            {
                temp.emplace_back(j,i);
                missing_points=0;
                region_points=0;
            }

            lidar_points.emplace_back(j,i);
        }
        else if(!temp.empty())
        {
            missing_points++;
            if(missing_points>missing_num)
            {
                if (region_points > minRegionpoints)//形成一段区域
                {
                    Region temp_region;
                    temp_region.points = temp;
                    temp_region.mark = mark;
                    regions.push_back(temp_region);
                    temp.clear();
                    mark++;
                }
                missing_points=0;
                region_points=0;
            }
        }
    }
   
    if(region_points>minRegionpoints)
    {
        Region temp_region;
        temp_region.points=temp;
        temp_region.mark=mark;
        regions.push_back(temp_region);
    }

    cloud->points.clear();
    for (int i = 0; i <regions.size(); i++) 
    {  
        Region temp_r=regions[i];
        for(int j = 0; j < temp_r.points.size();j++)
        {
            PointXYZRGB temp_point;  
            temp_point.x=temp_r.points[j].x;
            temp_point.y=temp_r.points[j].y;
            temp_point.z=0;
            temp_point.r=(i%3==0)*255;
            temp_point.g=(i%3==1)*255;
            temp_point.b=(i%3==2)*255;
                /*
                if(temp_r.mark%3==0)
                {
                    cloud->points.emplace_back(1.0*x/3000,1.0*y/3000,0,255,0,0);
                }
                else if(temp_r.mark%3==1)
                {
                    cloud->points.emplace_back(1.0*x/3000,1.0*y/3000,0,0,255,0);
                }
                else
                {
                    cloud->points.emplace_back(1.0*x/3000,1.0*y/3000,0,0,0,255);
                }*/
                
            cloud->points.push_back(temp_point);
        }
    }
}
    //gettimeofday(&time2,NULL);
    //t0=time2.tv_sec*1000+time2.tv_usec/1000-time1.tv_sec*1000-time1.tv_usec/1000;
    //cout<<t0<<endl;

myCircle Lidar::fit_circle(Region &temp_r)//参考https://blog.csdn.net/liyuanbhu/article/details/50889951
{
    /*
    double X2,X1,X1Y1,Y1,X3,X1Y2,Y2,X2Y1,Y3;
    X2=X1=X1Y1=Y1=X3=X1Y2=Y2=X2Y1=Y3=0;
    double a,b,c,C,D,E,G,H,A,B,R;
    C=D=E=G=H=a=b=c=0;
    int N = temp_r.points.size();
    for(int i=0;i<temp_r.points.size();i++)
    {
        X1+=temp_r.points[i].x;
        Y1+=temp_r.points[i].y;
        X2+=temp_r.points[i].x*temp_r.points[i].x;
        Y2+=temp_r.points[i].y*temp_r.points[i].y;
        X1Y1+=temp_r.points[i].y*temp_r.points[i].x;
        X1Y2+=temp_r.points[i].y*temp_r.points[i].x*temp_r.points[i].y;
        X2Y1+=temp_r.points[i].x*temp_r.points[i].x*temp_r.points[i].y;
        X3+=temp_r.points[i].x*temp_r.points[i].x*temp_r.points[i].x;
        Y3+=temp_r.points[i].y*temp_r.points[i].y*temp_r.points[i].y;
    }
    //cout<<X3<<endl;
    C = N*X2 - X1*X1;
    D = N*X1Y1 - X1*Y1;
    E = N*X3 + N*X1Y2 - (X2+Y2)*X1;
    G = N*Y2 - Y1*Y1;
    H = N*X2Y1 + N*Y3 - (X2+Y2)*Y1;
    a = (H*D-E*G)/(C*G-D*D);
    b = (H*C-E*D)/(D*D-G*C);
    c = -(a*X1 + b*Y1 + X2+Y2)/N;
    
    A = a/(-2);
    B = b/(-2);
    R = sqrt(a*a+b*b-4*c)/2;*/

	







    double A,B,R;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
for(int j=0;j<temp_r.points.size();j++)
{
     PointXYZ temp_point;  
    temp_point.x=temp_r.points[j].x;
    temp_point.y=temp_r.points[j].y;
     temp_point.z=0;
    cloud->points.push_back(temp_point);
}

	//----------------------RANSAC框架----------------------------   
	pcl::SampleConsensusModelCircle2D<pcl::PointXYZ>::Ptr circle2D(new pcl::SampleConsensusModelCircle2D<pcl::PointXYZ>(cloud));
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(circle2D);	// 定义RANSAC算法对象
	ransac.setDistanceThreshold(0.01);	// 设置距离阈值
	ransac.setMaxIterations(1000);		// 设置最大迭代次数
	ransac.computeModel();				// 拟合二维圆

	vector<int> inliers;				// 用于存放内点索引的vector
	ransac.getInliers(inliers);			// 获取内点索引

	//------------根据内点索引提取拟合的二维圆点云----------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr circle_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *circle_cloud);

	//--------------输出模型参数：圆心坐标和半径------------------
	Eigen::VectorXf coeff;
	ransac.getModelCoefficients(coeff);
	//cout << "圆心的x坐标为：" << coeff[0] << "\n"
	//	<< "圆心的y坐标为：" << coeff[1] << "\n"
	//	<< "圆的半径为：" << coeff[2] << endl;
A=coeff[0];
B=coeff[1];
R=coeff[2] ;


    //cout<<2*R<<endl;
    vector<double> bias(temp_r.points.size());
    myCircle temp_c;
    for(int i=0;i<temp_r.points.size();i++)
    {
       // cout<<fabs((x_all[i]-A)*(x_all[i]-A)+(y_all[i]-B)*(y_all[i]-B)-R*R)<<endl;
        bias[i]=fabs(sqrt((temp_r.points[i].x-A)*(temp_r.points[i].x-A)+(temp_r.points[i].y-B)*(temp_r.points[i].y-B))-R);
        temp_c.L0_error+=bias[i];
    }
    temp_c.L0_error/=bias.size();
    vector<double> bias_temp=bias;
    Region all_temp;
    sort(bias.begin(),bias.end()-1);
    
    for(int i=0;i<temp_r.points.size();i++)
    {
        if(bias_temp[i]<=bias[int(0.8*temp_r.points.size())])
        {
            all_temp.points.push_back(temp_r.points[i]);
        }
    }

    float startx=temp_r.points[0].x;
    float starty=temp_r.points[0].y;
    float endx=temp_r.points[temp_r.points.size()-1].x;
    float endy=temp_r.points[temp_r.points.size()-1].y;
    float d1=get_distance(startx,starty,A,B);
    float d2=get_distance(endx,endy,A,B);

    temp_c.angle=180*acos(((startx-A)*(endx-A)+(starty-B)*(endy-B))/(d2*d1))/CV_PI;
    temp_c.center=Point2f(A,B);
    temp_c.diameter=2*R;
    temp_c.L0_error/=R;

    
    temp_r=all_temp;
    return temp_c;
}

int Lidar::calculate_radius()
{
    circles.clear();
    for(int i=0;i<regions.size();i++)
    {
        Point2f start_point=regions[i].points[0];
        Point2f end_point=regions[i].points[int(regions[i].points.size()-1)];
        double d=get_distance(start_point.x,start_point.y,end_point.x,end_point.y);
        if(d<0.08||d>0.6)//圆弧两点距离太远或者太近
        {
            continue;
        }

        fit_circle(regions[i]);
        myCircle temp_c=fit_circle(regions[i]);

        if(temp_c.diameter>0.15&&temp_c.diameter<0.6&&temp_c.angle>90&&
        temp_c.L0_error<radius_error)
        {
            temp_c.region=&(regions[i]);
            
            for(int j=0;j<360;j++)//画圆
            {
                PointXYZRGB temp_p;

                double x=temp_c.diameter*cos(j*CV_PI/180)/2+temp_c.center.x;
                double y=temp_c.diameter*sin(j*CV_PI/180)/2+temp_c.center.y;
                temp_p.x=x;
                temp_p.y=y;
                temp_p.z=0;
                temp_p.r=155;
                temp_p.g=0;
                temp_p.b=255;
               cloud->points.push_back(temp_p);
            }
            circles.push_back(temp_c);
        }
    }
    return 0;
}


void Lidar::fit_line()
{
    Lines.clear();
    for(int i=0;i<regions.size();i++)
    {
        Region temp_region=regions[i];
        if(temp_region.points.size()>50)
        {
            
            //BreakPolyLine(temp_region.points);//拆分折线
        }
    }

    //select_lines();//筛选直线
}

void Lidar::BreakPolyLine(vector<Point2f> points)//拆分折线，先不把直线方程求出来
{
    Point2f endp=points.back();
    Point2f startp=points[0];
    vector<Point3f> line_para;
    //将折线的端点连线旋转到与x轴平行，这样便于计算点到连线的距离
    double dist=get_distance(endp.x,endp.y,startp.x,startp.y);
    double cosTheta = (endp.x - startp.x) / dist;  
    double sinTheta = - (endp.y - startp.y) / dist;  
    Line templine;
    double MaxDis = 0;  
    int i ;  
    int MaxDisInd1 = -1;  
    
    for(int i=0;i<points.size();i++)
    {
        double dbDis = abs( (points[i].y - startp.y) * cosTheta + (points[i].x - startp.x)* sinTheta); 
        if(MaxDis<dbDis)
        {
            MaxDis=dbDis;
            MaxDisInd1=i;
        }
    }
    
    if(MaxDis<break_line_max_dis)//只有一条直线
    {
        //Point2f kb = {0,0};
        //Point2f kb = get_line_kb(points);
        templine.points=points;templine.k=0;templine.b=0;
        Lines.push_back(templine);
        return;
    }
    
    vector<Point2f> points1,points2,points3,points_temp;
    points1.insert(points1.end(),points.begin(),points.begin()+MaxDisInd1);
    points2.insert(points2.end(),points.begin()+MaxDisInd1+1,points.end());
    
    int MaxDisInd2=0;
    for(int i=0;i<2;i++)//最多找到拟合第三条折线就可以结束了
    {
        if(i==0)
            points_temp=points1;
        else
            points_temp=points2;
        
        endp=points_temp.back();
        startp=points_temp[0];
        //将折线的端点连线旋转到与x轴平行，这样便于计算点到连线的距离
        dist=get_distance(endp.x,endp.y,startp.x,startp.y);
        cosTheta = (endp.x - startp.x) / dist;  
        sinTheta = - (endp.y - startp.y) / dist;  
        
        double MaxDis = 0;  
    
        int MaxDisInd = -1;  
        
        for(int j=0;j<points_temp.size();j++)
        {
            double dbDis = abs( (points_temp[j].y - startp.y) * cosTheta + (points_temp[j].x - startp.x)* sinTheta); 
            if(MaxDis<dbDis)
            {
                MaxDis=dbDis;
                MaxDisInd2=j;
            }
        }
        
        if(MaxDis>break_line_max_dis)//需要分成两段
        {
            MaxDisInd2=MaxDisInd2+i*MaxDisInd1;
            break;
        }
        else
        {
            MaxDisInd2=0;
        }
    }
    
    if(MaxDisInd2==0)//只有两段线
    {
        //Point2f kb1 = get_line_kb(points1);
        //Point2f kb2 = get_line_kb(points2);
        templine.points=points1;templine.k=0;templine.b=0;
        Lines.push_back(templine);
        templine.points=points2;templine.k=0;templine.b=0;
        Lines.push_back(templine);
        return;
    }
    
    points1.clear();points2.clear();
    if(MaxDisInd2<MaxDisInd1)//三段线
    {
        int temp=MaxDisInd2;
        MaxDisInd2=MaxDisInd1;
        MaxDisInd1=temp;
    }
    
    points1.insert(points1.end(),points.begin(),points.begin()+MaxDisInd1);
    points2.insert(points2.end(),points.begin()+MaxDisInd1+1,points.begin()+MaxDisInd2);
    points3.insert(points3.end(),points.begin()+MaxDisInd2+1,points.end());
    //Point2f kb1 = get_line_kb(points1);
    //Point2f kb2 = get_line_kb(points2);
    //Point2f kb3 = get_line_kb(points3);
    
    templine.points=points1;templine.k=0;templine.b=0;
    Lines.push_back(templine);
    templine.points=points2;templine.k=0;templine.b=0;
    Lines.push_back(templine);
    templine.points=points3;templine.k=0;templine.b=0;
    Lines.push_back(templine);
}

/*
Point2f Lidar::get_curvature_angle(vector<Point2f> points)//取得一个点的曲率和主方向
{
    Point2f aver_points;
    for(int i=0;i<points.size();i++)
    {
        aver_points+=points[i];
    }
    aver_points=Point2f(aver_points.x/points.size(),aver_points.y/points.size());
    float covxx=0,covxy=0,covyy=0;//协方差
    for(int i=0;i<points.size();i++)
    {
        covxx+=(points[i].x-aver_points.x)*(points[i].x-aver_points.x);
    }
    covxx/=points.size();
    for(int i=0;i<points.size();i++)
    {
        covyy+=(points[i].y-aver_points.y)*(points[i].y-aver_points.y);
    }
    covyy/=points.size();
    for(int i=0;i<points.size();i++)
    {
        covxy+=(points[i].y-aver_points.y)*(points[i].x-aver_points.x);
    }
    covxy/=points.size();
    Matrix2f Cov;
    Cov << covxx,covxy,covxy,covyy;
    SelfAdjointEigenSolver<Matrix2f> eigensolver(Cov);
    //Point2f direction;//主方向
    double EigenValue1 = eigensolver.eigenvalues()[0];
    double EigenValue2 = eigensolver.eigenvalues()[1];
    double curvature=min(EigenValue1,EigenValue2)/max(EigenValue1,EigenValue2);//曲率
    Matrix2f eigenvectors = eigensolver.eigenvectors();
    float angle;
    if(EigenValue1>EigenValue2)
    {
        angle=180*atan2(eigenvectors(0,1),eigenvectors(0,0))/CV_PI;
    }
    else
    {
        angle=180*atan2(eigenvectors(1,1),eigenvectors(1,0))/CV_PI;
    }
    if(angle<0)
        angle+=180;

    return Point2f(curvature,angle);

}

/*
void Lidar::select_lines()//通过长度、曲率和角度分布筛选直线
{
    vector<Line> temp_Lines;
    for(int i=0;i<Lines.size();i++)
    {
        if(lines[i].points.size()<max_line_points)//长度
            continue;

        //用kdtree加快邻近点搜索
        Mat source = cv::Mat(Lines[i].points).reshape(1);
        cv::flann::KDTreeIndexParams indexParams(2);//二维
        cv::flann::Index kdtree(source, indexParams);
        cv::flann::SearchParams params(32);//这个参数不知道什么意思
        vector<Point2f> curvature_angle;//曲率和角度

        for (int j = 0; j < Lines[i].points.size(); j++)
        {
            vector<float> query_point(2);
            vector<int> vecindex(queryNum);
            vector<float> vecdist(queryNum);

            query_point = {temp_region.points[j].x, temp_region.points[j].y};
            kdtree.knnSearch(query_point, vecindex, vecdist, queryNum, params);
            vector<Point2f> neighbor_point;//这个点附近的几个点
            for (int z = 0; z < queryNum; z++)
            {
                neighbor_point.push_back(temp_region.points[vecindex[z]]);
            }
            neighbor_point.push_back(temp_region.points[j]);

            Point2f temp_curvature_angle = get_curvature_angle(neighbor_point);//得到角度和曲率
            curvature_angle.push_back(temp_curvature_angle);
        }


        //用曲率筛掉s形状的线，这种线可能角度分布很一致
        int line_point = 0;
        for (int j = 0; j < curvature_angle.size(); j++)
        {
            if (curvature_angle[j].x < 0.05)
                line_point++;
        }
        if (1.0 * line_point / temp_region.points.size() < 0.90)//折线不够直
            continue;

        //用角度分布筛掉弧线，这种线可能曲率不大
        int hist[18]={};//角度分布直方图，每10度一个区域
        for(int j=0;j < curvature_angle.size();j++)
        {
            hist[int(curvature_angle.y/10)]++;
        }

        int max_hist=hist[0]+hist[17];//找到点数最多的两个相邻区域
        for(int j=1;j<17;j++)
        {
            if(max_hist<hist[j]+hist[j+1])
                max_hist=hist[j]+hist[j+1]
        }

        if(max_hist<0.9*Lines[i].points.size)
            continue;


        temp_Lines.push_back(Lines[i]);
    }

    Lines=temp_Lines;
}*/


