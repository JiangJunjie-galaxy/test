//
// Created by sifan on 19-11-19.
//

//
// Created by sifan on 19-6-17.
//#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>

#include <nodelet/nodelet.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/Int8.h"
#include "std_msgs/Int64.h"

#include <string>
#include <mutex>
#include <memory>
#include <iostream>

#include<cmath>

#include <geometry_msgs/Twist.h>

#include <sstream>
#include <vector>
#include <fstream>

//#include <QCoreApplication>
//#include <LMS1xx.h>
#include <math.h>
#include <mlpack/methods/kmeans/kmeans.hpp>
#include <armadillo>


#define PI 3.1415926
#define DEG2RAD PI/180.0
#define RAD2DEG 180.0/PI

using namespace mlpack::kmeans;
using namespace arma;
using std::string;
using namespace std;
bool target_get,search_get;
int findelse();
double array1[12];
double array2[12];


ros::Publisher center_point;
ros::Publisher point_filter_pub;
ros::Publisher point_marker_pub;
void Lasercallback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{
    std::vector<float> ranges = msg->ranges;
    std::vector<float> intensity = msg->intensities;
    auto deg_resolution = msg->angle_increment;
    double min_angle = msg->angle_min;
    double max_angle = msg->angle_max;

    std::vector<float> pub_ranges = ranges;
    std::vector<float> pub_intensity = intensity;
    sensor_msgs::LaserScan scan_msg;
    scan_msg.angle_min = msg->angle_min;
    scan_msg.angle_max = msg->angle_max;
    scan_msg.angle_increment = msg->angle_increment;
    scan_msg.header = msg->header;
    scan_msg.scan_time = msg->scan_time;
    scan_msg.range_max = msg->range_max;
    scan_msg.range_min = 0;
    arma::mat dataset; //matrice
    dataset.resize(2, ranges.size());//这里要做什么
    dataset.zeros();

    int count =0;
    for(size_t i =0; i < ranges.size(); i++)
    {

        if(intensity[i] >1335)
        {
            std::cout << intensity[i] << " " << ranges[i] << std::endl;
            dataset(0, count) = ranges[i]*cos(min_angle + deg_resolution*i);
            dataset(1, count) = ranges[i]*sin(min_angle + deg_resolution*i);
            count++;
        }

        else
        {
            pub_ranges[i] =0;
        }
    }
    //发送第一次阈值后的点

    scan_msg.ranges = pub_ranges;
    scan_msg.intensities = intensity;
    point_filter_pub.publish(scan_msg);

    //Kmeans聚类
    dataset.resize(2, count);
    KMeans<> K;//全部选择默认
    size_t cluster;
    arma::Row<size_t> assignments;//聚类结果
    arma::mat centroid; //聚类的中心点
    //K.Cluster((arma::mat)dataset, 24, assignments, centroid);//将数据分成4类，assigenments储存每个数据对应的类别*****
    //centroid里面有四个点，八个坐标，一个点对应两个坐标x和y
    K.Cluster((arma::mat)dataset, 12, assignments, centroid);
    //std::cout << "the centroid number is " << centroid.size()<< std::endl;*****
    std::cout << "the centroid number is " << centroid.size()/2<< std::endl;

    
//    for(auto i=0; i< assignments.size();i++)
//    {
//        std::cout << "the class number is" << assignments.col(i)<< std::endl;
//    }

//    将Kmeans聚类出来的四个点用marker表示出来
    visualization_msgs::Marker points;
    points.header.frame_id = "scan";
    points.header.stamp = ros::Time::now();
    points.ns = "points";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 0.02;
    points.scale.y = 0.02;
    points.color.g = 1.0f;
    points.color.a = 1.0;
    geometry_msgs::Point p;
    std::cout<<"dataset_size "<<dataset.size()<<std::endl;
    std::cout<<"count "<<count<<std::endl;

    //提取数据
     for(int m=0;m<12;m++)
     {
         array1[m]=centroid(0,m);
     }
      for(int m=0;m<12;m++)
     {
         array2[m]=centroid(1,m);
     }
    //验证程序
    //   int find=0;
    //   find=findelse();
    //   cout<<"find=  "<<find<<endl;

    //将12个点合并成四个中心点
      double point1_x = 0;   //array1[0];
      double point1_y = 0;   //array2[0];
      double point2_x = 0;
      double point2_y = 0;
      double point3_x = 0;
      double point3_y = 0;
      double point4_x = 0;
      double point4_y = 0;
      
      double polar1_x = 0;
      double polar1_y = 0;
      double polar2_x = 0;
      double polar2_y = 0;
      double polar3_x = 0;
      double polar3_y = 0;
      double polar4_x = 0;
      double polar4_y = 0;

      double center_x = 0;
      double center_y = 0;
      
      int size1 =0;
      int size2 =0;
      int size3 =0;
      int size4 =0;
    

    //多层嵌套if语句实现分类
    for(int i=0 ; i<12 ; i++)
    {
        if((array1[0]-array1[i])*(array1[0]-array1[i]) +(array2[0]-array2[i])*(array2[0]-array2[i])<1)
        {
            point1_x += centroid(0,i);
            point1_y += centroid(1,i);
            size1++;
        }
        else if((array1[0]-array1[i])*(array1[0]-array1[i]) +(array2[0]-array2[i])*(array2[0]-array2[i])>4)
        {
            point4_x += centroid(0,i);
            point4_y += centroid(1,i);
            size4++;
        }
        else 
        {
            int find =findelse();
            if((array1[find]-array1[i])*(array1[find]-array1[i]) +(array2[find]-array2[i])*(array2[find]-array2[i])<1)
            {
                point2_x += centroid(0,i);
                point2_y += centroid(1,i);
                size2++;
            }
            else
            {
                point3_x += centroid(0,i);
                point3_y += centroid(1,i);
                size3++;
            }
        }
    }

         polar1_x = point1_x/size1;
         polar1_y = point1_y/size1;
         polar2_x = point2_x/size2;
         polar2_y = point2_y/size2;
         polar3_x = point3_x/size3;
         polar3_y = point3_y/size3;
         polar4_x = point4_x/size4;
         polar4_y = point4_y/size4;

            if(size2 == 0)
            {
                center_x = (polar1_x + polar4_x)/2;
                center_y = (polar1_y + polar4_y)/2;
                goto finish;
             }
            else if(size3 == 0)
            {
                center_x = (polar1_x + polar4_x)/2;
                center_y = (polar1_y + polar4_y)/2;
                goto finish;
             }
            else if(size4 == 0)
            {
                center_x = (polar2_x + polar3_x)/2;
                center_y = (polar2_y + polar3_y)/2;
                goto finish;
             }
             else 
             {
               
                    center_x = (polar1_x+polar2_x+polar3_x+polar4_x)/4;
                    center_y = (polar1_y+polar2_y+polar3_y+polar4_y)/4;
             }
             finish :
        //  if (size4 == 0)
        //  {
        //      center_x = (polar2_x + polar3_x)/2;
        //      center_y = (polar2_y + polar3_y)/2;
        //  }
        //  else
        //  {
        //      center_x = (polar1_x + polar4_x)/2;
        //      center_y = (polar1_y + polar4_y)/2;
        //  }
    
    for (uint32_t i=0; i<(centroid.size()/2); i++)
    {
        //p.x = centroid(0, i);
        //p.y = centroid(1, i);
        
        //四点聚合算法
        // p.x = (polar1_x+polar2_x+polar3_x+polar4_x)/4;
        // p.y = (polar1_y+polar2_y+polar3_y+polar4_y)/4;
       
       //两点聚合算法
        p.x = center_x;
        p.y = center_y;
        p.z = 0;
        points.points.push_back(p);
        
        int a = findelse();
        cout<<a<<endl;
        cout<<"---------------------"<<endl;
        cout<<size1<<endl;
        cout<<size2<<endl;
        cout<<size3<<endl;
        cout<<size4<<endl;
        
        
        cout<<"四个点坐标为"<<endl;
        cout<<point1_x/size1<<"     "<<point1_y/size1<<endl;
        cout<<point2_x/size2<<"     "<<point2_y/size2<<endl;
        cout<<point3_x/size3<<"     "<<point3_y/size3<<endl;
        cout<<point4_x/size4<<"     "<<point4_y/size4<<endl;
      
        cout<<"中心点的位置为"<<endl;
        cout<<center_x<<"     "<<center_y<<endl;
        cout<<"-----------------------------------"<<endl;
        
        for(int m=0 ; m<12 ; m++)
        {
        cout<<centroid(0,m)<<"   "<<centroid(1,m)<<endl;
        }
        
    
    }
      //points.points.push_back(p);
//    p.x = dataset(0, 0);
//    p.y = dataset(1, 0);
    
    point_marker_pub.publish(points);
    

// 1.computer center point
}



 //寻找和选取的第一个点差距大的另外一个X
    int findelse()
    {
        for(int i=0;i<12;i++)
        {
            if((array1[0]-array1[i])*(array1[0]-array1[i]) +(array2[0]-array2[i])*(array2[0]-array2[i]) >1.0 && 
                (array1[0]-array1[i])*(array1[0]-array1[i]) +(array2[0]-array2[i])*(array2[0]-array2[i]) <4.0)
            return i;
        }
    }
   

int main(int argc, char **argv)
{
    
    
    // // 验证程序 看能不能选出要求的i
    // int find=0;
    // find=findelse();
    // cout<<"x=  "<<find<<endl; 


    ros::init(argc, argv, "laserscan_receive");
    ros::NodeHandle nh;
    ros::Subscriber laserscan_sub = nh.subscribe("/r2000_node/scan", 1, Lasercallback);
    // center_point = nh.advertise<geometry_msgs::Twist>("/center_points", 10); //发送计算出来的中心点位置
    point_filter_pub = nh.advertise<sensor_msgs::LaserScan>("/scan1", 10);//这两句什么意思
    point_marker_pub = nh.advertise<visualization_msgs::Marker>("visulization_marker",10);
    ros::spin();

    return 0;
}

