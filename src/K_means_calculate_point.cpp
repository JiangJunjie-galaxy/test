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
int num =0;
vector<double>lastx;
vector<double>lasty;

size_t last_size = 0;
int timer = 0;


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

        if(intensity[i] >1320)
        {
            std::cout << intensity[i] << " " << ranges[i] << std::endl;
            dataset(0, count) = ranges[i]*cos(min_angle + deg_resolution*i);
            dataset(1, count) = ranges[i]*sin(min_angle + deg_resolution*i);
            count++;
            //cout<<"第一个数据为"<<dataset(0,0)<<dataset(1,0)<<endl;
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
    points.scale.x = 0.005;
    points.scale.y = 0.005;
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
      int realsize = 12;

      
      //删除偏离的点
      for(int i=0 ; i<realsize ;i++)
      {

          for(int j=0 ; j<realsize ;j++)
          {
              if((array1[i]-array1[j])*(array1[i]-array1[j]) +(array2[i]-array2[j])*(array2[i]-array2[j])>8)
              {
                  for(j ; j<realsize ; j++)
                  {
                      array1[j]=array1[j+1];
                      array2[j]=array2[j+1];
                  }
                  realsize = realsize - 1 ;

                  for(i ; i<realsize ; i++)
                  {
                      array1[i]=array1[i+1];
                      array2[i]=array2[i+1];
                      
                  }

                  realsize = realsize - 1 ;
                      
                      j = 0;
                      i = 0;
              }
          }
      }
    

    //多层嵌套if语句实现分类
    for(int i=0 ; i<realsize ; i++)
    {
        if((array1[0]-array1[i])*(array1[0]-array1[i]) +(array2[0]-array2[i])*(array2[0]-array2[i])<1)
        {
            point1_x += array1[i];
            point1_y += array2[i];
            size1++;
        }
        else if((array1[0]-array1[i])*(array1[0]-array1[i]) +(array2[0]-array2[i])*(array2[0]-array2[i])>4)
        {
            point4_x +=  array1[i];
            point4_y +=  array2[i];
            size4++;
        }
        else 
        {
            int find =findelse();
            if((array1[find]-array1[i])*(array1[find]-array1[i]) +(array2[find]-array2[i])*(array2[find]-array2[i])<1)
            {
                point2_x +=  array1[i];
                point2_y +=  array2[i];
                size2++;
            }
            else
            {
                point3_x +=  array1[i];
                point3_y +=  array2[i];
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

         double theta1 = 0;
         double theta2 = 0;
         double distance1 = 0;
         double distance2 = 0;
         double supposex1 = 0;
         double supposey1 = 0;
         double supposex2 = 0;
         double supposey2 = 0;
         double suppose_centerx = 0;
         double suppose_centery = 0;
         double sumx = 0;
         double sumy = 0;


            if(size2 == 0)
            {
                theta1 = atan((polar1_x-polar3_x)/(polar3_y-polar1_y));
                distance1 = sqrt((polar3_x-polar4_x)*(polar3_x-polar4_x)+(polar3_y-polar4_y)*(polar3_y-polar4_y))/2;

                suppose_centerx = (polar1_x + polar4_x)/2;
                suppose_centery = (polar1_y + polar4_y)/2;

                supposex1 = (polar1_x+polar3_x)/2+cos(theta1)*distance1;
                supposey1 = (polar1_y+polar3_y)/2+sin(theta1)*distance1;

                supposex2 = (polar1_x+polar3_x)/2-cos(theta1)*distance1;
                supposey2 = (polar1_y+polar3_y)/2-sin(theta1)*distance1;

                if((suppose_centerx-supposex1)*(suppose_centerx-supposex1)+(suppose_centery-supposey1)*(suppose_centery-supposey1)<0.1)
                {
                    sumx = suppose_centerx + supposex1;
                    sumy = suppose_centery + supposey1;
                }
                else
                {
                    sumx = suppose_centerx + supposex2;
                    sumy = suppose_centery + supposey2;
                }

                theta2 = atan((polar3_x-polar4_x)/(polar4_y-polar3_y));
                distance2 = sqrt((polar1_x-polar3_x)*(polar1_x-polar3_x)+(polar1_y-polar3_y)*(polar1_y-polar3_y))/2;
                
                supposex1 = (polar4_x+polar3_x)/2+cos(theta2)*distance2;
                supposey1 = (polar4_y+polar3_y)/2+sin(theta2)*distance2;

                supposex2 = (polar4_x+polar3_x)/2-cos(theta2)*distance2;
                supposey2 = (polar4_y+polar3_y)/2-sin(theta2)*distance2;

                if((suppose_centerx-supposex1)*(suppose_centerx-supposex1)+(suppose_centery-supposey1)*(suppose_centery-supposey1)<0.1)
                {
                    sumx = sumx + supposex1;
                    sumy = sumy + supposey1;
                }
                else
                {
                    sumx = sumx + supposex2;
                    sumy = sumy + supposey2;
                }
                sumx = sumx/3;
                sumy = sumy/3;
                center_x = sumx;
                center_y = sumy;
                

                goto finish;
             }
            else if(size3 == 0)
            {
                suppose_centerx = (polar1_x + polar4_x)/2;
                suppose_centery = (polar1_y + polar4_y)/2;
                theta1 = atan((polar1_x-polar2_x)/(polar2_y-polar1_y));
                distance1 = sqrt((polar2_x-polar4_x)*(polar2_x-polar4_x)+(polar2_y-polar4_y)*(polar2_y-polar4_y))/2;


                supposex1 = (polar1_x+polar2_x)/2+cos(theta1)*distance1;
                supposey1 = (polar1_y+polar2_y)/2+sin(theta1)*distance1;

                supposex2 = (polar1_x+polar2_x)/2-cos(theta1)*distance1;
                supposey2 = (polar1_y+polar2_y)/2-sin(theta1)*distance1;

                if((suppose_centerx-supposex1)*(suppose_centerx-supposex1)+(suppose_centery-supposey1)*(suppose_centery-supposey1)<0.1)
                {
                    sumx = suppose_centerx + supposex1;
                    sumy = suppose_centery + supposey1;
                }
                else
                {
                    sumx = suppose_centerx + supposex2;
                    sumy = suppose_centery + supposey2;
                }

                theta2 = atan((polar2_x-polar4_x)/(polar4_y-polar2_y));
                distance2 = sqrt((polar1_x-polar2_x)*(polar1_x-polar2_x)+(polar1_y-polar2_y)*(polar1_y-polar2_y))/2;
                
                supposex1 = (polar4_x+polar2_x)/2+cos(theta2)*distance2;
                supposey1 = (polar4_y+polar2_y)/2+sin(theta2)*distance2;

                supposex2 = (polar4_x+polar2_x)/2-cos(theta2)*distance2;
                supposey2 = (polar4_y+polar2_y)/2-sin(theta2)*distance2;

                if((suppose_centerx-supposex1)*(suppose_centerx-supposex1)+(suppose_centery-supposey1)*(suppose_centery-supposey1)<0.1)
                {
                    sumx = sumx + supposex1;
                    sumy = sumy + supposey1;
                }
                else
                {
                    sumx = sumx + supposex2;
                    sumy = sumy + supposey2;
                }
                sumx = sumx/3;
                sumy = sumy/3;
                center_x = sumx;
                center_y = sumy;
                goto finish;
             }
            else if(size4 == 0)
            {
                suppose_centerx = (polar2_x + polar3_x)/2;
                suppose_centery = (polar2_y + polar3_y)/2;
                theta1 = atan((polar1_x-polar2_x)/(polar2_y-polar1_y));
                distance1 = sqrt((polar1_x-polar3_x)*(polar1_x-polar3_x)+(polar1_y-polar3_y)*(polar1_y-polar3_y))/2;


                supposex1 = (polar1_x+polar2_x)/2+cos(theta1)*distance1;
                supposey1 = (polar1_y+polar2_y)/2+sin(theta1)*distance1;

                supposex2 = (polar1_x+polar2_x)/2-cos(theta1)*distance1;
                supposey2 = (polar1_y+polar2_y)/2-sin(theta1)*distance1;

                if((suppose_centerx-supposex1)*(suppose_centerx-supposex1)+(suppose_centery-supposey1)*(suppose_centery-supposey1)<0.1)
                {
                    sumx = suppose_centerx + supposex1;
                    sumy = suppose_centery + supposey1;
                }
                else
                {
                    sumx = suppose_centerx + supposex2;
                    sumy = suppose_centery + supposey2;
                }

                theta2 = atan((polar1_x-polar3_x)/(polar3_y-polar1_y));
                distance2 = sqrt((polar1_x-polar2_x)*(polar1_x-polar2_x)+(polar1_y-polar2_y)*(polar1_y-polar2_y))/2;
                
                supposex1 = (polar1_x+polar3_x)/2+cos(theta2)*distance2;
                supposey1 = (polar1_y+polar3_y)/2+sin(theta2)*distance2;

                supposex2 = (polar1_x+polar3_x)/2-cos(theta2)*distance2;
                supposey2 = (polar1_y+polar3_y)/2-sin(theta2)*distance2;

                if((suppose_centerx-supposex1)*(suppose_centerx-supposex1)+(suppose_centery-supposey1)*(suppose_centery-supposey1)<0.1)
                {
                    sumx = sumx + supposex1;
                    sumy = sumy + supposey1;
                }
                else
                {
                    sumx = sumx + supposex2;
                    sumy = sumy + supposey2;
                }
                sumx = sumx/3;
                sumy = sumy/3;
                center_x = sumx;
                center_y = sumy;
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
    
    //for (uint32_t i=0; i<(centroid.size()/2); i++)
    //{
        //p.x = centroid(0, i);
        //p.y = centroid(1, i);
        
        //四点聚合算法
        // p.x = (polar1_x+polar2_x+polar3_x+polar4_x)/4;
        // p.y = (polar1_y+polar2_y+polar3_y+polar4_y)/4;
       
       
        
        //滤波
        lastx.push_back(center_x);
        lasty.push_back(center_y);
        
        if(num == 0)
        {
            center_x=lastx[num];
            center_y=lasty[num];
            num++;
        }
        else
        {
            if((center_x-lastx[num-1])*(center_x-lastx[num-1])+
            (center_y-lasty[num-1])*(center_y-lasty[num-1])<0.008)
            {
               center_x=lastx[num];
               center_y=lasty[num];
               num++;
            }
            else
            {
               center_x=lastx[num-1];
               center_y=lasty[num-1];
               lastx.pop_back(); 
               lasty.pop_back();
            }

        }

        // //判断是否要删除容器
            
            
            
            // {
            //         //   double a = center_x;
            //         //   double b = center_y;
            //           lastx.clear();
            //           lasty.clear();
            //         //   lastx.push_back(a);
            //         //   lasty.push_back(b);
            //           num = 0;
            // }
        
        
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
        
    
    //}
      //points.points.push_back(p);
//    p.x = dataset(0, 0);
//    p.y = dataset(1, 0);
    
    point_marker_pub.publish(points);
    

// 1.computer center point
}

     

 //寻找和选取的第一个点差距大的另外一个点
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
    ros::Rate loop(200);
    
    
    while(ros::ok())
    {        
       //num = 0;
    //    static size_t last_size = 0;
    //    static int cnt = 0;
       
       //判断数据包是否结束

       if(last_size=lastx.size())
       {

           timer++;

       }
       //如果经过6秒，容器的size都没有增加，说明数据包已经结束
       if(timer>300)
       {
           lastx.clear();
           lasty.clear();
           num = 0;
           timer = 0;
       }
        last_size =lastx.size();
       ros::spinOnce();
       //一个ros::Rate对象允许你制定循环的频率。它将会记录从上次调用Rate::sleep()到现在为止的时间，并且休眠正确的时间。在这个例子中，设置的频率为10hz。
       loop.sleep();
    }
    // ros::spin();


    return 0;
}

