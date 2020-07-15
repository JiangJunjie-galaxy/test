
//#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
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
#include <geometry_msgs/Point.h>

#include <sstream>
#include <vector>
#include <fstream>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>



#include <math.h>
#include"motion.h"
#include"landmark.h"




#define PI 3.1415926
#define DEG2RAD PI/180.0
#define RAD2DEG 180.0/PI

//using namespace mlpack::kmeans;
//using namespace arma;
using std::string;
using namespace std;
using namespace Eigen;
int real_total = 0;
vector<double>rea_distance;
vector<double>rea_angle;
vector<float>rea_polarx;
vector<float>rea_polary;
double marker_std_x = 0.1;
double marker_std_y = 0.1;
std::vector<geometry_msgs::Point> marker_truth;
Matrix<double, 3, 3> primitive_cov;
motion old_motion;
motion now_motion;
motion prediction_motion;
motion real_motion;
odom_parameter odom_parameter; 





ros::Publisher center_point;
ros::Publisher point_filter_pub;
ros::Publisher point_marker_pub;
ros::Publisher cal_odom_pub;
ros::Publisher real_points_pub;
ros::Publisher pre_odom_pub;
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
    //arma::mat dataset; //matrice
    //dataset.resize(2, ranges.size());//这里要做什么
    //dataset.zeros();

    
     float array1[ranges.size()];
     float array2[ranges.size()];
     vector<float>data_x;
     vector<float>data_y;

     vector<float>radius;
     vector<float>reflector_angle;
     int count =0;
    
    for(size_t i =0; i < ranges.size(); i++)
    {

        if(intensity[i] >900)
        {
            //std::cout << intensity[i] << " " << ranges[i] << std::endl;
            
                // dataset(0, count) = ranges[i]*cos(min_angle + deg_resolution*i);
                // dataset(1, count) = ranges[i]*sin(min_angle + deg_resolution*i);

                data_x.push_back(ranges[i]*cos(min_angle + deg_resolution*i));
                data_y.push_back(ranges[i]*sin(min_angle + deg_resolution*i));
                radius.push_back(ranges[i]);
                reflector_angle.push_back(min_angle + deg_resolution*i);
                
             count++;

        }

        else
        {
            pub_ranges[i] =0;
        }
    }
    
         //提取数据
             for(int i=0 ; i<count ; i++)
             {
                //   array1[i]=dataset(0,i);
                //   array2[i]=dataset(1,i);
                  array1[i]=data_x[i];
                  array2[i]=data_y[i];
                  //cout<<array1[i]<<endl;
                  //cout<<array2[i]<<endl;
                 // cout<<"  "<<endl;

             }
              

    //遍历完全以后存入各个容器
    vector<float>polarx;
    vector<float>polary;
    vector<float>storagex;
    vector<float>storagey;
    vector<double>d_radius;
    vector<double>d_angle;
    vector<int>realpoint_num;
    int total = 0;
    real_total = 0;


    for(int i=0; i<count; i++)
    {   

        int counter = 0;
        float sumx = 0;
        float sumy = 0; 
        float meansumx = 0;
        float meansumy = 0;
        double pillar_distance = 0;
        //三点准备定圆心
        //double pillar_distance_point1=0;  
        //double pillar_distance_point2=0;
        //double pillar_distance_point3=0;
        for(int j = i ; j<count; j++)
        {
             if((array1[i]-array1[j])*(array1[i]-array1[j]) +(array2[i]-array2[j])*(array2[i]-array2[j]) < 1)
          {
              sumx =array1[j]+sumx;
              sumy =array2[j]+sumy; 
              counter++;
            
          }
          if((array1[i]-array1[j])*(array1[i]-array1[j]) +(array2[i]-array2[j])*(array2[i]-array2[j]) > 1)
          {
            break;
          }
        }
        meansumx = sumx/counter;
        meansumy = sumy/counter;
        storagex.push_back(meansumx);
        storagey.push_back(meansumy);

        //把第一个数放入容器中，没有必要比较
        if(i==0)
        {
            polarx.push_back(meansumx);
            polary.push_back(meansumy);
            //d_radius.push_back(radius[0]);
            realpoint_num.push_back(counter);
            total++;
              //计算激光到各个柱子的距离
            //   for(int m = i; m<i+counter; m++)
            //   {
            //      pillar_distance += radius[m]; 
            //   }
            //   pillar_distance = pillar_distance/counter;
            //   d_radius.push_back(pillar_distance);
            pillar_distance = radius[i+counter/2]+0.029; 
            d_radius.push_back(pillar_distance);
            d_angle.push_back(reflector_angle[i+counter/2]);

        }
        //剩下的依次和先前存入容器的数据比较
        else
        {
             if((storagex[i]-polarx[total-1])*(storagex[i]-polarx[total-1]) +(storagey[i]-polary[total-1])*(storagey[i]-polary[total-1])>1)
          {
             polarx.push_back(meansumx);
             polary.push_back(meansumy);
             //d_radius.push_back(radius[i]);
             realpoint_num.push_back(counter);
             total++;
              //计算激光到各个柱子中心点的距离（取一个接近中间的点加上半径）
              
                 pillar_distance = radius[i+counter/2]+0.029; 
              
            //   计算激光到各个柱子中心点的距离（均值计算圆柱中心）
            //   for(int m = i; m<i+counter; m++)
            //   {
            //      pillar_distance += radius[m]; 
            //   }
            //   pillar_distance = pillar_distance/counter;
              d_radius.push_back(pillar_distance);
              d_angle.push_back(reflector_angle[i+counter/2]);
          }
        }

        //计算激光到各个柱子中心点的距离
        
        //cout<<count<<endl;
        //cout<<meansumx<<"聚类后的结果"<<meansumy<<endl;
       // cout<<counter<<endl;
        real_total=total;

    }
    // for(int d= 0; d < total; d++)
    // {
    // cout<<"聚类后的坐标"<<polarx[d]<<"     "<<polary[d]<<endl;
    // }
    //排除首尾相同的元素
    // if(total>1)
    // {
    // if(pow(polarx[0]-polarx[total-1], 2) + pow(rea_polary[0]-polary[total-1], 2) < 0.2)
    // {
    //   polarx.pop_back();
    //   polary.pop_back();
    //   d_radius.pop_back();
    //   d_angle.pop_back();
    //   total = total -1;
    //   real_total = total;
    // }
    // }
    //cout<<"各个柱子的坐标为"<<endl;
    //排除其他高反射度的物体

    for(int i =0; i<total ;i++)
    {
        double theta = 0; 
        int point_num = 0;
        //cout<<polarx[i]<<"      "<<polary[i]<<endl;
        //cout<<d_radius[i]<<endl;
        //cout<<realpoint_num[i]<<endl;
        theta = 2*asin(0.029/d_radius[i]);
        point_num = floor(theta/deg_resolution);
        //利用lanmark的半径来排除可能的其他物体
          //cout<<"测计算的点数和测量的点数"<<point_num<<"      "<<realpoint_num[i]<<endl;
        //筛选的误差点数差
        if(abs(point_num-realpoint_num[i]) > 5)
        {
            //cout<<"计算的点数和测量的点数"<<point_num<<"      "<<realpoint_num[i]<<endl;
            total = total-1;
            for(int j = i; j<total; j++)
            {
                polarx[j] = polarx[j+1];
                polary[j] = polary[j+1];
                d_radius[j] = d_radius[j+1];
                d_angle[j] = d_angle[j+1];
            }
            i = i -1;
            //real_total = real_total-1;
        }
    }

    //这段程序有点问题  核心已转储
    // for(int i = 0; i<real_total; i++)
    // {
    //     rea_distance[i] = d_radius[i];
    //     rea_angle[i] = d_angle[i];
    // }
    real_total = total;
    rea_distance = d_radius;
    rea_angle = d_angle;
    rea_polarx = polarx;
    rea_polary = polary;

    sensor_msgs::PointCloud real_points;
    real_points.header.frame_id = "scan";
    real_points.header.stamp = ros::Time::now();
    geometry_msgs::Point real_point;
    for(int i=0; i<real_total; i++)
    {
        geometry_msgs::Point32 real_point;
        real_point.x = polarx[i];
        real_point.y = polary[i];
        real_point.z = 0.0;
        real_points.points.push_back(real_point);
    }
    real_points_pub.publish(real_points);
    
    
    //发送第一次阈值后的点
    scan_msg.ranges = pub_ranges;
    scan_msg.intensities = intensity;
    point_filter_pub.publish(scan_msg);

  
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
    //std::cout<<"dataset_size "<<dataset.size()<<std::endl;
    //std::cout<<"count "<<count<<std::endl;

        float center_x = 0;
        float center_y = 0;
        for(int i = 0; i<real_total; i++)
        {
            center_x += polarx[i];
            center_y += polary[i];
            // polarx[i] = -polarx[i];
            // polary[i] = -polary[i]; 

        }
        
        p.x = center_x/total;
        p.y = center_y/total;
        p.z = 0;
        points.points.push_back(p);
        point_marker_pub.publish(points);



}
// double last_x = 0;
// double last_y = 0;
// double last_theta = 0;
static double
normalize(double z)   //  将弧度制的z，转换到 -PI～PI 区间
{
  return atan2(sin(z),cos(z));
}
static double
angle_diff(double a, double b)  //  求两个弧度a b 的差值
{
  double d1, d2;
  a = normalize(a);
  b = normalize(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1); //  fabs(x)  求浮点数x的绝对值
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}

void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // double last_x = 0;
    // double last_y = 0;
    // double last_theta = 0;

    Matrix<double, 3, 3> G; // Jacobian (motion update / state variable)
    Matrix<double, 3, 3> V; // Jacobian (motion noise / state variable)
    Matrix<double, 3, 3> M; // motion noise
    Matrix<double, 3, 3> COV;
    Matrix<double, 3, 1> MU;
    geometry_msgs::Point marker_proposal, p_truth;

    G.setIdentity();
    V.setZero();
    M.setZero();

    now_motion.x = msg->pose.pose.position.x;
    now_motion.y = msg->pose.pose.position.y;
    now_motion.theta = tf::getYaw(msg->pose.pose.orientation);
    //now_motion.theta = acos(msg->pose.pose.orientation.w)*2;
    // judge = msg->pose.pose.orientation.z;
    // if(judge > 0 )
    // {
    //    now_motion.theta = now_motion.theta;
    // }
    // else
    // {
    //     now_motion.theta = -now_motion.theta;
    // }
    if(now_motion.theta > PI)
    {
        now_motion.theta = now_motion.theta - 2*PI;
    }
    else if(now_motion.theta < -PI)
    {
        now_motion.theta = now_motion.theta + 2*PI;
    }
    //cout<<"last_motion的读数是"<<last_motion.x<<"    "<<last_motion.y<<"    "<<last_motion.theta<<endl;
    //cout<<"now_motion的读数是"<<now_motion.x<<"    "<<now_motion.y<<"    "<<now_motion.theta<<endl;
    
    odom_parameter.trans = sqrt (pow(now_motion.y-old_motion.y,2) + pow(now_motion.x-old_motion.x,2)) ;
    if(odom_parameter.trans<0.001)
    {
      odom_parameter.rot1 = 0;
    }
    else
    {
      odom_parameter.rot1 = atan2(now_motion.y-old_motion.y, now_motion.x-old_motion.x)- old_motion.theta;
    }
    odom_parameter.rot2 = now_motion.theta - old_motion.theta - odom_parameter.rot1;

    cout<<"rot1为"<<odom_parameter.rot1<<"  "<<"rot2为"<<odom_parameter.rot2<<endl;
    //cout<<"机器人偏转的角度为"<<real_motion.theta+odom_parameter.rot1<<endl;
    old_motion.x = now_motion.x;
    old_motion.y = now_motion.y;
    old_motion.theta = now_motion.theta;
    
    //cout<<"odom的参数是"<<odom_parameter.rot1<<"  "<<odom_parameter.trans<<"    "<<odom_parameter.rot2<<endl;
    //cout<<"real_motion的读数是"<<real_motion.x<<"    "<<real_motion.y<<"    "<<real_motion.theta<<endl;
    
    bool update = fabs(odom_parameter.trans)> 0.001 || fabs(odom_parameter.rot2) >0.000000000001;
    //prediction
    if(update)
    {
    prediction_motion.x = real_motion.x + odom_parameter.trans*cos(real_motion.theta+odom_parameter.rot1);
    prediction_motion.y = real_motion.y + odom_parameter.trans*sin(real_motion.theta+odom_parameter.rot1);
    prediction_motion.theta = real_motion.theta + odom_parameter.rot1 + odom_parameter.rot2;

    // prediction_motion.x = now_motion.x;
    // prediction_motion.y = now_motion.y;
    // prediction_motion.theta = now_motion.theta;
    nav_msgs::Odometry pre_odom;
    pre_odom.header.stamp = ros::Time::now();
    pre_odom.header.frame_id = "map";
    pre_odom.pose.pose.position.x = prediction_motion.x;
    pre_odom.pose.pose.position.y = prediction_motion.y;
    pre_odom.pose.pose.position.z = 0.0;
    geometry_msgs::Quaternion odom_quat_1 = tf::createQuaternionMsgFromYaw(prediction_motion.theta);
    pre_odom.pose.pose.orientation = odom_quat_1;
    pre_odom_pub.publish(pre_odom);
    
    //cout<<"prediction_motion的读数是"<<prediction_motion.x<<"    "<<prediction_motion.y<<"    "<<prediction_motion.theta<<endl;


    MU(0, 0) = prediction_motion.x;
    MU(1, 0) = prediction_motion.y;
    MU(2, 0) = prediction_motion.theta;
    
    //noise
    M(0, 0) = 0.01*pow(odom_parameter.rot1,2)+0.02*pow(odom_parameter.trans,2);
    M(1, 1) = 0.01*pow(odom_parameter.rot1,2)+0.03*pow(odom_parameter.trans,2)+0.04*pow(odom_parameter.rot2,2);
    M(2, 2) = 0.01*pow(odom_parameter.rot2,2)+0.01*pow(odom_parameter.trans,2);


    G(0, 2) = -odom_parameter.trans*sin(real_motion.theta+odom_parameter.rot1);
    G(1, 2) =  odom_parameter.trans*cos(real_motion.theta+odom_parameter.rot1);

    V(0, 0) = -odom_parameter.trans*sin(real_motion.theta+odom_parameter.rot1);
    V(0, 1) = cos(real_motion.theta+odom_parameter.rot1);
    V(1, 0) = odom_parameter.trans*cos(real_motion.theta+odom_parameter.rot1);
    V(1, 1) = sin(real_motion.theta+odom_parameter.rot1);
    V(2, 0) = 1;
    V(2, 2) = 1;

    COV = G*primitive_cov*G.transpose() + V*M*V.transpose();

    // cout<<"错误判断中点"<<endl;
    // correction step  

    Matrix<double, 2, 1> ZHAT; // expected sensor measurement
    Matrix<double, 2, 1> Z; // sensor measurement
    Matrix<double, 2, 3> H; // Jacobian
    Matrix<double, 2, 2> S;
    Matrix<double, 2, 2> Q; // measurement noise
    Matrix<double, 3, 2> K; // Kalman gain
    Matrix<double, 3, 3> I; // Identity
    Matrix<double, 2, 1> delta_Z;
   
    Q.setZero();
    H.setZero();
    I.setIdentity();

    double q;
    //double dist;
    if(pow(rea_polarx[0]-rea_polarx[real_total-1], 2) + pow(rea_polary[0]-rea_polary[real_total-1], 2) < 0.01)
    {
      rea_polarx.pop_back();
      rea_polary.pop_back();
      rea_distance.pop_back();
      rea_angle.pop_back();
      real_total = real_total -1;
    }
    
    // int real_total_rec = real_total;
    // vector<double> rea_polarx_rec;
    // vector<double> rea_polary_rec;
    // vector<double> rea_angle_rec;
    // vector<double> rea_distance_rec;
    // double dist_min;

    // // filter the outlier
    // for (int i = 0; i < real_total; i++)
    //       {
    //         marker_proposal.x = rea_polarx[i] * cos(MU(2)) + rea_polary[i] * sin(MU(2)) + MU(0);
    //         marker_proposal.y = rea_polarx[i] * sin(MU(2)) - rea_polary[i] * cos(MU(2)) + MU(1);
    //         marker_proposal.z = 0;
    //         int observed_id = 0;
    //         dist_min = 250000; //squared, big enough
    //         for (auto marker : marker_truth)
    //         {                                                                                            //loop for data association
    //           dist = pow(marker.x - marker_proposal.x, 2) + pow(marker.y - marker_proposal.y, 2); //NOTE:squared
    //           if (dist < dist_min)
    //           {
    //             dist_min = dist;
    //             observed_id = marker.z;
    //           }
    //         }
    //         if(dist_min < 3)
    //         {
    //           rea_polarx_rec.push_back(rea_polarx[i]);
    //           rea_polary_rec.push_back(rea_polary[i]);
    //           rea_angle_rec.push_back(rea_angle[i]);
    //           rea_distance_rec.push_back(rea_distance[i]);
    //         }
    //         else
    //         {
    //           real_total_rec = real_total_rec - 1;
    //         }
    //       }
    //       cout<<"dist_min的值是"<<dist_min<<endl;
    //       cout<<"real_total_rec的值为"<<real_total_rec<<endl;
    //       cout<<"real_total的值为"<<real_total<<endl;

    //add tri-angle localization 
    //cout<<"real_total的值为"<<real_total<<endl;

    //int real_total_rec = real_total;
    if(real_total > 2)
    {   
        cout<<"三角定位此时反光柱的数量为"<<real_total<<endl;
        cout<<"polarx的值为"<<rea_polarx.size()<<endl;
        //int column = 2;
        // Eigen::MatrixXd A(real_total- 1, 2);
        // Eigen::MatrixXd b(real_total- 1, 1);
        Eigen::Matrix<double, 2, 2> A_transition;
        Eigen::Matrix<double, 2, 1> position_final;
        double dist_min_detect = 900;
        //int i = 0;
      
        std::vector<double> arr_x;
        std::vector<double> arr_y;
        std::vector<double> arr_d;
        std::vector<double> arr_a;
        double theta = 0;


        for (int i = 0; i < real_total; i++)
          {
            marker_proposal.x = rea_polarx[i] * cos(MU(2)) - rea_polary[i] * sin(MU(2)) + MU(0);
            marker_proposal.y = rea_polarx[i] * sin(MU(2)) + rea_polary[i] * cos(MU(2)) + MU(1);
            cout<<"反光柱的投影坐标为"<<marker_proposal.x<<"  "<<marker_proposal.y<<endl;
            marker_proposal.z = 0;
            int observed_id = 0;
            double dist_min = 2500; //squared, big enough
            for (auto marker : marker_truth)
            {                                                                                            //loop for data association
              double dist = pow(marker.x - marker_proposal.x, 2) + pow(marker.y - marker_proposal.y, 2); //NOTE:squared
              if (dist < dist_min)
              {
                dist_min = dist;
                observed_id = marker.z;
              }
            }
            
            if(dist_min < 1)
            {
            for (auto marker : marker_truth)
            { //extract marker ground truth
              if (marker.z == observed_id)
              {
                arr_x.push_back(marker.x);
                arr_y.push_back(marker.y);
              }
            }
                arr_d.push_back(rea_distance[i]);
                arr_a.push_back(rea_angle[i]);
            }
            else
            {
              real_total = real_total -1;
              for(int j = i; j < real_total; j++)
              {
                rea_polarx[j] = rea_polarx[j+1];
                rea_polary[j] = rea_polary[j+1];
                rea_angle[j] = rea_angle[j+1];
                rea_distance[j] = rea_distance[j+1];
              }
              i = i -1;
              //continue;
            } 
          }

          //     for (int i = 0; i < real_total_rec; i++)
          // {
          //   marker_proposal.x = rea_polarx[i] * cos(MU(2)) - rea_polary[i] * sin(MU(2)) + MU(0);
          //   marker_proposal.y = rea_polarx[i] * sin(MU(2)) + rea_polary[i] * cos(MU(2)) + MU(1);
          //   cout<<"反光柱的投影坐标为"<<marker_proposal.x<<"  "<<marker_proposal.y<<endl;
          // } 

          if(real_total < 3)
          {
            goto no_triangle;
          }
          //cout<<"1"<<endl;
          cout<<"更新后的三角定位此时反光柱的数量为"<<real_total<<endl;
          double max_x = arr_x[real_total-1];
          double max_y = arr_y[real_total-1];
          double max_d = arr_d[real_total-1];
          vector<double> theta_test;
          //cout<<"更新后的三角定位此时反光柱的数量为"<<real_total<<endl;
          //cout<<"4"<<endl;
          Eigen::MatrixXd A(real_total- 1, 2);
          Eigen::MatrixXd b(real_total- 1, 1);
          //cout<<"3"<<endl;
          for(int m = 0; m < real_total- 1; m++)
          {
             A(m, 0) = 2*(arr_x[m] - max_x);
             A(m, 1) = 2*(arr_y[m] - max_y);
             b(m, 0) = pow(arr_x[m], 2) - pow(max_x, 2) + pow(arr_y[m], 2) - pow(max_y, 2) + pow(max_d, 2)-pow(arr_d[m], 2);
          }
          A_transition = A.transpose() * A;
          position_final = A_transition.inverse() * A.transpose() * b;
          for(int n = 0; n < real_total; n++)
          {
             double delta_theta = atan2(arr_y[n] - position_final[1], arr_x[n] - position_final[0]) - arr_a[n];
             if(delta_theta > PI)
             {
               delta_theta = delta_theta - 2*PI;
             }
             else if(delta_theta < -PI)
             {
               delta_theta = delta_theta + 2*PI;
             }
             theta_test.push_back(delta_theta);
             //cout<<"世界中的偏转为"<<atan2(arr_y[n] - position_final[1], arr_x[n] - position_final[0])<<endl;
             //cout<<"自身的偏转为"<<arr_a[n]<<endl;
             //cout<<"delta_theta的值是"<<delta_theta<<endl;
             // theta = theta + delta_theta;
          }
          //纠正角度
          for(int i = 1; i < real_total; i++)
          {
            double theta_exam = theta_test[i] - theta_test[0];
            if(fabs(theta_exam) > 5)
            { 
              double theta_sum;
              for(int m = 0; m < real_total; m++)
              {
                theta_sum += theta_test[m];
              }
              if(theta_sum > 0)
              {
                for(int m = 0; m < real_total; m++)
               {
                if(theta_test[m] < 0)
                {
                theta_test[m] = theta_test[m] + 2*PI;
                }
               }
              }
              else
              {
                for(int m = 0; m < real_total; m++)
               {
                if(theta_test[m] > 0)
                {
                theta_test[m] = theta_test[m] - 2*PI;
                }
               }
              }
              break;
            }
      
          }

          for(int m = 0; m < real_total; m++)
               {
                theta += theta_test[m];
               }

          cout<<"2"<<endl;
          theta = theta/real_total;
          real_motion.x = position_final[0];
          real_motion.y = position_final[1];
          real_motion.theta = theta;
          cout<<"机器人此时的位置为"<<position_final[0]<<"   "<<position_final[1];

    }
    else
    {
      no_triangle:
      Q(0, 0) = 0.5;
      Q(1, 1) = 0.14;
    for (int i = 0; i < real_total; i++)
          {
            marker_proposal.x = rea_polarx[i] * cos(MU(2)) - rea_polary[i] * sin(MU(2)) + MU(0);
            marker_proposal.y = rea_polarx[i] * sin(MU(2)) + rea_polary[i] * cos(MU(2)) + MU(1);
            marker_proposal.z = 0;
            int observed_id = 0;
            double dist_min = 2500; //squared, big enough
            cout<<"反光柱的投影坐标为"<<marker_proposal.x<<"  "<<marker_proposal.y<<endl;
            for (auto marker : marker_truth)
            {                                                                                            //loop for data association
              double dist = pow(marker.x - marker_proposal.x, 2) + pow(marker.y - marker_proposal.y, 2); //NOTE:squared
              if (dist < dist_min)
              {
                dist_min = dist;
                observed_id = marker.z;
              }
            }
            if(dist_min < 1)
            {
            for (auto marker : marker_truth)
            { //extract marker ground truth
              if (marker.z == observed_id)
              {
                p_truth = marker;
              }
            }
            //cout<<"p_truth的值为"<<p_truth.x<<"  "<<p_truth.y<<endl;
            // Q(0, 0) = 0.08*pow(rea_distance[i],2);
            //Q(1, 1) = 0.14*pow(rea_angle[i],2);

            Z(0) = rea_distance[i];
            Z(1) = rea_angle[i];
            q = pow(p_truth.x-MU(0),2)+pow(p_truth.y-MU(1),2);
            ZHAT(0) = sqrt(q);
            ZHAT(1) = atan2(p_truth.y-MU(1), p_truth.x-MU(0))-MU(2);
            if(ZHAT(1) > PI)
            {
                ZHAT(1) = ZHAT(1) - 2*PI;
            }
            else if (ZHAT(1) < -PI)
            {
                ZHAT(1) = ZHAT(1) + 2*PI;
            }
            H(0, 0) = -(p_truth.x-MU(0))/sqrt(q);
            H(0, 1) = -(p_truth.y-MU(1))/sqrt(q);
            H(1, 0) = (p_truth.y-MU(1))/q;
            H(1, 1) = -(p_truth.x-MU(0))/q;
            H(1, 2) = -1;

           delta_Z = Z - ZHAT;
           if(delta_Z(1) > PI)
           {
             delta_Z(1) = 2*PI-delta_Z(1) ;
           }
           else if(delta_Z(1) < -PI)
           {
             delta_Z(1) = -delta_Z(1) - 2*PI;
           }
           S = H*COV*H.transpose() + Q;
           K = COV*H.transpose()*S.inverse();
           //S = H*COV*H.transpose() + Q;
           MU = MU + K*delta_Z;
           COV = (I - K*H)*COV; 
              if(MU(2)>PI)
           {
               MU(2) = MU(2)-2*PI;
           }
           else if(MU(2)<-PI)
           {
               MU(2) = MU(2)+2*PI;
           }
        }
        else
        {
          continue;
        }
        
    //set the error
    // if(2*sqrt(COV(0, 0)) < 100 && 2*sqrt(COV(1, 1)) < 100 && 2*sqrt(COV(2, 2)) < 100)
    // {
    //   real_motion.x = MU(0);
    //   real_motion.y = MU(1);
    //   real_motion.theta = MU(2);
    //   primitive_cov = COV;
    // }
    // else
    // {
    //   real_motion = last_motion;
    //   // real_motion.x = MU(0);
    //   // real_motion.y = MU(1);
    //   // real_motion.theta = MU(2);
    //   // primitive_cov = COV;
    // }
    // }
        }
    
    real_motion.x = MU(0);
    real_motion.y = MU(1);
    real_motion.theta = MU(2);
    primitive_cov = COV;
    }
    }
    //last_motion = real_motion;
    
    //real_last_motion = real_now_motion;
    
    //cout<<"里程计正常调用"<<endl;
    //cout<<relative_x<<endl;
    //real_motion = prediction_motion;
    //primitive_cov = COV;
    //cout<<"COV是"<<COV(0, 0)<<"   "<<COV(1, 1)<<"和"<<COV(2, 2)<<endl;
    //cout<<"目前机器人的定位是"<<real_motion.x<<"   "<<real_motion.y<<"和"<<real_motion.theta<<endl;
    nav_msgs::Odometry cal_odom;
    cal_odom.header.stamp = ros::Time::now();
    cal_odom.header.frame_id = "map";
    cal_odom.pose.pose.position.x = real_motion.x;
    cal_odom.pose.pose.position.y = real_motion.y;
    cal_odom.pose.pose.position.z = 0.0;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(real_motion.theta);
    cal_odom.pose.pose.orientation = odom_quat;
    cal_odom_pub.publish(cal_odom);
  } 




int main(int argc, char **argv)
{
    
    // double last_x = 0;
    // double last_y = 0;
    // double last_theta = 0;
    primitive_cov.setZero();
    real_motion.x = 0;
    real_motion.y = 0;
    real_motion.theta = 0;

    old_motion.x = 0;
    old_motion.y = 0;
    old_motion.theta = 0;

    geometry_msgs::Point p_truth;

    std::ifstream ifs;
  ifs.open("/home/jjj/K_means/catkin_ws/landmarks.txt", std::ios::in);
  if (!ifs.is_open())
  {
    std::cout << "文件打开失败"
              << "\n";
  }
  else
  {
    std::string line_read;
    int read_count = 0;
    // Run over each single line:
    while (std::getline(ifs, line_read))
    {
      if (read_count == 0)
      {
        std::istringstream iss_map(line_read);
        iss_map >> marker_std_x;
        iss_map >> marker_std_y;
        read_count++;
      }
      else
      {
        std::istringstream iss_map(line_read);

        // Declare landmark values and ID:
        // float landmark_x_f, landmark_y_f;
        // int id_i;

        // Read data from current line to values::
        iss_map >> p_truth.x;
        iss_map >> p_truth.y;
        iss_map >> p_truth.z;
        read_count++;
        marker_truth.push_back(p_truth);
      }
    }
    std::cerr << "marker_std_x " << marker_std_x << " marker_std_y " << marker_std_y << "\n";
    for (auto truth : marker_truth)
    {
      std::cerr << "truth x: " << truth.x << " y " << truth.y << "\n";
    }
  }



    ros::init(argc, argv, "laserscan_receive");
    ros::init(argc, argv, "odom_listener");
    ros::NodeHandle nh;
    ros::Subscriber laserscan_sub = nh.subscribe("/r2000_node/scan", 10, Lasercallback);
    //需要找到确定的话题
    ros::Subscriber sub = nh.subscribe("ir100_velocity_controller/odom", 1000, chatterCallback);
    center_point = nh.advertise<geometry_msgs::Twist>("/center_points", 10); //发送计算出来的中心点位置
    point_filter_pub = nh.advertise<sensor_msgs::LaserScan>("/scan1", 10);//这两句什么意思
    point_marker_pub = nh.advertise<visualization_msgs::Marker>("visulization_marker",10);
    cal_odom_pub = nh.advertise<nav_msgs::Odometry>("/cal_odom",1000);
    pre_odom_pub = nh.advertise<nav_msgs::Odometry>("/pre_odom",1000);
    real_points_pub = nh.advertise<sensor_msgs::PointCloud>("real_pillar",10);
    ros::Rate loop(200);
    
    
    while(ros::ok())
    {        
        
       ros::spinOnce();
       loop.sleep();
    }

    return 0;
}

