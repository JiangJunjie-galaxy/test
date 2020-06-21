
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
ros::Time last_time ;
vector<double>rea_distance;
vector<double>rea_angle;
motion last_motion;
motion now_motion;
motion prediction_motion;
motion real_motion;
odom_parameter odom_parameter; 





ros::Publisher center_point;
ros::Publisher point_filter_pub;
ros::Publisher point_marker_pub;
ros::Publisher cal_odom_pub;
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

        if(intensity[i] >1320)
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
                //   cout<<array1[i]<<endl;
                //   cout<<array2[i]<<endl;
                //   cout<<"  "<<endl;

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
        for(int j=0 ; j<count; j++)
        {
             if((array1[i]-array1[j])*(array1[i]-array1[j]) +(array2[i]-array2[j])*(array2[i]-array2[j])<1)
          {
              sumx =array1[j]+sumx;
              sumy =array2[j]+sumy; 
              counter++;
            
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
        
        // cout<<count<<endl;
        // cout<<meansumx<<"聚类后的结果"<<meansumy<<endl;
        // cout<<counter<<endl;
        real_total=total;

    }
    if(pow(polarx[0]-polarx[total-1],2)+pow(polary[0]-polary[total-1],2)<0.2)
    {
        polarx.pop_back();
        polary.pop_back();
    }
    cout<<"各个柱子的坐标为"<<endl;
    //排除其他高反射度的物体
    for(int i =0; i<total ;i++)
    {
        double theta = 0; 
        int point_num = 0;
        cout<<polarx[i]<<"      "<<polary[i]<<endl;
        cout<<d_radius[i]<<endl;
        cout<<realpoint_num[i]<<endl;
        theta = 2*asin(0.029/d_radius[i]);
        point_num = floor(theta/deg_resolution);
        //利用lanmark的半径来排除可能的其他物体
        cout<<"测计算的点数和测量的点数"<<point_num<<"      "<<realpoint_num[i]<<endl;
        //筛选的误差点数差
        if(abs(point_num-realpoint_num[i])>5)
        {
            //cout<<"计算的点数和测量的点数"<<point_num<<"      "<<realpoint_num[i]<<endl;
            for(int j = i; j<total; j++)
            {
                polarx[j] = polarx[j+1];
                polary[j] = polary[j+1];
                d_radius[j] = d_radius[j+1];
                d_angle[j] = d_angle[j+1];
            }
            real_total=total-1;
        }
    }
    //这段程序有点问题  核心已转储
    // for(int i = 0; i<real_total; i++)
    // {
    //     rea_distance[i] = d_radius[i];
    //     rea_angle[i] = d_angle[i];
    // }
    rea_distance = d_radius;
    rea_angle = d_angle;
    
    
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
void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    now_motion.x = msg->pose.pose.position.x;
    now_motion.y = msg->pose.pose.position.y;
    now_motion.theta = acos(msg->pose.pose.orientation.w)*2;
    double judge = msg->pose.pose.orientation.z;
    if(judge > 0 )
    {
       now_motion.theta = now_motion.theta;
    }
    else
    {
        now_motion.theta = -now_motion.theta;
    }
    if(now_motion.theta > PI)
    {
        now_motion.theta = now_motion.theta - 2*PI;
    }
    else if(now_motion.theta < -PI)
    {
        now_motion.theta = now_motion.theta + 2*PI;
    }
    //cout<<"now_motion的读数是"<<now_motion.x<<"    "<<now_motion.y<<"    "<<now_motion.theta<<endl;
    
    double v = msg->twist.twist.linear.x;
    double w = msg->twist.twist.angular.z;
    double dt = (ros::Time::now() -last_time).toSec();
    

    Matrix<double, 3, 3> G; // Jacobian (motion update / state variable)
    Matrix<double, 3, 2> V; // Jacobian (motion noise / state variable)
    Matrix<double, 2, 2> M; // motion noise /
    Matrix<double, 3, 3> COV;
    Matrix<double, 3, 1> MU;

    G.setIdentity();
    V.setZero();
    M.setZero();

    

    
    const double EPS = 0.001;
    double theta = m_mu(2, 0);

    M(0, 0) = pow(0.1*fabs(v) + 0.1*fabs(w), 2);
    M(1, 1) = pow(0.01*fabs(v) + 0.01*fabs(w), 2);

    // check if angular velocity is close to zero
    if (fabs(w) > EPS) 
    {
        // Jacobian
        G(0, 2) = -(v/w)*cos(theta) + (v/w)*cos(theta + w*dt);
        G(1, 2) = -(v/w)*sin(theta) + (v/w)*sin(theta + w*dt);
        cout<<"v的值是"<<v<<endl;
        cout<<"w的值是"<<w<<endl;

        V(0, 0) = (-sin(theta) + sin(theta + w*dt))/w;
        V(1, 0) = ( cos(theta) - cos(theta + w*dt))/w;
        V(0, 1) =  v*(sin(theta) - sin(theta + w*dt))/(w*w) + v*cos(theta + w*dt)*dt/w;
        V(1, 1) = -v*(cos(theta) - cos(theta + w*dt))/(w*w) + v*sin(theta + w*dt)*dt/w;
        V(2, 0) = 0;
        V(2, 1) = dt;

        // Prediction
        MU(0) = m_mu(0) - (v/w)*sin(theta) + (v/w)*sin(theta + w*dt);
        MU(1) = m_mu(1) + (v/w)*cos(theta) - (v/w)*cos(theta + w*dt);
        MU(2) = m_mu(2) + w*dt;

        COV = G*m_cov*G.transpose() + V*M*V.transpose();
    } 
    else
    {
        // Handle case when w ~ 0
        // Use L'Hopital rule with lim w -> 0
        G(0, 2) = -v*sin(theta)*dt;
        G(1, 2) =  v*cos(theta)*dt;
        cout<<"v的值是"<<v<<endl;
        cout<<"w的值是"<<w<<endl;

        V(0, 0) = cos(theta)*dt;
        V(1, 0) = sin(theta)*dt;
        V(0, 1) = -v*sin(theta)*dt*dt*0.5;
        V(1, 1) =  v*cos(theta)*dt*dt*0.5;
        V(2, 0) = 0;
        V(2, 1) = dt;

        MU(0) = m_mu(0) + v*cos(theta)*dt;
        MU(1) = m_mu(1) + v*sin(theta)*dt;
        MU(2) = m_mu(2);

        COV = G*m_cov*G.transpose() + V*M*V.transpose();
    }

    //cout<<"MU(0):   "<<MU(0)<<endl;
    //cout<<"MU(1):   "<<MU(1)<<endl;
    //cout<<"MU(2):   "<<MU(2)<<endl;
    //cout<<"now_motion的读数是"<<now_motion.x<<"    "<<now_motion.y<<"    "<<now_motion.theta<<endl;


    // cout<<"错误判断中点"<<endl;
    // correction step
    Matrix<double, 2, 1> ZHAT; // expected sensor measurement
    Matrix<double, 2, 1> Z; // sensor measurement
    Matrix<double, 2, 3> H; // Jacobian
    Matrix<double, 2, 2> S;
    Matrix<double, 2, 2> Q; // measurement noise
    Matrix<double, 3, 2> K; // Kalman gain
    Matrix<double, 3, 3> I; // Identity
   
    Q.setZero();
    H.setZero();
    I.setIdentity();

    vector<double>map_pillar_x;
    vector<double>map_pillar_y;
    double q;
    double likelihood;

    
    //四根柱子在map中的坐标
    map_pillar_x.push_back(1.98054);
    map_pillar_x.push_back(2.02225);
    map_pillar_x.push_back(-0.33849);
    map_pillar_x.push_back(-1.12622);

    map_pillar_y.push_back(-1.61094);
    map_pillar_y.push_back(-0.0352299);
    map_pillar_y.push_back(1.5742);
    map_pillar_y.push_back(0.0245592);

    // map_pillar_x.push_back(-1.1147);
    // map_pillar_x.push_back(1.29467);
    // map_pillar_x.push_back(-0.444274);
    // map_pillar_x.push_back(1.18373);

    // map_pillar_y.push_back(0.2218904);
    // map_pillar_y.push_back(-2.25384);
    // map_pillar_y.push_back(1.61582);
    // map_pillar_y.push_back(0.0801615);
    


    //measurement noise
    //Q(0, 0) = 0.01;
    //Q(1, 1) = 0.01;
    // cout<<"错误判断中点"<<endl;

    for(int i = 0; i<real_total; i++)
    {   

        
        //measurement noise
        Q(0, 0) = pow(rea_distance[i]*0.1,2);
        Q(1, 1) = pow(rea_angle[i]*0.1,2);

        Z(0, 0) = rea_distance[i];
        Z(1, 0) = rea_angle[i];
        cout<<"rea_distance的值是"<<rea_distance[i]<<endl;
        cout<<"rea_angle的值是"<<rea_angle[i]<<endl;
        vector<double>likewise;
        //地图中全部柱子的数量
        for(int j = 0; j<4; j++)
        {
           q = pow(map_pillar_x[j]-MU(0),2)+pow(map_pillar_y[j]-MU(1),2);
           ZHAT(0, 0) = sqrt(q);
           //这个在机器人坐标系中算的方向角
           ZHAT(1, 0) = atan2(map_pillar_y[j]-MU(1), map_pillar_x[j]-MU(0))-MU(2);
           //cout<<"MU(0)的值是"<<MU(0)<<endl;
           // cout<<"MU(1)的值是"<<MU(1)<<endl;
           //cout<<"MU(2)的值是"<<MU(2)<<endl;

           if(ZHAT(1) > PI)
            {
                ZHAT(1) = ZHAT(1) - 2*PI;
            }
            else if (ZHAT(1) < -PI)
            {
                ZHAT(1) = ZHAT(1) + 2*PI;
            }
           H(0, 0) = -(map_pillar_x[j]-MU(0))/sqrt(q);
           H(0, 1) = -(map_pillar_y[j]-MU(1))/sqrt(q);
           H(1, 0) = (map_pillar_y[j]-MU(1))/q;
           H(1, 1) = -(map_pillar_x[j]-MU(0))/q;
           H(1, 2) = -1;
           cout<<"机器人和柱子的测量距离是"<<ZHAT(0, 0)<<endl;
           cout<<"机器人和柱子的角度是"<<ZHAT(1, 0)<<endl;
           S = H*COV*H.transpose() + Q;
           likelihood = pow((2*PI*S).determinant(),-0.5)*exp(-0.5*(Z-ZHAT).transpose()*S.inverse()*(Z-ZHAT));
     
           cout<<"likelihood的值是"<<likelihood<<endl;
           //cout<<(Z-ZHAT)(0, 0)<<endl;
           //cout<<(Z-ZHAT)(1, 0)<<endl;
           likewise.push_back(likelihood);

        }
        // cout<<"错误判断中点"<<endl;
        int marker;
        double mid = 0;
        for(int m = 0; m<4; m++)
        {
           if(likewise[m]>mid)
           {
              mid = likewise[m];
              marker = m;
           }

        }  
           //cout<<"marker的值"<<marker<<endl;
           q = pow(map_pillar_x[marker]-MU(0),2)+pow(map_pillar_y[marker]-MU(1),2);
           ZHAT(0, 0) = sqrt(q);
           ZHAT(1, 0) = atan2(map_pillar_y[marker]-MU(1), map_pillar_x[marker]-MU(0))-MU(2);
           if(ZHAT(1) > PI)
            {
                ZHAT(1) = ZHAT(1) - 2*PI;
            }
            else if (ZHAT(1) < -PI)
            {
                ZHAT(1) = ZHAT(1) + 2*PI;
            }
           H(0, 0) = -(map_pillar_x[marker]-MU(0))/sqrt(q);
           H(0, 1) = -(map_pillar_y[marker]-MU(1))/sqrt(q);
           H(1, 0) = (map_pillar_y[marker]-MU(1))/q;
           H(1, 1) = -(map_pillar_x[marker]-MU(0))/q;
           
           //measurement noise
           //Q(0, 0) = pow(rea_distance[i]*0.1,2);
           //Q(1, 1) = pow(rea_angle[i]*0.1,2);
          
           S = H*COV*H.transpose() + Q;
           K = COV*H.transpose()*S.inverse();
           //S = H*COV*H.transpose() + Q;
           MU = MU + K*(Z - ZHAT);
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

            cout<<"里程计正常调用"<<endl;
    //         //cout<<relative_x<<endl;
            m_mu = MU;
            m_cov = COV;

            cout<<"now_motion的读数是"<<now_motion.x<<"    "<<now_motion.y<<"    "<<now_motion.theta<<endl;
            cout<<"目前机器人的定位是"<<m_mu(0, 0)<<"   "<<m_mu(1, 0)<<"和"<<m_mu(2, 0)<<endl;
            last_time = ros::Time::now();
            nav_msgs::Odometry cal_odom;
            cal_odom.header.stamp = ros::Time::now();
            cal_odom.header.frame_id = "odom";
            cal_odom.pose.pose.position.x = MU(0);
            cal_odom.pose.pose.position.y = MU(1);
            cal_odom.pose.pose.position.z = 0.0;
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(MU(2));
            cal_odom.pose.pose.orientation = odom_quat;
            cal_odom_pub.publish(cal_odom);

            
}

int main(int argc, char **argv)
{
    
    // double last_x = 0;
    // double last_y = 0;
    // double last_theta = 0;
    m_mu.setZero();
    m_cov.setZero();
    //last_time = ros::Time::now();
    


    //ros::Time::init();
    ros::init(argc, argv, "laserscan_receive");
    ros::init(argc, argv, "odom_listener");
    ros::NodeHandle nh;
    ros::Subscriber laserscan_sub = nh.subscribe("/r2000_node/scan", 1, Lasercallback);
    //需要找到确定的话题
    ros::Subscriber sub = nh.subscribe("ir100_velocity_controller/odom", 1000, chatterCallback);
    center_point = nh.advertise<geometry_msgs::Twist>("/center_points", 10); //发送计算出来的中心点位置
    point_filter_pub = nh.advertise<sensor_msgs::LaserScan>("/scan1", 10);//这两句什么意思
    point_marker_pub = nh.advertise<visualization_msgs::Marker>("visulization_marker",10);
    cal_odom_pub = nh.advertise<nav_msgs::Odometry>("/cal_odom",1000);

    last_time = ros::Time::now();
    ros::Rate loop(200);
    
    
    while(ros::ok())
    {        
        
       ros::spinOnce();
       loop.sleep();
    }

    return 0;
}

