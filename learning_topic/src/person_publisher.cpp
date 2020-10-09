/** * 该例程将发布/person_info话题，自定义消息类型learning_topic::Person */ 
#include <ros/ros.h>
#include "learning_topic/Homework_pub.h"
#include "learning_topic/Homework_srv.h"
#include <iostream>


bool multiplyCallback(learning_topic::Homework_srv::Request  &req,
         			learning_topic::Homework_srv::Response &res)
{
	// 设置反馈数据
	res.result = req.num1*req.num2;
        //std::cout<<req.result<<std::endl;
        ROS_INFO(" result:%d ", res.result);

    return true;
}

int main(int argc, char **argv) 
{ 
// ROS节点初始化 
ros::init(argc, argv, "robot_2001998");
 // 创建节点句柄 
ros::NodeHandle n;
 // 创建一个Publisher，发布名为/person_info的topic，消息类型为learning_topic::Person，队列长度10 
ros::Publisher words_pub = n.advertise<learning_topic::Homework_pub>("/words", 10); 
// 设置循环的频率
ros::ServiceServer words_service = n.advertiseService("/multiply", multiplyCallback);


ros::Rate loop_rate(5); 
int count = 1; 

while (ros::ok()) 
{ 

// 初始化learning_topic::Person类型的消息
 learning_topic::Homework_pub homework_msg;
 if(count%10) 
 {
         homework_msg.words = "What are you doing";
 }
 else
 {
         homework_msg.words = "OK";
 }
 
 
// 发布消息
 words_pub.publish(homework_msg); 
 count++;
 ROS_INFO("Publish Person Info: words:%s", homework_msg.words.c_str()); 

// 按照循环频率延时 
ros::spinOnce();
loop_rate.sleep();
        } 
return 0; 
         } 

