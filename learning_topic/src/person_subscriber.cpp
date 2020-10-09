/** * 该例程将订阅/person_info话题，自定义消息类型learning_topic::Person */
 #include <ros/ros.h>
#include "learning_topic/Homework_pub.h"
#include "learning_topic/Homework_srv.h"

bool flag = 0;

 // 接收到订阅的消息后，会进入消息回调函数
void wordsCallback(const learning_topic::Homework_pub::ConstPtr& msg) 
{ 
if(msg->words == "OK")
{
    flag = 1;
}
else
{
    ROS_INFO("I'am studying now");
}
}


// 将接收到的消息打印出来 
//ROS_INFO("Subcribe Person Info: name:%s age:%d sex:%d", msg->name.c_str(), msg->age, msg->sex); 
int main(int argc, char **argv)
 { 
 
//bool flag = 0;
// 初始化ROS节点 
ros::init(argc, argv, "JiangJunjie");

// 创建节点句柄 
ros::NodeHandle n; 

ros::service::waitForService("/multiply");
ros::ServiceClient words_client = n.serviceClient<learning_topic::Homework_srv>("/multiply");
learning_topic::Homework_srv srv;
// if(flag)
// {
//     srv.request.num1 = 3;
//     srv.request.num2 = 5;
//     ROS_INFO("1");
//     words_client.call(srv);
//     ROS_INFO("2");
//     flag = 0;
// }

// 创建一个Subscriber，订阅名为/person_info的topic，注册回调函数personInfoCallback 
ros::Subscriber words_sub = n.subscribe("/words", 10, wordsCallback); 
ros::Rate loop(5);

// 循环等待回调函数 
//ros::spin();
 while(ros::ok())
    {        
    if(flag)
    {
        learning_topic::Homework_srv srv;
        srv.request.num1 = 3;
        srv.request.num2 = 5;
        words_client.call(srv);
        ROS_INFO(" result:%d ", srv.response.result);
        flag = 0;
    }
       ros::spinOnce();
       loop.sleep();
    }
 return 0;
 }

