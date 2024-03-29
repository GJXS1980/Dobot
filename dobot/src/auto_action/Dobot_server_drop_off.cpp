#include <ros/ros.h>
#include "ros/console.h"

#include "std_msgs/String.h"
#include "dobot/SetCmdTimeout.h"
#include "dobot/SetQueuedCmdClear.h"
#include "dobot/SetQueuedCmdStartExec.h"
#include "dobot/SetQueuedCmdForceStopExec.h"
#include "dobot/GetDeviceVersion.h"
#include "dobot/SetEndEffectorParams.h"
#include "dobot/SetPTPJointParams.h"
#include "dobot/SetPTPCoordinateParams.h"
#include "dobot/SetPTPJumpParams.h"
#include "dobot/SetPTPCommonParams.h"
#include "dobot/SetPTPCmd.h"
#include "dobot/SetHOMEParams.h"
#include "dobot/SetHOMECmd.h"
#include "dobot/SetEndEffectorSuctionCup.h"
#include "dobot/GetQueuedCmdCurrentIndex.h"
#include "dobot/DobotAction.h"

#include <actionlib/server/simple_action_server.h>
#include "object_detect/Center_msg.h"

#include "std_msgs/Int64.h"
#include "dobot/DobotActionFeedback.h"

#include <iostream>
#include <unistd.h>
#include <cstdlib>

using namespace std;

typedef actionlib::SimpleActionServer<dobot::DobotAction> Server;


/*  ********************************************************************* */
/*      －－－－－－     执行放置物体动作    －－－－－   */

// 收到action的goal后调用的回调函数
void execute_drop_off(const dobot::DobotGoalConstPtr& goal, Server* as)
{
    ros::Rate r(1);
    dobot::DobotFeedback feedback;

    ROS_INFO("Manipulator arm %d is working.", goal->dobot_pick_id);

    ros::NodeHandle n;

    ros::ServiceClient client;

// 清除指令队列
    client = n.serviceClient<dobot::SetQueuedCmdClear>("/DobotServer/SetQueuedCmdClear");
    dobot::SetQueuedCmdClear srv2;
    client.call(srv2);


// 创建PTP客户端
    client = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
    dobot::SetPTPCmd srv;

        // 初始化机械臂的位姿
        do {
            srv.request.ptpMode = 1;
            srv.request.x = 212;
            srv.request.y = 0;
            srv.request.z = -76.5;
            srv.request.r = 0;
            client.call(srv);
            if (srv.response.result == 0) {
                break;
            }     
            ros::spinOnce();

        } while (0);


/* 
        do {
            srv.request.ptpMode = 1;
            srv.request.x = 0;
            srv.request.y = 217;
            srv.request.z = 80;
            srv.request.r = 0;
            client.call(srv);
            if (srv.response.result == 0) {
                break;
            }     
            ros::spinOnce();

        } while (0);
*/

//  识别物体，机械臂的吸盘运动到物体的中心


/*  ********************************************************************* */
//  识别物体，机械臂的吸盘运动到物体的中心

/*        do {

            Listener listener;
            ros::Subscriber sub = n.subscribe("chatter", 1000, &Listener::callback, &listener);
            ros::Rate loop_rate(10);

            while(ros::ok() and listener.count <=2){
                ros::spinOnce();
                loop_rate.sleep();
            }
  //std::cout << "After spin: \n";
            sx = listener.print_datax();
            sy = listener.print_datay();

            std::cout << sx << "\n";
            std::cout << sy << "\n";
            } while (0);


        // 机械臂运动到物体中心位置
        do {
            x = (319.5 - sx)/4.522;
            y = (sy - 239.5)/4.522 + 217 - 35.37;
            srv.request.ptpMode = 1;
            srv.request.x = x;
            srv.request.y = y;
            srv.request.z = -37.5;
            srv.request.r = 0;
            client.call(srv);
            if (srv.response.result == 0) {
                break;
            }     
            ros::spinOnce();
            if (ros::ok() == false) {
                break;
            }
        } while (0);*/
/*  ********************************************************************* */



//  松开物体
        client = n.serviceClient<dobot::SetEndEffectorSuctionCup>("/DobotServer/SetEndEffectorSuctionCup");
        dobot::SetEndEffectorSuctionCup sck3;
        //drop off 
        do{    
            sck3.request.enableCtrl = 1;
            sck3.request.suck = 0;
            sck3.request.isQueued = true;
            client.call(sck3); 
            ros::spinOnce(); //spinOnce() will execute any commands in the queue.
            if (ros::ok() == false) {
                break;
            }
        }while (0);


//  获取当前的指令索引
    client = n.serviceClient<dobot::GetQueuedCmdCurrentIndex>("/DobotServer/GetQueuedCmdCurrentIndex");
    dobot::GetQueuedCmdCurrentIndex srv9;
    client.call(srv9);

//  等待所有的动作完成
while(srv9.response.queuedCmdIndex < sck3.response.queuedCmdIndex)
    if(client.call(srv9))
    {
        //cout << "wait!" << endl;
    }

// 创建PTP客户端
    client = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
    dobot::SetPTPCmd srv1;

        // 初始化机械臂的位姿
        do {
            srv1.request.ptpMode = 1;
            srv1.request.x = 200;
            srv1.request.y = -40;
            srv1.request.z = 80;
            srv1.request.r = 0;
            client.call(srv1);
            if (srv1.response.result == 0) {
                break;
            }     
            ros::spinOnce();

        } while (0);


//  当动作完成之后，反馈回动作完成的标准
    feedback.percent_complete = 4;
    as->publishFeedback(feedback);

    // 当action完成后，向客户端返回结果
    ROS_INFO("Manipulator arm %d finish working.", goal->dobot_pick_id);
    as->setSucceeded();
}



/*  ********************************************************************* */
/*                       －－－－－－     主函数    －－－－－                    */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "PlaceObjects_server");
    ros::NodeHandle n;
    ros::ServiceClient client;


/*                   －－－－－－     设置机械臂参数    －－－－－                  */

    // 设置命令超时时间
    client = n.serviceClient<dobot::SetCmdTimeout>("/DobotServer/SetCmdTimeout");
    dobot::SetCmdTimeout srv1;
    srv1.request.timeout = 3000;
    if (client.call(srv1) == false) {
        ROS_ERROR("Failed to call SetCmdTimeout. Maybe DobotServer isn't started yet!");
        return -1;
    }

    // 清除指令队列
    client = n.serviceClient<dobot::SetQueuedCmdClear>("/DobotServer/SetQueuedCmdClear");
    dobot::SetQueuedCmdClear srv2;
    client.call(srv2);

    // 开始运行指令队列
    client = n.serviceClient<dobot::SetQueuedCmdStartExec>("/DobotServer/SetQueuedCmdStartExec");
    dobot::SetQueuedCmdStartExec srv3;
    client.call(srv3);


    // 获取设备的版本信息
    client = n.serviceClient<dobot::GetDeviceVersion>("/DobotServer/GetDeviceVersion");
    dobot::GetDeviceVersion srv4;
    client.call(srv4);
    if (srv4.response.result == 0) {
        ROS_INFO("Device version:%d.%d.%d", srv4.response.majorVersion, srv4.response.minorVersion, srv4.response.revision);
    } else {
        ROS_ERROR("Failed to get device version information!");
    }


    // 设置末端执行器参数
    client = n.serviceClient<dobot::SetEndEffectorParams>("/DobotServer/SetEndEffectorParams");
    dobot::SetEndEffectorParams srv5;
    srv5.request.xBias = 70;
    srv5.request.yBias = 0;
    srv5.request.zBias = 0;
    client.call(srv5);


    // 设置PTP模式下的关节参数
    do {
        client = n.serviceClient<dobot::SetPTPJointParams>("/DobotServer/SetPTPJointParams");
        dobot::SetPTPJointParams srv;

        for (int i = 0; i < 4; i++) {
            srv.request.velocity.push_back(100);
        }
        for (int i = 0; i < 4; i++) {
            srv.request.acceleration.push_back(100);
        }
        client.call(srv);
    } while (0);

    // 设置PTP的坐标参数
    do {
        client = n.serviceClient<dobot::SetPTPCoordinateParams>("/DobotServer/SetPTPCoordinateParams");
        dobot::SetPTPCoordinateParams srv;

        srv.request.xyzVelocity = 100;
        srv.request.xyzAcceleration = 100;
        srv.request.rVelocity = 100;
        srv.request.rAcceleration = 100;
        client.call(srv);
    } while (0);

    // 设置PTP的jump模式下的参数
    do {
        client = n.serviceClient<dobot::SetPTPJumpParams>("/DobotServer/SetPTPJumpParams");
        dobot::SetPTPJumpParams srv;

        srv.request.jumpHeight = 20;
        srv.request.zLimit = 200;
        client.call(srv);
    } while (0);

    // 设置PTP常用的参数
    do {
        client = n.serviceClient<dobot::SetPTPCommonParams>("/DobotServer/SetPTPCommonParams");
        dobot::SetPTPCommonParams srv;

        srv.request.velocityRatio = 50;
        srv.request.accelerationRatio = 50;
        client.call(srv);
    } while (0);

/*  ********************************************************************* */

    // 定义一个抓取物体的服务器
    
    Server server(n, "PlaceObjects", boost::bind(&execute_drop_off, _1, &server), false);
        
    // 服务器开始运行
    server.start();


    ros::spin();

    return 0;
}
