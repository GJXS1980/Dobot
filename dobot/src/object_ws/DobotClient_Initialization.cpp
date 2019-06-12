
#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include "dobot/SetCmdTimeout.h"
#include "dobot/SetQueuedCmdClear.h"
#include "dobot/SetQueuedCmdStartExec.h"
#include "dobot/SetQueuedCmdForceStopExec.h"
#include "dobot/GetDeviceVersion.h"
#include <unistd.h>
 
#include "dobot/SetEndEffectorParams.h"
#include "dobot/SetPTPJointParams.h"
#include "dobot/SetPTPCoordinateParams.h"
#include "dobot/SetPTPJumpParams.h"
#include "dobot/SetPTPCommonParams.h"
#include "dobot/SetPTPCmd.h"

#include "dobot/SetEndEffectorSuctionCup.h"
#include "dobot/SetHOMEParams.h"
#include "dobot/SetHOMECmd.h"
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "DobotClient");   // init the DobotClient node
    ros::NodeHandle n;   // create node handle
 
    ros::ServiceClient client;
 
    // SetCmdTimeout
    client = n.serviceClient<dobot::SetCmdTimeout>("/DobotServer/SetCmdTimeout");
    dobot::SetCmdTimeout srv1;
    srv1.request.timeout = 3000;
    if (client.call(srv1) == false) {
        ROS_ERROR("Failed to call SetCmdTimeout. Maybe DobotServer isn't started yet!");
        return -1;
    }
 
    // Clear the command queue
    client = n.serviceClient<dobot::SetQueuedCmdClear>("/DobotServer/SetQueuedCmdClear");
    dobot::SetQueuedCmdClear srv2;
    client.call(srv2);
 
    // Start running the command queue
    client = n.serviceClient<dobot::SetQueuedCmdStartExec>("/DobotServer/SetQueuedCmdStartExec");
    dobot::SetQueuedCmdStartExec srv3;
    client.call(srv3);
 
    // Get device version information
    client = n.serviceClient<dobot::GetDeviceVersion>("/DobotServer/GetDeviceVersion");
    dobot::GetDeviceVersion srv4;
    client.call(srv4);
    if (srv4.response.result == 0) {
        ROS_INFO("Device version:%d.%d.%d", srv4.response.majorVersion, srv4.response.minorVersion, srv4.response.revision);
    } else {
        ROS_ERROR("Failed to get device version information!");
    }
 
    // Set PTP coordinate parameters
    do {
        client = n.serviceClient<dobot::SetPTPCoordinateParams>("/DobotServer/SetPTPCoordinateParams");
        dobot::SetPTPCoordinateParams srv;
 
        srv.request.xyzVelocity = 100;
        srv.request.xyzAcceleration = 100;
        srv.request.rVelocity = 100;
        srv.request.rAcceleration = 100;
        client.call(srv);
    } while (0);
 
    // Set PTP common parameters
    do {
        client = n.serviceClient<dobot::SetPTPCommonParams>("/DobotServer/SetPTPCommonParams");
        dobot::SetPTPCommonParams srv;
 
        srv.request.velocityRatio = 50;
        srv.request.accelerationRatio = 50;
        client.call(srv);
    } while (0);
 
    do{
        client = n.serviceClient<dobot::SetHOMEParams>("/DobotServer/SetHOMEParams");
        dobot::SetHOMEParams home;
        home.request.x = 200;
        home.request.y = 0;
        home.request.z = 0;
        home.request.r = 0;
        home.request.isQueued = 1;
        client.call(home); 
 
    } while(0);
 
    
    return 0;
}

