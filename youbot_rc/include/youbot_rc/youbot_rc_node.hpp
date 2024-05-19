#ifndef YOUBOT_RC_NODE_HPP
#define YOUBOT_RC_NODE_HPP

#include <ros/ros.h>
#include <cstddef>
#include <iostream>
#include <vector>

#include <geometry_msgs/Twist.h>
// #include <brics_actuator/JointPositions.h>

#include "simple_socket/simple_socket.hpp"
#include "net_protocol.hpp"

class YoubotServer
{
public:
    YoubotServer();
    ~YoubotServer();

    void spin();
    void pub_cmdvel(double x, double y, double angle);
    void pub_arm_joint();
    void pub_gripper_joint();

private:
    void connect();
    int receive_command();
    void update_state();

private:
    ros::NodeHandle nh;
    ros::Publisher cmdVelPub;
    ros::Publisher armPosPub;
    ros::Publisher gripperPosPub;
    ros::Rate rate = ros::Rate(30);

    sockets::TCPServer tcpServer;
    bool isConnected = false;
    YoubotMsg rxMsg;
    YoubotMsg youbotState;

    double gripperPosition = 0;
    double gripperSpeed = 0.1;
};


#endif
