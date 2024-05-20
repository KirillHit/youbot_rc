#include "youbot_rc/youbot_rc_node.hpp"

#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/io.hpp>
#include <algorithm>


YoubotServer::YoubotServer()
{   
    cmdVelPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    armPosPub = nh.advertise<brics_actuator::JointPositions > ("arm_1/arm_controller/position_command", 1);
	gripperPosPub = nh.advertise<brics_actuator::JointPositions > ("arm_1/gripper_controller/position_command", 1);

    int port;
    nh.getParam("youbot_rc_node/port", port);
    tcpServer.set_socket("", port);
    tcpServer.set_keepalive(1, 1, 1);
    if(tcpServer.socket_bind() < 0) {
        ROS_ERROR("Bind error. Try changing the port, wait a few minutes or reboot.");
        ros::shutdown();
    }

    ROS_INFO("Init success!");
}


YoubotServer::~YoubotServer()
{
    pub_cmdvel(0, 0, 0);
}


void YoubotServer::pub_cmdvel(double x, double y, double angle)
{  
    geometry_msgs::Twist twist_msg;
    
    twist_msg.linear.x = x;
    twist_msg.linear.y = y;
    twist_msg.linear.z = 0;
    twist_msg.angular.x = 0;
    twist_msg.angular.y = 0;
    twist_msg.angular.z = angle;

    cmdVelPub.publish(twist_msg);
}


void YoubotServer::pub_arm_joint()
{    
    brics_actuator::JointPositions command;

    std::vector <brics_actuator::JointValue> arm_joint_positions;
    arm_joint_positions.resize(5);

    for (int idx = 0; idx < 5; ++idx) 
    {
        arm_joint_positions[idx].joint_uri = std::string("arm_joint_")+ std::to_string(idx + 1);
        arm_joint_positions[idx].value = rxMsg.axis[idx] / 100.0;
        arm_joint_positions[idx].unit = boost::units::to_string(boost::units::si::radians);
    };

    command.positions = arm_joint_positions;
    armPosPub.publish(command);
}


void YoubotServer::pub_gripper_joint()
{   
    switch (rxMsg.grip_cmd)
    {
    case GripControl::WAIT:
        return;
        break;
    case GripControl::COMPRESS:
        gripperPosition -= gripperSpeed;
        break;
    case GripControl::OPEN:
        gripperPosition += gripperSpeed;
        break;
    default:
        break;
    }

    gripperPosition = std::max(gripperPosition, 0.0);
    gripperPosition = std::min(gripperPosition, 0.0115);

    brics_actuator::JointPositions command;
    std::vector<brics_actuator::JointValue> gripper_joint_positions;
    gripper_joint_positions.resize(2);

    gripper_joint_positions[0].joint_uri = "gripper_finger_joint_l";
    gripper_joint_positions[0].value = gripperPosition;
    gripper_joint_positions[0].unit = boost::units::to_string(boost::units::si::meter);

    gripper_joint_positions[1].joint_uri = "gripper_finger_joint_r";
    gripper_joint_positions[1].value = gripperPosition;
    gripper_joint_positions[1].unit = boost::units::to_string(boost::units::si::meter);

    command.positions = gripper_joint_positions;
    gripperPosPub.publish(command);
}


void YoubotServer::connect()
{
    ROS_INFO("Waint connection");

    int res = tcpServer.socket_listen();

    if(res < 0) {
        ROS_WARN("Client connection error! Code: %d", res);
        return;
    }

    isConnected = true;

    ROS_INFO("Connected");
}


int YoubotServer::receive_command()
{
    if (tcpServer.receive(reinterpret_cast<char*>(&rxMsg), YOUBOT_MSG_SIZE) < 0) {
        ROS_WARN("Receive error!");
        return -1;
    }

    update_state();

    if (tcpServer.send_mes(reinterpret_cast<char*>(&youbotState), YOUBOT_MSG_SIZE) < 0) {
        ROS_WARN("Send error!");
        return -1;
    }

    return 0;
}


void YoubotServer::update_state()
{
    bool state_changed = false;
    
    if (rxMsg.ang_speed != youbotState.ang_speed || rxMsg.x_vel != youbotState.x_vel || rxMsg.y_vel != youbotState.y_vel) 
    {
        pub_cmdvel(rxMsg.x_vel/100.0, rxMsg.y_vel/100.0, rxMsg.ang_speed/100.0);
        state_changed = true;
    }

    if ((rxMsg.axis[0] != youbotState.axis[0]) || (rxMsg.axis[1] != youbotState.axis[1])
        || (rxMsg.axis[2] != youbotState.axis[2]) || (rxMsg.axis[3] != youbotState.axis[3])
        || (rxMsg.axis[4] != youbotState.axis[4]))
    {
        pub_arm_joint();
        state_changed = true;
    }

    pub_gripper_joint();

    if (state_changed) youbotState = rxMsg;
}


void YoubotServer::spin()
{
    while (nh.ok()) 
    {
        if (!isConnected) {
            connect();
            continue;
        }
        
        if (receive_command() < 0) {
            isConnected = false;
            pub_cmdvel(0, 0, 0);
        }

        ros::spinOnce();
        rate.sleep();
    }
}


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "youbot_rc_node");

    YoubotServer youbot_server;
    youbot_server.spin();

    return 0;
}