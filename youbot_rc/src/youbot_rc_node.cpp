#include "youbot_rc/youbot_rc_node.hpp"

#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/io.hpp>
#include <algorithm>


YoubotServer::YoubotServer()
{   
    cmdVelPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    //armPosPub = n.advertise<brics_actuator::JointPositions > ("arm_1/arm_controller/position_command", 1);
	//gripperPosPub = n.advertise<brics_actuator::JointPositions > ("arm_1/gripper_controller/position_command", 1);

    tcpServer.set_socket("", 10001); // TODO ip
    tcpServer.set_keepalive(1, 1, 1);
    if(tcpServer.socket_bind() < 0) {
        std::cout << "Bind error" << std::endl;
        ros::shutdown();
    }

    std::cout << "Init success!" << std::endl;
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
    /*
    brics_actuator::JointPositions command;

    vector <brics_actuator::JointValue> arm_joint_positions;
    arm_joint_positions.resize(5);

    for (int idx = 0; idx < 5; ++idx) 
    {
        armJointPositions[i].joint_uri = std::string("arm_joint_")+ std::to_string(idx + 1);
        armJointPositions[i].value = rxMsg.axis[idx] / 100.0;
        armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
    };

    command.positions = arm_joint_positions;
    armPosPub.publish(command);
    */
}


void YoubotServer::pub_gripper_joint()
{
    double new_gripper_pos = 0;
    
    switch (rxMsg.grip_cmd)
    {
    case GripControl::WAIT:
        new_gripper_pos = gripperPosition;
        break;
    case GripControl::COMPRESS:
        new_gripper_pos = gripperPosition - gripperSpeed;
        break;
    case GripControl::OPEN:
        new_gripper_pos = gripperPosition + gripperSpeed;
        break;
    default:
        break;
    }

    std::min(new_gripper_pos, 0.0);
    std::max(new_gripper_pos, 2.0);

    /*
    brics_actuator::JointPositions command;
    std::vector<brics_actuator::JointValue> gripper_joint_positions;
    gripperJointPositions.resize(2);

    gripperJointPositions[0].joint_uri = "gripper_finger_joint_l";
    gripperJointPositions[0].value = readValue;
    gripperJointPositions[0].unit = boost::units::to_string(boost::units::si::meter);

    gripperJointPositions[1].joint_uri = "gripper_finger_joint_r";
    gripperJointPositions[1].value = readValue;
    gripperJointPositions[1].unit = boost::units::to_string(boost::units::si::meter);

    command.positions = gripperJointPositions;
    gripperPosPub.publish(command);
    */
}


void YoubotServer::connect()
{
    std::cout << "Waint connection" << std::endl;

    int res = tcpServer.socket_listen();

    if(res < 0) {
        std::cout << "Client connection error! Code: " << res << std::endl;
        return;
    }

    isConnected = true;

    std::cout << "Connected" << std::endl;
}


int YoubotServer::receive_command()
{
    if (tcpServer.receive(reinterpret_cast<char*>(&rxMsg), YOUBOT_MSG_SIZE) < 0) {
        std::cout << "Receive error!" << std::endl;
        return -1;
    } 
    
    /* for (const char& i: rx_buffer_)
        std::cout << std::hex << (int) static_cast<uint8_t>(i) << " ";
    std::cout << std::dec << std::endl; */

    update_state();

    if (tcpServer.send_mes(reinterpret_cast<char*>(&rxMsg), YOUBOT_MSG_SIZE) < 0) {
        std::cout << "Send error!" << std::endl;
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

    if (rxMsg.grip_cmd != youbotState.grip_cmd) {
        pub_gripper_joint();
        state_changed = true;
    }

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
    ros::init(argc, argv, "youbot_rc");

    YoubotServer youbot_server;
    youbot_server.spin();

    return 0;
}