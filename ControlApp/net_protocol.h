#ifndef NET_PROTOCOL_H
#define NET_PROTOCOL_H


#include <cstdint>


const uint16_t YOUBOT_PORT = 10000;

enum class GripControl : uint8_t {WAIT, COMPRESS, OPEN};

#pragma pack(push, 1)
struct YoubotMsg
{
    int16_t x_vel = 0;
    int16_t y_vel = 0;
    int16_t ang_speed = 0;
    int16_t axis1;
    int16_t axis2;
    int16_t axis3;
    int16_t axis4;
    int16_t axis5;
    GripControl grip_cmd = GripControl::WAIT;
};
#pragma pack(pop)
const size_t YOUBOT_MSG_SIZE = sizeof(YoubotMsg);

#endif
