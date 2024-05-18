#ifndef NET_PROTOCOL_H
#define NET_PROTOCOL_H


#include <cstdint>


const uint16_t YOUBOT_PORT = 10100;

enum class GripControl : uint8_t {WAIT, COMPRESS, OPEN};

#pragma pack(push, 1)
struct Msg
{
    uint16_t x_vel;
    uint16_t y_vel;
    uint16_t ang_speed;
    uint16_t axis1;
    uint16_t axis2;
    uint16_t axis3;
    uint16_t axis4;
    uint16_t axis5;
    GripControl grip_cmd;
};
#pragma pack(pop)


#endif
