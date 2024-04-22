#include <cstdint>
#include <bitset>
#include <iostream>
#include <string>

#ifndef PARAMS_H
#define PARAMS_H

struct ParamsData
{
    float KP;
    float KI;
    float KD;
    float LA;
    float LB;
    float MPE;
    float MPC;
    int32_t KMTT;
    float IMU_Z;
};

struct ImuInfo
{
    double axaxis;
    double ayaxis;
    double azaxis;
    double gxaxis;
    double gyaxis;
    double gzaxis;
    double q0;
    double q1;
    double q2;
    double q3;
    double roll;
    double yaw;
    double pitch;
};

enum ControlStatus {
    EMERGENCY_STOP,   // 急停状态
    PROGRAM_CONTROL,  // 程序控制状态
    REMOTE_CONTROL    // 遥控控制状态
    };
    
enum SysStatus {
    SYS_STANDBY,
    SYS_RUNNING_URT,
    SYS_RUNNING_JOY,
    SYS_EMG_PSB,
    SYS_EMG_APT
}

int hexToD(std::string hexString)
{
    try
    {
        // 使用std::stoi进行转换
        int decimalNumber = std::stoi(hexString, 0, 16);

        // 打印结果
        // std::cout << "16进制字符串 " << hexString << " 转换为10进制: " << decimalNumber << std::endl;
        return decimalNumber;
    }
    catch (const std::invalid_argument &e)
    {

        return -1;
    }
    catch (const std::out_of_range &e)
    {

        return -1;
    }

    return -1;
}

#endif
