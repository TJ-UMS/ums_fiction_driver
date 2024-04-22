#ifndef __UMS_SERIAL_METHODS_H__
#define __UMS_SERIAL_METHODS_H__

//
// Created by anaple on 24-3-22.
//

#include <vector>
#include <cstdint>
#include <string>
#include <memory>
#include <iomanip>
#include <iostream>
#include <thread>
#include <future>
#include <unistd.h>
#include "serial/serial.h"
#include "crc.hpp"
#include "entity.hpp"


class UmsSerialMethods
{
public:
    std::shared_ptr<serial::Serial> getSerial();
    void sendTwistData(const std::shared_ptr<TwistCustom>& twistData);
    void loopUmsFictionData(const std::shared_ptr<FictionData>& FictionData);
    void sendMessageToGetParamData();
    void reStartSerial(const std::string& portName, int baudRate);
    void startSerial(const std::string& portName, int baudRate);
    void setParamsData(ParamsData paramsData);
    void getSysStatus();
    void sendEditParamsData(){
        ParamDataWrite();
    }
    void refuseController();
    UmsSerialMethods()
    {
        std::cout << "UmsSerial" << std::endl;
    };
    UmsSerialMethods(const std::string& portName, int baudRate)
    {
        std::cout << "UmsSerial2" << std::endl;
        startSerial(portName, baudRate);
    }
    ~UmsSerialMethods(){
        sp.reset();
    }

private:
    static int Rfid(std::vector<uint8_t> &byteVector);

    static std::string magneticDataProcess(const std::vector<uint8_t>& NativeData);

    static int32_t HexArrayToInt32(uint8_t *hexArray, size_t size);

    static float HexArrayToFloat32(uint8_t *hexArray, size_t size);
    // 参数数据写入
    bool ParamDataWrite();

    // ICD
    static ICDRemote convertBackDataToControl(int channel1Value, int channel2Value, int channel3Value);
    // RCBUS
    static RCSBUSRemote convertRCBusRemote(std::vector<uint8_t> &byteVector);

    void tdLoopUmsFictionData(const std::shared_ptr<serial::Serial>& Sp, const std::shared_ptr<FictionData>& FictionData);

    /**********************************************************************
    函数功能：消息帧内容转义
    入口参数：std::vector<std::string>& byteVector
    返回  值：byteVector
    **********************************************************************/
    static std::vector<uint8_t> CompoundVector(std::vector<uint8_t> &byteVector);

    /**********************************************************************
    函数功能：帧内容合成发送
    入口参数：uint8_t signbit 标志位        std::vector<uint8_t>& Vector 数据
    返回  值：无
    **********************************************************************/
    static std::vector<uint8_t> DataDelivery(uint8_t signbit, std::vector<uint8_t> &Vector);

    static void LowerParameterOperationInt(const std::string& basicString, uint8_t address, int32_t data, const std::shared_ptr<serial::Serial>& Sp);

    /**********************************************************************
    函数功能：下位参数读写操作 FLOAT
    入口参数：red_write读取或写入  address写入的地址  data写入的数据
    返回  值：无
    **********************************************************************/
    void LowerParameterOperation(const std::string& red_write, uint8_t address, float data, const std::shared_ptr<serial::Serial>& Sp);

    /**********************************************************************
    函数功能：数据校验
    入口参数：校验数据
    返回  值：是否通过校验
    **********************************************************************/
    static bool DataCheck(std::vector<uint8_t> &data);

    /**********************************************************************
    函数功能：计算并返回转换后的 double 数值
    入口参数：startIndex 开始下标    count 截取的元素个数   byteData 1组8bytes数据
    返回  值：double result
    **********************************************************************/
    static double DirectionalInterception(int startIndex, int count, const std::vector<uint8_t> &byteData);

    /**********************************************************************
    函数功能：将二进制数据转换为 double 类型
    入口参数：std::vector<uint8_t>& byteData
    返回  值：double result
    **********************************************************************/
    static double BinaryToDouble(const std::vector<uint8_t> &byteData);

    /**********************************************************************
    函数功能：函数将 double 转换为 std::vector<uint8_t> 表示
    入口参数：double value
    返回  值：无
    **********************************************************************/
    static std::vector<uint8_t> DoubleToBytes(double value);

    /**********************************************************************
    函数功能：下位数据还原
    入口参数：buffer
    返回  值：FictionData result
    **********************************************************************/
    static void EscapeVector(std::vector<uint8_t> &byteVector);

    static PowerInfo PowerDataProcess(const std::vector<uint8_t>& NativeData);
    static ImuInfo ImuDataProcess(std::vector<uint8_t> ImuData);
    static OdomInfo OdomDataProcess(const std::vector<uint8_t>& ImuData);
    static ParamsData ParamDataRead(uint8_t *data);
    std::shared_ptr<serial::Serial> sp;
    std::shared_ptr<FictionData> fictionData;
    void createSerial(const std::string& portName, int baudRate);

    std::atomic<bool> stopFlag = false;
    std::atomic<bool> timeoutOccurred{};
    std::chrono::steady_clock::time_point lastReceiveTime;
    std::thread spThread;
    std::thread rdThread;
    std::thread reThread;
    std::thread sysStatusThread;
    ParamsData inputParam{};


    void monitorTimeout();

    void loopToGetSysStatus();
};

#endif // UMS_SERIAL_METHODS_H_
