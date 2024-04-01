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
    void sendTwistData(std::shared_ptr<serial::Serial> Sp, std::shared_ptr<TwistCustom> twistData);
    void test(uint8_t a);
    void loopUmsFictionData(std::shared_ptr<serial::Serial> Sp, std::shared_ptr<FictionData> FictionData);
    void sendGetParamData(std::shared_ptr<serial::Serial> Sp);
    void reStartSerial(std::string portName, int baudRate);
    void startSerial(std::string portName, int baudRate);
    UmsSerialMethods()
    {
        std::cout << "UmsSerial" << std::endl;
    };
    UmsSerialMethods(std::string portName, int baudRate)
    {
        std::cout << "UmsSerial2" << std::endl;
        startSerial(portName, baudRate);
    }
    ~UmsSerialMethods(){
        sp.reset();
        stopFlag = true;
        if(spThread.joinable()){
            spThread.join();
        }
        if(rdThread.joinable()){
            rdThread.join();
        }
    }
private:
    int Rfid(std::vector<uint8_t> &byteVector);
    void tdLoopUmsFictionData(std::shared_ptr<serial::Serial> Sp, std::shared_ptr<FictionData> FictionData);
    // ICD
    ICDRemote convertBackDataToControl(int channel1Value, int channel2Value, int channel3Value);
    // RCBUS
    RCSBUSRemote convertRCBusRemote(std::vector<uint8_t> &byteVector);


    std::string magneticDataProcess(std::vector<uint8_t> NativeData);

    int32_t HexArrayToInt32(uint8_t *hexArray, size_t size);

    float HexArrayToFloat32(uint8_t *hexArray, size_t size);

    /**********************************************************************
    函数功能：消息帧内容转义
    入口参数：std::vector<std::string>& byteVector
    返回  值：byteVector
    **********************************************************************/
    std::vector<uint8_t> CompoundVector(std::vector<uint8_t> &byteVector);

    /**********************************************************************
    函数功能：帧内容合成发送
    入口参数：uint8_t signbit 标志位        std::vector<uint8_t>& Vector 数据
    返回  值：无
    **********************************************************************/
    std::vector<uint8_t> DataDelivery(uint8_t signbit, std::vector<uint8_t> &Vector);

    void LowerParameterOperationInt(std::string readOrwrite, uint8_t address, int32_t data, std::shared_ptr<serial::Serial> Sp);

    /**********************************************************************
    函数功能：下位参数读写操作 FLOAT
    入口参数：red_write读取或写入  address写入的地址  data写入的数据
    返回  值：无
    **********************************************************************/
    void LowerParameterOperation(std::string red_write, uint8_t address, float data, std::shared_ptr<serial::Serial> Sp);

    /**********************************************************************
    函数功能：数据校验
    入口参数：校验数据
    返回  值：是否通过校验
    **********************************************************************/
    bool DataCheck(std::vector<uint8_t> &data);

    /**********************************************************************
    函数功能：计算并返回转换后的 double 数值
    入口参数：startIndex 开始下标    count 截取的元素个数   byteData 1组8bytes数据
    返回  值：double result
    **********************************************************************/
    double DirectionalInterception(int startIndex, int count, const std::vector<uint8_t> &byteData);

    /**********************************************************************
    函数功能：将二进制数据转换为 double 类型
    入口参数：std::vector<uint8_t>& byteData
    返回  值：double result
    **********************************************************************/
    double BinaryToDouble(const std::vector<uint8_t> &byteData);

    /**********************************************************************
    函数功能：函数将 double 转换为 std::vector<uint8_t> 表示
    入口参数：double value
    返回  值：无
    **********************************************************************/
    std::vector<uint8_t> DoubleToBytes(double value);

    /**********************************************************************
    函数功能：下位数据还原
    入口参数：buffer
    返回  值：FictionData result
    **********************************************************************/
    void EscapeVector(std::vector<uint8_t> &byteVector);

    PowerInfo PowerDataProcess(std::vector<uint8_t> NativeData);
    ImuInfo ImuDataProcess(std::vector<uint8_t> ImuData);
    OdomInfo OdomDataProcess(std::vector<uint8_t> ImuData);
    ParamsData ParamDataRead(uint8_t *data);
    std::shared_ptr<serial::Serial> sp;
    void createSerial(std::string portName, int baudRate);

    std::atomic<bool> stopFlag = false;
    std::thread spThread;
    std::thread rdThread;
};

#endif // UMS_FICTION_DRIVER_MAIN_UMS_SERIAL_METHODS_H
