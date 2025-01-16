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
#include "serial/serial.h"
#include "crc.hpp"
#include <unistd.h>
#include "entity.hpp"
#include <log4cpp/Category.hh>
#include <log4cpp/OstreamAppender.hh>
#include <log4cpp/PatternLayout.hh>
#include "queue.h"

using namespace std;

class UmsSerialMethods
{
public:
    std::shared_ptr<serial::Serial> getSerial();
    void sendTwistData(const std::shared_ptr<TwistCustom> &twistData);
    void loopUmsFictionData(const std::shared_ptr<FictionData> &FictionData);
    void sendMessageToGetParamData();
    void reStartSerial(const std::string &portName, int baudRate);
    void startSerial(const std::string &portName, int baudRate);
    void setParamsData(ParamsData paramsData);
    void sendEditParamsData();
    void refuseController();
    UmsSerialMethods();
    UmsSerialMethods(const std::string &portName, int baudRate, bool isDebug, int queueSize,AgreementVersion agreementVersion);
	void stop() {
        stopFlag = true;
        cleanup();
    }

    // 添加状态查询方法
    bool isRunning() const {
        return !stopFlag;
    }

    // 修改析构函数
    ~UmsSerialMethods() {
        stop();
    }
private:
    void cleanup();
    bool checkDataLength(uint8_t signLength, size_t size);
    bool checkSignValue(uint8_t sign);
    int Rfid(std::vector<uint8_t> &byteVector);
    void getSysStatus();
    std::string magneticDataProcess(const std::vector<uint8_t> &NativeData);

    int32_t HexArrayToInt32(uint8_t *hexArray, size_t size);

    float HexArrayToFloat32(uint8_t *hexArray, size_t size);
    // 参数数据写入
    bool ParamDataWrite();
    // ICD
    ICDRemote convertBackDataToControl(int channel1Value, int channel2Value, int channel3Value);
    // RCBUS
    RCSBUSRemote convertRCBusRemote(std::vector<uint8_t> &byteVector);

    void tdLoopUmsFictionData(const std::shared_ptr<serial::Serial> &Sp, const std::shared_ptr<FictionData> &FictionData);

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

    void LowerParameterOperationInt(const std::string &basicString, uint8_t address, int32_t data, const std::shared_ptr<serial::Serial> &Sp);

    /**********************************************************************
    函数功能：下位参数读写操作 FLOAT
    入口参数：red_write读取或写入  address写入的地址  data写入的数据
    返回  值：无
    **********************************************************************/
    void LowerParameterOperation(const std::string &red_write, uint8_t address, float data, const std::shared_ptr<serial::Serial> &Sp);

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

    PowerInfo PowerDataProcess(const std::vector<uint8_t> &NativeData);
    ImuInfo ImuDataProcess(std::vector<uint8_t> ImuData);
    OdomInfo OdomDataProcess(const std::vector<uint8_t> &ImuData);
    ParamsData ParamDataRead(uint8_t *data);

    void createSerial(const std::string &portName, int baudRate);
    void monitorTimeout();
    void loopToGetSysStatus();
    ImuInfo ImuStructural{};
    AgreementVersion runtimeVersion;

    std::shared_ptr<serial::Serial> sp;
    std::shared_ptr<FictionData> fictionData;

    std::atomic<bool> stopFlag {false};
    std::atomic<bool> timeoutOccurred{};
    std::chrono::steady_clock::time_point lastReceiveTime;
    std::thread spThread;
    std::thread rdThread;
    std::thread reThread;
    std::thread sysStatusThread;
    ParamsData inputParam{};
    log4cpp::Category &root = log4cpp::Category::getRoot();

    std::shared_ptr<CircularQueue> circularQueue;

    vector<uint8_t> signedValue = {0x41, 0x42, 0x45, 0x51, 0x52, 0x46, 0x49, 0x4b, 0x53, 0x54, 0x55};

    std::string stringToHex(const std::string &input);

    bool extractPacket(vector<uint8_t> &buffer, vector<uint8_t> &packet);

    std::vector<uint8_t> comFrameReduction(std::vector<uint8_t> &arr);
    size_t findElement(const vector<uint8_t> &buffer, uint8_t element, size_t startPos);

    void readSerialData();
};

#endif // UMS_SERIAL_METHODS_H_
