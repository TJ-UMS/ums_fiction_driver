#include <iostream>
#include <string>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <fstream>
#include <iomanip>
#include <thread>
#include "serial/serial.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int8.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"

using namespace std::chrono_literals;

class TrolleyControl : public rclcpp::Node
{
private:
public:
    TrolleyControl(/* args */) : Node("ums_fiction_driver_node")
    {
        this->declare_parameter<int>("baudrate");
        this->declare_parameter<std::string>("port");


        this->get_parameter<std::string>("port", serialPort);
        this->get_parameter<int>("baudrate", baudrate);



        RfidPub = this->create_publisher<std_msgs::msg::String>("rfid_ori_data", rclcpp::SensorDataQoS());
        MagnetSensorPub = this->create_publisher<std_msgs::msg::String>("magnet_data", rclcpp::SensorDataQoS());
        OdomPublisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::SensorDataQoS());
        TwistSub = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1, std::bind(&TrolleyControl::VelCallback, this, std::placeholders::_1));
        IoOut = this->create_subscription<std_msgs::msg::String>("IO", 1, std::bind(&TrolleyControl::IoCallback, this, std::placeholders::_1));
        MotorPowerControlSub = this->create_subscription<std_msgs::msg::String>("motor_power_control", 1, std::bind(&TrolleyControl::MotorPowerCallback, this, std::placeholders::_1));
        ImuPublisher = this->create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS());
        PowerInformation = this->create_publisher<std_msgs::msg::String>("PowerInformationData", rclcpp::SensorDataQoS());
        Timer = this->create_wall_timer(5ms, std::bind(&TrolleyControl::CarPubCallBack, this));
        serial::Timeout to = serial::Timeout::simpleTimeout(100);



        RCLCPP_INFO(this->get_logger(), "尝试打开: 串口%s,波特率%d",serialPort.c_str(),baudrate);




        sp.setPort(serialPort);
        sp.setBaudrate(baudrate);
        sp.setTimeout(to);

        try
        {
            sp.open();
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
            return;
        }
        if (sp.isOpen())
        {
            std::cout << "底层串口打开成功" << std::endl;
        }
        else
        {
            return;
        }

        // 初始化里程计数据
        current_time_ = this->now();
        last_time_ = this->now();
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;

        Thread = std::thread(&TrolleyControl::DataProcessingThread, this);
    }

    ~TrolleyControl()
    {
        Thread.join();
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr MagnetSensorPub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr RfidPub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr TwistSub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr IoOut;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr MotorPowerControlSub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr ImuPublisher;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr OdomPublisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr PowerInformation;
    std::shared_ptr<tf2_ros::TransformBroadcaster> TfBroadcaster;
    rclcpp::TimerBase::SharedPtr Timer;
    serial::Serial sp;
    int baudrate = 0;
    std::string serialPort;
    std::thread Thread;


    void CarPubCallBack();
    void DataProcessingThread();
    void DataProcessingThreadTwo();
    std::vector<uint8_t> DoubleToBytes(double value);
    uint16_t Crc16(const std::vector<uint8_t> &data);
    double BinaryToDouble(const std::vector<uint8_t> &byteData);
    void VelCallback(geometry_msgs::msg::Twist::SharedPtr twist_msg);
    void ImuDataProcess();
    void PowerData();
    void OdometerData();
    double DirectionalInterception(int startIndex, int count, const std::vector<uint8_t> &byteData);
    void Rfid(std::vector<uint8_t> &byteVector);
    void MagneticLineSensor();
    void EscapeVector(std::vector<uint8_t> &byteVector);
    void CompoundVector(std::vector<uint8_t> &byteVector);
    void IoCallback(std_msgs::msg::String::SharedPtr io_msg);
    void MotorPowerCallback(std_msgs::msg::String::SharedPtr power_msg);
    void MotorSpeedPositionControl(uint8_t id, double velocitypower);
    void DataDelivery(uint8_t signbit, std::vector<uint8_t> &Vector);

    double CarSpeed;
    double CarAngle;
    std::string MagneticSensorData;
    float UltrasonicSensorData;
    int RfidData;
    std::vector<uint8_t> ImuData;
    std::vector<uint8_t> NativeData;
    bool DataJudgment = true;
    bool Send_bit = false;
    bool Collision_Detection = false;
    int8_t Collision_Count = 0;

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
    } ImuStructural;

    rclcpp::Time current_time_;
    rclcpp::Time last_time_;
    double x_;
    double y_;
    double theta_;
};

/**********************************************************************
函数功能：订阅cmd_vel
入口参数：geometry_msgs::msg::Twist::SharedPtr twist_msg
返回  值：无
**********************************************************************/
void TrolleyControl::VelCallback(geometry_msgs::msg::Twist::SharedPtr twist_msg)
{
    CarSpeed = twist_msg->linear.x;
    CarAngle = twist_msg->angular.z;
    Send_bit = true;
}

/**********************************************************************
函数功能：订阅IO控制12v输出
入口参数：std_msgs::msg::String::SharedPtr io_msg
返回  值：无
**********************************************************************/
void TrolleyControl::IoCallback(std_msgs::msg::String::SharedPtr io_msg)
{
    std::vector<uint8_t> Control_io = {0x03, 0x03, 0x00, 0x00};
    if (io_msg->data == "1")
    {
        Control_io[1] = {0x03};
        RCLCPP_INFO(this->get_logger(), "开启12v输出。");
    }
    else if (io_msg->data == "0")
    {
        Control_io[1] = {0x00};
        RCLCPP_INFO(this->get_logger(), "关闭12v输出。");
    }

    DataDelivery(0x4f, Control_io);
}

/**********************************************************************
函数功能：订阅电机功率输出
入口参数：std_msgs::msg::String::SharedPtr power_msg
返回  值：无
**********************************************************************/
void TrolleyControl::MotorPowerCallback(std_msgs::msg::String::SharedPtr power_msg)
{
    std::string Motor_id = "0";      // 电机编号
    std::string Motor_pattern = "0"; // 电机输出模式
    double a = 0;
    Motor_id = power_msg->data[0];
    Motor_pattern = power_msg->data[1];
    if (Motor_pattern == "0")
    {
        a = -1;
    }
    else if (Motor_pattern == "1")
    {
        a = 1;
        RCLCPP_INFO(this->get_logger(), "关闭12v输出。");
    }

    RCLCPP_INFO(this->get_logger(), "电机%s,功率%s", Motor_id.c_str(), Motor_pattern.c_str());
    MotorSpeedPositionControl(static_cast<uint8_t>(std::stoi(Motor_id)), a);
}

/**********************************************************************
函数功能：电机速度功率控制协议转换
入口参数：id 电机id    velocitypower速度或功率
返回  值：无
**********************************************************************/
void TrolleyControl::MotorSpeedPositionControl(uint8_t id, double velocitypower)
{
    std::vector<uint8_t> SpeedPositionControl;
    std::vector<uint8_t> v_p = DoubleToBytes(velocitypower);
    SpeedPositionControl.push_back(id);
    SpeedPositionControl.insert(SpeedPositionControl.end(), v_p.begin(), v_p.end());

    DataDelivery(0x50, SpeedPositionControl);

    SpeedPositionControl.clear();
    v_p.clear();
}

/**********************************************************************
函数功能：主回调
入口参数：无
返回  值：无
**********************************************************************/
void TrolleyControl::CarPubCallBack()
{
    std::vector<uint8_t> SendHexData;

    if (Collision_Detection == true)
    {
        if (Collision_Count < 10)
        {
            CarSpeed = -0.08;
            CarAngle = 0.0;
            Collision_Count++;
        }
        else if (Collision_Count == 10)
        {
            CarSpeed = 0;
            Collision_Detection = false;
        }
    }

    std::vector<uint8_t> LeftWheelSpeed = DoubleToBytes(CarSpeed);
    std::vector<uint8_t> RightWheelSpeed = DoubleToBytes(CarSpeed);
    std::vector<uint8_t> AngularVelocity = DoubleToBytes(CarAngle);

    SendHexData.insert(SendHexData.end(), LeftWheelSpeed.begin(), LeftWheelSpeed.end());
    SendHexData.insert(SendHexData.end(), RightWheelSpeed.begin(), RightWheelSpeed.end());
    SendHexData.insert(SendHexData.end(), AngularVelocity.begin(), AngularVelocity.end());

    if (Send_bit == true)
    {
        DataDelivery(0x4b, SendHexData);
        Send_bit = false;
    }

    LeftWheelSpeed.clear();
    RightWheelSpeed.clear();
    AngularVelocity.clear();
    SendHexData.clear();
}

/**********************************************************************
函数功能：IMU数据解析发布
入口参数：无
返回  值：无
**********************************************************************/
void TrolleyControl::ImuDataProcess()
{
    std::vector<uint8_t> subVector;

    if (DataJudgment == true)
    {
        RCLCPP_INFO(get_logger(), "串口打开正确！！！");
        DataJudgment = false;
    }

    try
    {
        if (ImuData[4] == 0x00)
        {
            if (ImuData.size() == 32)
            {
                subVector.insert(subVector.begin(), ImuData.begin() + 5, ImuData.begin() + 13);
                ImuStructural.axaxis = BinaryToDouble(subVector);
                subVector.clear();

                subVector.insert(subVector.begin(), ImuData.begin() + 13, ImuData.begin() + 21);
                ImuStructural.ayaxis = BinaryToDouble(subVector);
                subVector.clear();

                subVector.insert(subVector.begin(), ImuData.begin() + 21, ImuData.begin() + 29);
                ImuStructural.azaxis = BinaryToDouble(subVector);
                subVector.clear();
            }
        }
        else if (ImuData[4] == 0x01)
        {
            if (ImuData.size() == 32)
            {
                subVector.insert(subVector.begin(), ImuData.begin() + 5, ImuData.begin() + 13);
                ImuStructural.gxaxis = BinaryToDouble(subVector);
                subVector.clear();

                subVector.insert(subVector.begin(), ImuData.begin() + 13, ImuData.begin() + 21);
                ImuStructural.gyaxis = BinaryToDouble(subVector);
                subVector.clear();

                subVector.insert(subVector.begin(), ImuData.begin() + 21, ImuData.begin() + 29);
                ImuStructural.gzaxis = BinaryToDouble(subVector);
                subVector.clear();
            }
        }
        else if (ImuData[4] == 0x02)
        {
            if (ImuData.size() == 40)
            {
                subVector.insert(subVector.begin(), ImuData.begin() + 5, ImuData.begin() + 13);
                ImuStructural.q0 = BinaryToDouble(subVector);
                subVector.clear();

                subVector.insert(subVector.begin(), ImuData.begin() + 13, ImuData.begin() + 21);
                ImuStructural.q1 = BinaryToDouble(subVector);
                subVector.clear();

                subVector.insert(subVector.begin(), ImuData.begin() + 21, ImuData.begin() + 29);
                ImuStructural.q2 = BinaryToDouble(subVector);
                subVector.clear();

                subVector.insert(subVector.begin(), ImuData.begin() + 29, ImuData.begin() + 37);
                ImuStructural.q3 = BinaryToDouble(subVector);
                subVector.clear();
            }
        }
        else if (ImuData[4] == 0x03)
        {
            if (ImuData.size() == 32)
            {
                subVector.insert(subVector.begin(), ImuData.begin() + 5, ImuData.begin() + 13);
                ImuStructural.pitch = BinaryToDouble(subVector);
                subVector.clear();

                subVector.insert(subVector.begin(), ImuData.begin() + 13, ImuData.begin() + 21);
                ImuStructural.roll = BinaryToDouble(subVector);
                subVector.clear();

                subVector.insert(subVector.begin(), ImuData.begin() + 21, ImuData.begin() + 29);
                ImuStructural.yaw = BinaryToDouble(subVector);
                subVector.clear();
            }
        }
    }
    catch (const std::exception &e)
    {
        // std::cerr << e.what() << '\n';
    }

    auto message = sensor_msgs::msg::Imu();
    message.header.stamp = this->now();
    message.header.frame_id = "imu_link";
    message.angular_velocity.x = ImuStructural.gxaxis;
    message.angular_velocity.y = ImuStructural.gyaxis;
    message.angular_velocity.z = ImuStructural.gzaxis;
    message.linear_acceleration.x = ImuStructural.axaxis;
    message.linear_acceleration.y = ImuStructural.ayaxis;
    message.linear_acceleration.z = ImuStructural.azaxis;
    message.orientation.x = ImuStructural.q0;
    message.orientation.y = ImuStructural.q1;
    message.orientation.z = ImuStructural.q2;
    message.orientation.w = ImuStructural.q3;
    tf2::Quaternion q;
    q.setRPY(ImuStructural.roll / 180 * M_PI, ImuStructural.pitch / 180 * M_PI, ImuStructural.yaw / 180 * M_PI);
    message.orientation.x = q.getX();
    message.orientation.y = q.getY();
    message.orientation.z = q.getZ();
    message.orientation.w = q.getW();

    // std::cout<<"roll:"<<ImuStructural.roll<<" pitch:"<<ImuStructural.pitch<<" yaw:"<<ImuStructural.yaw<<std::endl;

    ImuPublisher->publish(message);

    subVector.clear();
}

/**********************************************************************
函数功能：电源数据处理发布
入口参数：无
返回  值：无
**********************************************************************/
void TrolleyControl::PowerData()
{
    // 总线电流
    double bus = DirectionalInterception(4, 8, NativeData);
    // 5v输出
    double output = DirectionalInterception(12, 8, NativeData);
    // 输入电压
    double input = DirectionalInterception(20, 8, NativeData);
    // 19v输出
    double output19 = DirectionalInterception(28, 8, NativeData);

    std::ostringstream oss;
    oss << bus << "," << output << "," << input << "," << output19;
    std::string result = oss.str();

    auto msgs = std_msgs::msg::String();
    msgs.data = result;
    PowerInformation->publish(msgs);
}

/**********************************************************************
函数功能：里程计数据处理发布
入口参数：无
返回  值：无
**********************************************************************/
void TrolleyControl::OdometerData()
{
    // x方向速度 vx ，y 方向速度 vy ，角速度 ωz
    double vx_chassis = 0;
    double vy_chassis = 0;
    double wz_chassis = 0;
    if (NativeData.size() == 31)
    {
        vx_chassis = DirectionalInterception(4, 8, NativeData);
        vy_chassis = DirectionalInterception(12, 8, NativeData);
        wz_chassis = DirectionalInterception(20, 8, NativeData);
        // std::cout<<"vx:"<<vx<<" vy:"<<vy<<" wz:"<<wz<<std::endl;
    }

    if (!TfBroadcaster)
    {
        TfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this->shared_from_this());
    }

    // 获取时间
    current_time_ = this->now();

    // 计算时间间隔
    double dt = (current_time_ - last_time_).seconds();

    // 根据底盘速度计算机器人位姿
    double vx = vx_chassis;
    double vy = vy_chassis;
    double vth = wz_chassis;

    double delta_x = (vx * cos(theta_) - vy * sin(theta_)) * dt;
    double delta_y = (vx * sin(theta_) + vy * cos(theta_)) * dt;
    double delta_th = vth * dt;

    x_ += delta_x;
    y_ += delta_y;
    theta_ += delta_th;

    // 发布tf坐标变换
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time_;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0.0;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, theta_);
    odom_trans.transform.rotation.x = quat.x();
    odom_trans.transform.rotation.y = quat.y();
    odom_trans.transform.rotation.z = quat.z();
    odom_trans.transform.rotation.w = quat.w();

    TfBroadcaster->sendTransform(odom_trans);

    // 发布里程计消息
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time_;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_trans.transform.rotation;
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    OdomPublisher->publish(odom);

    last_time_ = current_time_;
}

/**********************************************************************
函数功能：计算并返回转换后的 double 数值
入口参数：startIndex 开始下标    count 截取的元素个数   byteData 1组8bytes数据
返回  值：double result
**********************************************************************/
double TrolleyControl::DirectionalInterception(int startIndex, int count, const std::vector<uint8_t> &byteData)
{
    size_t startIndexs = startIndex;
    size_t counts = count;
    std::vector<uint8_t> subVector(byteData.begin() + startIndexs, byteData.begin() + startIndexs + counts);
    double result = BinaryToDouble(subVector);
    return result;
}

/**********************************************************************
函数功能：将二进制数据转换为 double 类型
入口参数：std::vector<uint8_t>& byteData
返回  值：double result
**********************************************************************/
double TrolleyControl::BinaryToDouble(const std::vector<uint8_t> &byteData)
{
    if (byteData.size() != sizeof(double))
    {
        std::cerr << "错误：二进制数据长度不符合 double 数据类型的长度。" << std::endl;
        return 0.0;
    }
    double result;
    std::memcpy(&result, byteData.data(), sizeof(double));

    return result;
}

/**********************************************************************
函数功能：函数将 double 转换为 std::vector<uint8_t> 表示
入口参数：double value
返回  值：无
**********************************************************************/
std::vector<uint8_t> TrolleyControl::DoubleToBytes(double value)
{
    std::vector<uint8_t> bytes;
    uint8_t *ptr = reinterpret_cast<uint8_t *>(&value);

    for (size_t i = 0; i < sizeof(double); i++)
    {
        bytes.push_back(*ptr);
        ptr++;
    }

    return bytes;
}

/**********************************************************************
函数功能：计算给定数据的 CRC-16 校验和。
入口参数：需要计算 CRC-16 的输入数据。
返回  值：CRC-16 校验和值。
**********************************************************************/
uint16_t TrolleyControl::Crc16(const std::vector<uint8_t> &data)
{
    const uint16_t polynomial = 0xA001;
    uint16_t crc = 0xFFFF;

    for (auto byte : data)
    {
        crc ^= byte;
        for (int i = 0; i < 8; ++i)
        {
            if (crc & 1)
            {
                crc = (crc >> 1) ^ polynomial;
            }
            else
            {
                crc = crc >> 1;
            }
        }
    }

    return crc;
}

/**********************************************************************
函数功能：帧内容合成发送
入口参数：uint8_t signbit 标志位        std::vector<uint8_t>& Vector 数据
返回  值：无
**********************************************************************/
void TrolleyControl::DataDelivery(uint8_t signbit, std::vector<uint8_t> &Vector)
{
    std::vector<uint8_t> Send_data;
    std::vector<uint8_t> ResultBytes;
    // 数据长度计算
    std::size_t DataLength = Vector.size();
    // 处理数据长度
    std::vector<uint8_t> Data_length_;
    Data_length_.push_back(DataLength & 0xFF);
    if ((DataLength >> 8) != 0x00)
    {
        Data_length_.push_back((DataLength >> 8) & 0xFF);
    }
    Send_data.push_back(signbit);
    Send_data.insert(Send_data.end(), Data_length_.begin(), Data_length_.end());
    Send_data.insert(Send_data.end(), Vector.begin(), Vector.end());

    uint16_t result = Crc16(Send_data);
    ResultBytes.push_back(result & 0xFF);
    ResultBytes.push_back(result >> 8);

    Send_data.insert(Send_data.end(), ResultBytes.begin(), ResultBytes.end());

    CompoundVector(Send_data);

    Send_data.push_back(0x0A);
    Send_data.push_back(0x0D);
    Send_data.insert(Send_data.begin(), 0x3A);

    sp.write(Send_data);

    Send_data.clear();
    ResultBytes.clear();
    Data_length_.clear();
}

/**********************************************************************
函数功能：消息帧内容还原
入口参数：std::vector<std::string>& strVector
返回  值：无
**********************************************************************/
void TrolleyControl::EscapeVector(std::vector<uint8_t> &byteVector)
{
    for (size_t i = 0; i < byteVector.size() - 1; ++i)
    {
        if (byteVector[i] == 0x5c)
        {
            if (byteVector[i + 1] == 0x00)
            {
                // 将 5c 00 替换为 5c
                byteVector.erase(byteVector.begin() + i + 1);
            }
            else if (byteVector[i + 1] == 0x01)
            {
                // 将 5c 01 替换为 3a
                byteVector[i] = 0x3a;
                byteVector.erase(byteVector.begin() + i + 1);
            }
            else if (byteVector[i + 1] == 0x02)
            {
                // 将 5c 02 替换为 0a
                byteVector[i] = 0x0a;
                byteVector.erase(byteVector.begin() + i + 1);
            }
            else if (byteVector[i + 1] == 0x03)
            {
                // 将 5c 03 替换为 0d
                byteVector[i] = 0x0d;
                byteVector.erase(byteVector.begin() + i + 1);
            }
        }
    }
}

/**********************************************************************
函数功能：消息帧内容转义
入口参数：std::vector<std::string>& byteVector
返回  值：无
**********************************************************************/
void TrolleyControl::CompoundVector(std::vector<uint8_t> &byteVector)
{
    std::vector<uint8_t> modifiedVector; // 用于存储修改后的值

    for (uint8_t byte : byteVector)
    {
        if (byte == 0x5C)
        {
            modifiedVector.push_back(0x5C);
            modifiedVector.push_back(0x00);
        }
        else if (byte == 0x3A)
        {
            modifiedVector.push_back(0x5C);
            modifiedVector.push_back(0x01);
        }
        else if (byte == 0x0A)
        {
            modifiedVector.push_back(0x5C);
            modifiedVector.push_back(0x02);
        }
        else if (byte == 0x0D)
        {
            modifiedVector.push_back(0x5C);
            modifiedVector.push_back(0x03);
        }
        else
        {
            modifiedVector.push_back(byte);
        }
    }

    byteVector = modifiedVector; // 将原始向量替换为修改后的向量
}

/**********************************************************************
函数功能：多线程数据接收处理
入口参数：无
返回  值：无
**********************************************************************/
void TrolleyControl::DataProcessingThread()
{
    while (true)
    {
        std::string str;
        size_t n = sp.available();
        if (n != 0)
        {
            str = sp.readline();
            std::vector<uint8_t> buffer(str.begin(), str.end());
            NativeData = buffer;

            EscapeVector(NativeData);

            try
            {
                switch (NativeData[2])
                {
                case 0x41: // 电源树 总线电流、5V 输出、输入电压、19V 输出。
                    PowerData();
                    break;
                case 0x45: // 磁条数据
                    MagneticLineSensor();
                    break;
                case 0x51: // imu数据
                    ImuData = NativeData;
                    ImuDataProcess();
                    break;
                case 0x52: // rfid数据
                    Rfid(NativeData);
                    break;
                case 0x56: // 电机数据

                    break;
                case 0x57: // 电机控制数据

                    break;
                case 0x53: // 四个电机的相关数据

                    break;
                case 0x54: // 数字式温度传感器

                    break;
                case 0x55: // 超声数据
                    // 将位置 4 和 5 的数据组合成一个 16 位无符号整数
                    UltrasonicSensorData = static_cast<float>((NativeData[4] << 8) | NativeData[5]);
                    break;
                }
                if (NativeData[2] == 0x4b) // 里程计
                {
                    OdometerData();
                    // for (const auto &element : NativeData)
                    // {
                    //     std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(element) << " ";
                    // }
                    // std::cout << std::dec << std::endl;
                }
            }
            catch (const std::exception &e)
            {
                // std::cerr << e.what() << '\n';
            }
        }
        NativeData.clear();
        ImuData.clear();
        str.clear();
    }
}

void TrolleyControl::Rfid(std::vector<uint8_t> &byteVector)
{
    auto msg = std_msgs::msg::String();

    // 使用 std::stringstream 构建十六进制字符串
    std::stringstream ss;
    for (const auto &byte : byteVector)
    {
        // 将 uint8_t 格式化为十六进制并追加到 stringstream 中
        ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
    }

    // 从 stringstream 中获取字符串
    std::string hexString = ss.str();
    msg.data = hexString;
    if (hexString.length() == 46){
        RfidPub->publish(msg);
    }   
}

void TrolleyControl::MagneticLineSensor()
{
    std::stringstream ss;
    for (uint8_t hexValue : NativeData)
    {
        ss << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(hexValue);
    }
    std::string FullHexString = ss.str();
    // 截取字符串的一部分并赋值给 MagneticSensorData
    size_t startIndex = 8; // 从第三个字符开始（索引从0开始）
    size_t length = 32;    // 截取32个字符
    MagneticSensorData = FullHexString.substr(startIndex, length);
    if (MagneticSensorData[21] == 'f')
    {
        Collision_Detection = true;
        Collision_Count = 0;
    }

    auto msg = std_msgs::msg::String();
    msg.data =  MagneticSensorData;
    MagnetSensorPub->publish(msg);

    rclcpp::Time RosTimestamp = rclcpp::Clock().now();
    MagneticSensorData.clear();
}
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrolleyControl>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
