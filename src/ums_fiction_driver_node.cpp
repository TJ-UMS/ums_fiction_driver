//
// Created by anaple on 24-3-25.
//

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ums_serial_methods.hpp"
#include "battery.h"
#include "tf2/LinearMath/Quaternion.h"

class UMSFictionROS2 : public rclcpp::Node
{
public:
    std::shared_ptr<serial::Serial> currentSerial = nullptr;
    std::shared_ptr<FictionData> currentFictionData = std::make_shared<FictionData>();
    /*声明参数*/
    std::string port;
    int baudrate;
    bool imu_enable = true;
    bool odom_enable = true;

    UMSFictionROS2() : Node("ums_fiction_driver_node")
    {

        this->declare_parameter<int>("con_baudrate",2000000 );
        this->declare_parameter<std::string>("con_port", "/dev/ttyUSB0");
        this->get_parameter<std::string>("con_port", port);                // 端口号
        this->get_parameter<int>("con_baudrate", baudrate);                // 波特率
        this->declare_parameter<std::string>("LC_read_write", "read"); // 读或写
        this->declare_parameter<float>("KP", 0.0);                       // 速度闭环控制器 比例增益
        this->declare_parameter<float>("KI", 0.0);                       // 速度闭环控制器 积分增益
        this->declare_parameter<float>("KD", 0.0);                       // 速度闭环控制器 微分增益
        this->declare_parameter<float>("MPE", 0.0);                      // 速度测算 脉冲周数比
        this->declare_parameter<float>("MPC", 0.0);                      // 速度测算 轮圆周长 单位：m
        this->declare_parameter<float>("LA", 0.0);                       // 底盘尺寸 轮间距/2 单位：m
        this->declare_parameter<float>("LB", 0.0);                       // 底盘尺寸 轴间距/2 单位：m
        this->declare_parameter<int32_t>("KMTT", 0);                   // 运动学模型类型
        this->declare_parameter<float>("IMU_Z", 0.0);                  // IMU Z 轴 航向角零偏修正偏置值
        this->declare_parameter<bool>("odom_enable", true);             // IMU enable
        this->declare_parameter<bool>("imu_enable", true);              // odom enable




        umsSerialMethodsPtr->startSerial(port, baudrate);
        this->get_parameter("odom_enable", odom_enable);
        this->get_parameter("imu",imu_enable);
        // 初始化发布器
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        ultrasonic_publisher_ = this->create_publisher<std_msgs::msg::Float32>("ultrasonic", 10);
        temperature_publisher_ = this->create_publisher<std_msgs::msg::Float32>("temperature", 10);
        battery_publisher_ = this->create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 10);
        rfid_publisher_ = this->create_publisher<std_msgs::msg::String>("rfid_data", 10);
        magnetic_publisher_ = this->create_publisher<std_msgs::msg::String>("magnetic_data", 10);
        sysStatus_publisher_ = this->create_publisher<std_msgs::msg::Int8>("sys_status", 10);

        // 初始化订阅器
        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&UMSFictionROS2::cmd_vel_callback, this, std::placeholders::_1));
        // 初始化参数监听
        parameter_event_subscriber_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
                "/parameter_events", 10, std::bind(&UMSFictionROS2::parameterCallback, this, std::placeholders::_1));


        currentSerial = umsSerialMethodsPtr->getSerial();
        // 初始化定时器
        timer_ = this->create_wall_timer(
                std::chrono::milliseconds(5), // 1000ms / 60Hz = 约16.67ms
                std::bind(&UMSFictionROS2::timer_callback, this));
        idle_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(17), // 1000ms / 60Hz = 约16.67ms
                std::bind(&UMSFictionROS2::twistIdle,this)
        );
        idle_time_ = std::make_shared<rclcpp::Time>(this->now());
        paramWriteByYaml();
        umsSerialMethodsPtr->loopUmsFictionData(currentFictionData);
    }
    ~UMSFictionROS2()
    {
        currentFictionData.reset();
        currentSerial.reset();
        umsSerialMethodsPtr.reset();
    }

private:
    void parameterCallback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
    {
        bool dwChange = false;
        for (const auto &changed_parameter : event->changed_parameters)
        {   if(changed_parameter.name == "con_port"){
                umsSerialMethodsPtr->reStartSerial(changed_parameter.value.string_value, baudrate);
                port  = changed_parameter.value.string_value;
                currentSerial.reset();
                currentSerial =  umsSerialMethodsPtr->getSerial();
            }else if(changed_parameter.name == "con_baudrate"){
                umsSerialMethodsPtr->reStartSerial(port, changed_parameter.value.integer_value);
                baudrate = changed_parameter.value.integer_value;
                currentSerial.reset();
                currentSerial = umsSerialMethodsPtr->getSerial();
            } else if (changed_parameter.name == "imu_enable"){
                imu_enable = changed_parameter.value.bool_value;
        } else if (changed_parameter.name == "odom_enable"){
                odom_enable = changed_parameter.value.bool_value;
        }
            if(currentSerial != nullptr) {
                if (changed_parameter.name == "KP" &&
                    changed_parameter.value.double_value != currentFictionData->paramsData.KP) {
                    currentFictionData->paramsData.KP = changed_parameter.value.double_value;
                    dwChange = true;
                } else if (changed_parameter.name == "KI" &&
                           changed_parameter.value.double_value != currentFictionData->paramsData.KI) {
                    currentFictionData->paramsData.KI = changed_parameter.value.double_value;
                    dwChange = true;

                } else if (changed_parameter.name == "KD" &&
                           changed_parameter.value.double_value != currentFictionData->paramsData.KD) {
                    currentFictionData->paramsData.KD = changed_parameter.value.double_value;
                    dwChange = true;
                } else if (changed_parameter.name == "MPE" &&
                           changed_parameter.value.double_value != currentFictionData->paramsData.MPE) {
                    currentFictionData->paramsData.MPE = changed_parameter.value.double_value;
                    dwChange = true;

                } else if (changed_parameter.name == "MPC" &&
                           changed_parameter.value.double_value != currentFictionData->paramsData.MPC) {
                    currentFictionData->paramsData.MPC = changed_parameter.value.double_value;
                    dwChange = true;
                } else if (changed_parameter.name == "LA" &&
                           changed_parameter.value.double_value != currentFictionData->paramsData.LA) {
                    currentFictionData->paramsData.LA = changed_parameter.value.double_value;
                    dwChange = true;
                } else if (changed_parameter.name == "LB" &&
                           changed_parameter.value.double_value != currentFictionData->paramsData.LB) {
                    currentFictionData->paramsData.LB = changed_parameter.value.double_value;
                    dwChange = true;
                } else if (changed_parameter.name == "KMTT" &&
                           changed_parameter.value.integer_value != currentFictionData->paramsData.KMTT) {
                    currentFictionData->paramsData.KMTT = changed_parameter.value.integer_value;
                    dwChange = true;

                } else if (changed_parameter.name == "IMU_Z" &&
                           changed_parameter.value.double_value != currentFictionData->paramsData.IMU_Z) {
                    currentFictionData->paramsData.IMU_Z = changed_parameter.value.double_value;
                    dwChange = true;
                }
                if(dwChange){
                    try {
                        umsSerialMethodsPtr->setParamsData(currentFictionData->paramsData);
                        umsSerialMethodsPtr->sendEditParamsData();
                        umsSerialMethodsPtr->sendMessageToGetParamData();
                        RCLCPP_INFO(this->get_logger(), "参数写入成功");
                    }catch (std::exception &e) {
                        RCLCPP_INFO(this->get_logger(), "参数写入失败");
                    }

                }
            }
        }
    }
    void ImuDataPublish(ImuInfo ImuStructural)
    {

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
        imu_publisher_->publish(message);
    }

    void OdometerDataPublish(OdomInfo data)
    {
        if (last_time_ == nullptr)
        {

            last_time_ = std::make_shared<rclcpp::Time>(this->now());
        }

        // 获取时间
        rclcpp::Time current_time_ = this->now();

        // 计算时间间隔
        double dt = (current_time_ - *last_time_).seconds();
        // 根据底盘速度计算机器人位姿
        double vx = data.delta_x;
        double vy = data.delta_y;
        double vth = data.delta_th;

        double delta_x = (vx * cos(theta_) - vy * sin(theta_)) * dt;
        double delta_y = (vx * sin(theta_) + vy * cos(theta_)) * dt;
        double delta_th = vth * dt;

        x_ += delta_x;
        y_ += delta_y;
        theta_ += delta_th;

        tf2::Quaternion quat;
        quat.setRPY(0, 0, theta_);

        // 发布里程计消息
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time_;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation.y = quat.y();
        odom.pose.pose.orientation.x = quat.x();
        odom.pose.pose.orientation.z = quat.z();
        odom.pose.pose.orientation.w = quat.w();
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;
        odom_publisher_->publish(odom);
        last_time_ = std::make_shared<rclcpp::Time>(current_time_);
    }



    void publishParams(const ParamsData &data)
    {
        RCLCPP_INFO(this->get_logger(), "参数更新");
    try{
        std::vector<rclcpp::Parameter> all_new_parameters;
        RCLCPP_INFO(this->get_logger(), "KP：%0.2f",data.KP);
        RCLCPP_INFO(this->get_logger(), "KI：%0.2f",data.KI);
        RCLCPP_INFO(this->get_logger(), "KD：%0.2f",data.KD);
        RCLCPP_INFO(this->get_logger(), "MPE：%0.2f",data.MPE);
        RCLCPP_INFO(this->get_logger(), "MPC：%0.2f",data.MPC);
        RCLCPP_INFO(this->get_logger(), "KMTT：%d",data.KMTT);
        RCLCPP_INFO(this->get_logger(), "LB：%0.2f",data.LB);
        RCLCPP_INFO(this->get_logger(), "LA：%0.2f",data.LA);
        RCLCPP_INFO(this->get_logger(), "IMU_Z：%0.2f",data.IMU_Z);
        all_new_parameters.emplace_back("KP", data.KP);
        all_new_parameters.emplace_back("KI", data.KI);
        all_new_parameters.emplace_back("KD", data.KD);
        all_new_parameters.emplace_back("LA", data.LA);
        all_new_parameters.emplace_back("LB", data.LB);
        all_new_parameters.emplace_back("MPE", data.MPE);
        all_new_parameters.emplace_back("MPC", data.MPC);
        all_new_parameters.emplace_back("KMTT", data.KMTT);
        all_new_parameters.emplace_back("IMU_Z", data.IMU_Z);
        // 使用 set_parameters 来批量设置参数
        auto result = this->set_parameters(all_new_parameters);
        for (const auto &res : result)
        {
            if (!res.successful)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to set parameter: %s", res.reason.c_str());
                return;
            }
        }

    }catch (const std::exception &e){
        RCLCPP_ERROR(this->get_logger(), "Failed to set parameter: %s", e.what());
        return;
    }


    }

    void timer_callback()
    {
        if (currentSerial != nullptr)
        {
            // 发布里程计
            if (odom_enable)
            OdometerDataPublish(currentFictionData->odomData);
            // 发布IMU
            if (imu_enable)
            ImuDataPublish(currentFictionData->imuStructural);
            // 发布 RFID
            auto r = std_msgs::msg::String();
            r.data = currentFictionData->rfidData;
            rfid_publisher_->publish(r);
            // 发布磁条
            auto m = std_msgs::msg::String();
            m.data = currentFictionData->magneticData;
            magnetic_publisher_->publish(m);

            // 发布超声
            auto u = std_msgs::msg::Float32();
            u.data = currentFictionData->ultrasonic;
            ultrasonic_publisher_->publish(u);

            // 发布电池
            batteryPublish(currentFictionData->powerData);

            // 发布参数
            if(hisParamsData.IMU_Z ==0 && hisParamsData.KP ==0 && hisParamsData.KD == 0 && hisParamsData.KI == 0 && hisParamsData.LB == 0 && hisParamsData.LA == 0 && hisParamsData.MPE == 0.0 && hisParamsData.MPC == 0 && hisParamsData.KMTT == 0 ){
                try{

                    umsSerialMethodsPtr->sendMessageToGetParamData();
                    RCLCPP_INFO(this->get_logger(), "等待参数初始化....");
                }catch (const std::exception &e){
                    RCLCPP_ERROR(this->get_logger(), "Failed to init parameter: %s", e.what());
                }
            }
            if(!currentFictionData->paramsData.sysStatusFrame){
                if(hisParamsData != currentFictionData->paramsData){
                    hisParamsData = currentFictionData->paramsData;
                    publishParams(hisParamsData);
                }
            }
            //发布系统状态
            if(currentFictionData->paramsData.sysStatusFrame){
                auto sysStatus = std_msgs::msg::Int8();
                sysStatus.data =  static_cast<int>(currentFictionData->paramsData.sysStatusData);
                //APT尝试恢复

                if(currentFictionData->paramsData.sysStatusData == SysStatus::SYS_EMG_APT){
                    RCLCPP_ERROR(this->get_logger(), "下位APT错误,尝试恢复");
                    umsSerialMethodsPtr->refuseController();
                }
                sysStatus_publisher_->publish(sysStatus);
            }


            // 发布温度
            auto temperature = std_msgs::msg::Float32();
            temperature.data = currentFictionData->temperature;
            temperature_publisher_->publish(temperature);
        }
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // 在这里处理接收到的cmd_vel消息
        idle_time_ = std::make_shared<rclcpp::Time>(this->now());
        auto twistA = std::make_shared<TwistCustom>();
        twistA->angular_z = msg->angular.z;
        twistA->linear_x = msg->linear.x;
        twistA->linear_y = msg->linear.y;
        umsSerialMethodsPtr->sendTwistData(twistA);
    }

    void paramWriteByYaml(){
        float KP, MPC, KD, LB, MPE, IMU_Z, LA, KI;
        int KMTT;
        std::string readOrWrite;

        this->get_parameter("KP", KP);
        this->get_parameter("KI", KI);
        this->get_parameter("KD", KD);
        this->get_parameter("MPE", MPE);
        this->get_parameter("MPC", MPC);
        this->get_parameter("KMTT", KMTT);
        this->get_parameter("LB", LB);
        this->get_parameter("LA", LA);
        this->get_parameter("IMU_Z", IMU_Z);
        this->get_parameter("LC_read_write", readOrWrite);


        if(readOrWrite == "read"){
            RCLCPP_INFO(this->get_logger(), "read");
            RCLCPP_INFO(this->get_logger(), "KP：%0.2f",KP);
            RCLCPP_INFO(this->get_logger(), "KI：%0.2f",KI);
            RCLCPP_INFO(this->get_logger(), "KD：%0.2f",KD);
            RCLCPP_INFO(this->get_logger(), "MPE：%0.2f",MPE);
            RCLCPP_INFO(this->get_logger(), "MPC：%0.2f",MPC);
            RCLCPP_INFO(this->get_logger(), "KMTT：%d",KMTT);
            RCLCPP_INFO(this->get_logger(), "LB：%0.2f",LB);
            RCLCPP_INFO(this->get_logger(), "LA：%0.2f",LA);
            RCLCPP_INFO(this->get_logger(), "IMU_Z：%0.2f",IMU_Z);
            umsSerialMethodsPtr->sendMessageToGetParamData();
        } else if(readOrWrite == "write"){
            RCLCPP_INFO(this->get_logger(), "write");
            RCLCPP_INFO(this->get_logger(), "KP：%0.2f",KP);
            RCLCPP_INFO(this->get_logger(), "KI：%0.2f",KI);
            RCLCPP_INFO(this->get_logger(), "KD：%0.2f",KD);
            RCLCPP_INFO(this->get_logger(), "MPE：%0.2f",MPE);
            RCLCPP_INFO(this->get_logger(), "MPC：%0.2f",MPC);
            RCLCPP_INFO(this->get_logger(), "KMTT：%d",KMTT);
            RCLCPP_INFO(this->get_logger(), "LB：%0.2f",LB);
            RCLCPP_INFO(this->get_logger(), "LA：%0.2f",LA);
            RCLCPP_INFO(this->get_logger(), "IMU_Z：%0.2f",IMU_Z);

            ParamsData paramsData;

            paramsData.KP = KP;
            paramsData.KI = KI;
            paramsData.KD = KD;
            paramsData.MPE = MPE;
            paramsData.KMTT = KMTT;
            paramsData.MPC = MPC;
            paramsData.IMU_Z = IMU_Z;

            umsSerialMethodsPtr->setParamsData(paramsData);
            umsSerialMethodsPtr->sendEditParamsData();
        }
    }

    void batteryPublish(const PowerInfo &data)
    {
        auto message = sensor_msgs::msg::BatteryState();
        message.header.frame_id = "battery";
        message.header.stamp = this->now();
        message.voltage = data.input; // 示例电压值
        message.current = data.bus;   // 示例电流值
        bool isCharging = batteryMonitor.updateVoltage(data.input);
        message.charge = isCharging;
        message.percentage = batteryMonitor.calculateBatteryPercentage() / 100; // 电池剩余百分比
        battery_publisher_->publish(message);
    }

    void twistIdle(){
        if(this->now().nanoseconds() - idle_time_->nanoseconds() > 100000000){
            auto twistA = std::make_shared<TwistCustom>();
            twistA->angular_z = 0;
            twistA->linear_x = 0;
            twistA->linear_y = 0;
            umsSerialMethodsPtr->sendTwistData(twistA);
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ultrasonic_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr temperature_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rfid_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr magnetic_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_subscriber_;

    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr sysStatus_publisher_;

    std::shared_ptr<UmsSerialMethods> umsSerialMethodsPtr = std::make_shared<UmsSerialMethods>();
    BatteryMonitor batteryMonitor = BatteryMonitor(12.6, 5.0);
    rclcpp::TimerBase::SharedPtr timer_; // 定时器

    std::shared_ptr<rclcpp::Time> idle_time_ = nullptr;
    rclcpp::TimerBase::SharedPtr idle_timer_; // 定时器



    std::shared_ptr<rclcpp::Time> last_time_ = nullptr;
    ParamsData hisParamsData{};
    double x_;
    double y_;
    double theta_;


};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UMSFictionROS2>());
    rclcpp::shutdown();
    return 0;
}
