//
// Created by anaple on 24-3-25.
//
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ums_serial_methods.hpp"
#include "battery.h"
#include "tf2/LinearMath/Quaternion.h"



class UMSFictionROS2 : public rclcpp::Node {
public:
    std::shared_ptr<serial::Serial> currentSerial = nullptr;
    std::shared_ptr<FictionData> currentFictionData = std::make_shared<FictionData>();
    UMSFictionROS2() : Node("ums_fiction_ros2") {

        /*声明参数*/
        std::string port;
        int baudrate;
        this->declare_parameter<int>("baudrate",921600);
        this->declare_parameter<std::string>("port","/dev/ttyUSB1");
        this->get_parameter<std::string>("port",port);          // 端口号
        this->get_parameter<int>("baudrate",baudrate);                // 波特率
        this->declare_parameter<std::string>("LC_read_write", "read"); // 读或写
        this->declare_parameter<float>("KP", 0);                       // 速度闭环控制器 比例增益
        this->declare_parameter<float>("KI", 0);                       // 速度闭环控制器 积分增益
        this->declare_parameter<float>("KD", 0);                       // 速度闭环控制器 微分增益
        this->declare_parameter<float>("MPE", 0);                      // 速度测算 脉冲周数比
        this->declare_parameter<float>("MPC", 0);                      // 速度测算 轮圆周长 单位：m
        this->declare_parameter<float>("LA", 0);                       // 底盘尺寸 轮间距/2 单位：m
        this->declare_parameter<float>("LB", 0);                       // 底盘尺寸 轴间距/2 单位：m
        this->declare_parameter<int32_t>("KMTT", 0);                   // 运动学模型类型
        this->declare_parameter<float>("IMU_Z", 0.0);                  // IMU Z 轴 航向角零偏修正偏置值
        this->declare_parameter<bool>("encodeRfid", false);

        umsSerialMethods.startSerial(port,baudrate);
        // 初始化发布器
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        ultrasonic_publisher_ = this->create_publisher<std_msgs::msg::Float32>("ultrasonic", 10);
        temperature_publisher_ = this->create_publisher<std_msgs::msg::Float32>("temperature", 10);
        battery_publisher_ = this->create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 10);
        rfid_publisher_ = this->create_publisher<std_msgs::msg::String>("rfid_data", 10);
        magnetic_publisher_ = this->create_publisher<std_msgs::msg::String>("magnetic_data", 10);
        // 初始化订阅器
        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel", 10,
                std::bind(&UMSFictionROS2::cmd_vel_callback, this, std::placeholders::_1));
        // 初始化定时器
        timer_ = this->create_wall_timer(
                std::chrono::milliseconds(18),  // 1000ms / 60Hz = 约16.67ms
                std::bind(&UMSFictionROS2::timer_callback, this));
        currentSerial = umsSerialMethods.getSerial();
        umsSerialMethods.sendGetParamData(currentSerial);
        umsSerialMethods.loopUmsFictionData(currentSerial, currentFictionData);
    }
    ~UMSFictionROS2() {
        currentFictionData.reset();
        currentSerial.reset();
    }


private:
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
        if(last_time_ == nullptr){

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
        odom.pose.pose.orientation.w= quat.w();
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;
        odom_publisher_->publish(odom);
        last_time_ = std::make_shared<rclcpp::Time>(current_time_);
    }


    void publishParams(const ParamsData& data) {
        std::vector<rclcpp::Parameter> all_new_parameters;
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
        for (const auto& res : result) {
            if (!res.successful) {
                RCLCPP_ERROR(this->get_logger(), "Failed to set parameter: %s", res.reason.c_str());
                return;
            }
        }
    }

    void timer_callback()
    {
        if(currentSerial  != nullptr){
            //发布里程计
            OdometerDataPublish(currentFictionData->odomData);
            //发布IMU
            ImuDataPublish(currentFictionData->imuStructural);
            //发布 RFID
            auto r = std_msgs::msg::String();
            r.data = currentFictionData->rfidData;
            rfid_publisher_->publish(r);
            //发布磁条
            auto m = std_msgs::msg::String();
            m.data = currentFictionData->magneticData;
            magnetic_publisher_->publish(m);

            //发布超声
            auto u = std_msgs::msg::Float32();
            u.data = currentFictionData->ultrasonic;
            ultrasonic_publisher_->publish(u);

            //发布电池
            batteryPublish(currentFictionData->powerData);

            //发布参数
            publishParams(currentFictionData->paramsData);

            //发布温度
            auto temperature = std_msgs::msg::Float32();
            temperature.data = currentFictionData->temperature;
            temperature_publisher_->publish(temperature);



        }
    }



    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // 在这里处理接收到的cmd_vel消息
        auto twistA = std::make_shared<TwistCustom>();
        twistA->angular_z = msg->angular.z;
        twistA->linear_x = msg->linear.x;
        twistA->linear_y = msg->linear.y;
        umsSerialMethods.sendTwistData(currentSerial,twistA);
    }

    void batteryPublish(const PowerInfo& data ){
        auto message = sensor_msgs::msg::BatteryState();
        message.header.frame_id = "battery";
        message.header.stamp = this->now();
        message.voltage = data.input;  // 示例电压值
        message.current = data.bus;   // 示例电流值
        bool isCharging = batteryMonitor.updateVoltage(data.input);
        message.charge = isCharging;
        message.percentage =  batteryMonitor.calculateBatteryPercentage()/100; // 电池剩余百分比
        battery_publisher_->publish(message);

    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ultrasonic_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr temperature_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rfid_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr magnetic_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;

    UmsSerialMethods umsSerialMethods = UmsSerialMethods();
    BatteryMonitor batteryMonitor = BatteryMonitor(12.6,5.0);
    rclcpp::TimerBase::SharedPtr timer_;  // 定时器

    std::shared_ptr<rclcpp::Time> last_time_ = nullptr;
    double x_;
    double y_;
    double theta_;

};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UMSFictionROS2>());
    rclcpp::shutdown();
    return 0;
}
