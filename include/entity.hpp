#include <string>

// 将一组十六进制数组合成一个int32类型
enum ControlStatus
{
    EMERGENCY_STOP,  // 急停状态
    PROGRAM_CONTROL, // 程序控制状态
    REMOTE_CONTROL   // 遥控控制状态
};

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

struct OdomInfo
{
    /* data */
    double delta_x;
    double delta_y;
    double delta_th;
};

struct PowerInfo{
    double bus;
    double output5v;
    double input;
    double output19v;

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

struct ICDRemote {
    double az;
    double vx;
    double vy;

};

struct RCSBUSRemote {
    int len;
    uint16_t  axes [8];
    uint16_t buttons [8];
};



struct FictionData
{
    ImuInfo imuStructural;
    OdomInfo odomData;
    ParamsData paramsData;
    std::string magneticData;
    std::string rfidData;
    PowerInfo powerData;
    float ultrasonic;
    float temperature;
    ICDRemote icdData;
    RCSBUSRemote rcsBusData;
};

struct TwistCustom
{
    float linear_x;
    float linear_y;
    float angular_z;
};
