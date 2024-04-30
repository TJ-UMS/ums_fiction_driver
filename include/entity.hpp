#include <string>

// 将一组十六进制数组合成一个int32类型
enum ControlStatus
{
    EMERGENCY_STOP,  // 急停状态
    PROGRAM_CONTROL, // 程序控制状态
    REMOTE_CONTROL   // 遥控控制状态
};

enum class SysStatus : int{
    SYS_STANDBY,
    SYS_RUNNING_URT,
    SYS_RUNNING_JOY,
    SYS_EMG_PSB,
    SYS_EMG_APT
};

enum class AgreementVersion : int
{
    V1,
    V2
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
    bool sysStatusFrame; //状态帧
    SysStatus sysStatusData; //系统运作状态

    // Overload the equality operator (==)
    bool operator==(const ParamsData& other) const {
        return (KP == other.KP && KI == other.KI && KD == other.KD &&
                LA == other.LA && LB == other.LB && MPE == other.MPE &&
                MPC == other.MPC && KMTT == other.KMTT && IMU_Z == other.IMU_Z && sysStatusData == other.sysStatusData);
    }

    // Overload the inequality operator (!=)
    bool operator!=(const ParamsData& other) const {
        return !(*this == other);  // Leverage already defined == operator
    }
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
