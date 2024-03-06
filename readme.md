*声明参数*
this->declare_parameter<std::string>("LC_read_write", "read");                                          //读或写
this->declare_parameter<int32_t>("LC_Motion_control_instruction", 0);                                   //系统控制指令
this->declare_parameter<int32_t>("LC_System_operating_status", 0);                                      //系统运行状态
this->declare_parameter<float>("LC_Speed_closed_loop_controller_proportional_gain", 0);                 //速度闭环控制器 比例增益
this->declare_parameter<float>("LC_Speed_closed_loop_controller_integral_gain", 0);                     //速度闭环控制器 积分增益
this->declare_parameter<float>("LC_Speed_closed_loop_controller_differential_gain", 0);                 //速度闭环控制器 微分增益
this->declare_parameter<float>("LC_Speed_measurement_pulse_cycle_ratio", 0);                            //速度测算 脉冲周数比
this->declare_parameter<float>("LC_Speed_measures_the_circumference_of_the_wheel", 0);                  //速度测算 轮圆周长 单位：m
this->declare_parameter<float>("LC_Chassis_dimensions_Wheel_spacing_/2", 0);                            //底盘尺寸 轮间距/2 单位：m
this->declare_parameter<float>("LC_Chassis_dimensions_Axle_spacing_/2", 0);                             //底盘尺寸 轴间距/2 单位：m
this->declare_parameter<int32_t>("LC_Kinematic_model_type", 0);                                         //运动学模型类型
this->declare_parameter<float>("LC_IMU_Z-axis_course_Angle_zero_offset_correction_bias_value", 0.0);      //IMU Z 轴 航向角零偏修正偏置值