#ifndef ROBOT_ARM_CLION_RBT_JOINT_H
#define ROBOT_ARM_CLION_RBT_JOINT_H
#include "dvc_dmmotor.h"
#include "alg_fsm.h"
// 达妙关节的目标角度和电机角度换算。
class Class_Joint_DM_Motor
{
public:
    Class_DM_Motor_J4310 Joint_Motor_DM_J4340;
    //关节输出端重力矩，N*m；内部处理传动比例、方向和驱动输出刻度。
    bool Set_Joint_FeedForward_Torque_Nm(float torque_nm);
    // CAN 反馈回调更新的关节实际角度。
    float Now_Joint_Angle_Deg = 0.0f;
    float Now_Joint_Angle_Rad = 0.0f;

    void Init(FDCAN_HandleTypeDef *hcan, Enum_DM_Motor_ID can_id, float position_max, float omega_max, float torque_max);
    void Set_Joint_Target_Angle_Deg(float target_angle_deg);
    void Set_Joint_Target_Angle_Rad(float target_angle_rad);
    float Get_Joint_Target_Angle_Deg();
    float Get_Joint_Target_Angle_Rad();
    float Get_Joint_Now_Angle_Deg();
    float Get_Joint_Now_Angle_Rad();
    void Set_Joint_Angle_Gain(float joint_angle_gain);
    float Get_Joint_Angle_Gain();
    void Set_Joint_Angle_Bias_Deg(float joint_angle_bias_deg);
    void Set_Joint_Angle_Bias_Rad(float joint_angle_bias_rad);
    float Get_Joint_Angle_Bias_Deg();
    float Get_Joint_Angle_Bias_Rad();
    void Set_Max_Joint_Angle_Deg(float max_joint_angle_deg);
    void Set_Max_Joint_Angle_Rad(float max_joint_angle_rad);
    float Get_Max_Joint_Angle_Deg();
    float Get_Max_Joint_Angle_Rad();
    void Set_Min_Joint_Angle_Deg(float min_joint_angle_deg);
    void Set_Min_Joint_Angle_Rad(float min_joint_angle_rad);
    float Get_Min_Joint_Angle_Deg();
    float Get_Min_Joint_Angle_Rad();

    void Set_Real_Target_Torque(float real_torque_Nm);
    float Get_Real_Target_Torque();

protected:
    float Joint_Angle_Gain = 1.0f; //除了电机自带的减速箱外，由传动装置（如同步带）造成的角度增益，也用于调整由于电机安装方式造成的角度正方向与机械臂角度正方向相反
    float Joint_Angle_Bias_Deg = 0.0f;
    float Joint_Angle_Bias_Rad = 0.0f; //角度偏置，关节角度 = Joint_Angle_Gain * (电机角度 + 角度偏置)，用于设置某关节的零点位置；Set 函数会同步度和弧度偏置
    float Target_Joint_Angle_Rad = 0.0f;
    float Target_Joint_Angle_Deg = 0.0f;

    float Max_Joint_Angle_Deg = 180.0f;
    float Min_Joint_Angle_Deg = -180.0f;
    float Max_Joint_Angle_Rad = 3.14159265358979323846f;
    float Min_Joint_Angle_Rad = -3.14159265358979323846f;
};

enum Enum_Joint_DM_Gear_Calibration_Status
{
    Joint_DM_Gear_Calibration_WAIT = 0,
    Joint_DM_Gear_Calibration_ROTATE,
    Joint_DM_Gear_Calibration_STALL_DETECT,
    Joint_DM_Gear_Calibration_SET_ZERO,
    Joint_DM_Gear_Calibration_COMPLETE,
    Joint_DM_Gear_Calibration_OFFLINE,
    Joint_DM_Gear_Calibration_IDLE, //6：未触发，上电保持此状态
    Joint_DM_Gear_Calibration_STATUS_NUMBER,
};

class Class_Joint_DM_Gear;
class Class_FSM_Joint_DM_Gear_Calibration : public Class_FSM
{
public:
    Class_Joint_DM_Gear *Joint = nullptr;
    void Reload_TIM_Status_PeriodElapsedCallback();
};

class Class_Joint_DM_Gear
{
public:
    Class_FSM_Joint_DM_Gear_Calibration FSM_Calibration;
    friend class Class_FSM_Joint_DM_Gear_Calibration;

    //每1 ms调用；先运行校准FSM，再执行电机MIT控制。Init后保持IDLE，由Request_Recalibration()或调试触发进入WAIT。
    void TIM_Process_PeriodElapsedCallback(bool send_control = true);
    void Update_Joint_Feedback();
    //omega为MIT目标速度rad/s（符号决定方向，遵循现有驱动单位）；torque与Get_Now_Torque同单位。
    //参数直接写入；保留bool返回值以兼容现有调用，始终返回true。
    bool Set_Calibration_Parameter(float omega, float locked_torque,
                                   uint32_t locked_time_ms, float mit_kd = 1.0f);
    void Request_Recalibration();
    bool Get_Calibration_Complete();

    Class_DM_Motor_S2325 Joint_Motor_DM_S2325;
    float Now_Joint_Angle_Deg = 0.0f;
    float Now_Joint_Angle_Rad = 0.0f;

    void Init(FDCAN_HandleTypeDef *hcan, Enum_DM_Motor_ID can_id,
              float position_max, float omega_max, float torque_max);
    void Set_Joint_Target_Angle_Deg(float target_angle_deg);
    void Set_Joint_Target_Angle_Rad(float target_angle_rad);
    float Get_Joint_Target_Angle_Deg();
    float Get_Joint_Target_Angle_Rad();
    float Get_Joint_Now_Angle_Deg();
    float Get_Joint_Now_Angle_Rad();
    void Set_Joint_Angle_Gain(float joint_angle_gain);
    float Get_Joint_Angle_Gain();
    void Set_Joint_Angle_Bias_Deg(float joint_angle_bias_deg);
    void Set_Joint_Angle_Bias_Rad(float joint_angle_bias_rad);
    float Get_Joint_Angle_Bias_Deg();
    float Get_Joint_Angle_Bias_Rad();
    void Set_Max_Joint_Angle_Deg(float max_joint_angle_deg);
    void Set_Max_Joint_Angle_Rad(float max_joint_angle_rad);
    float Get_Max_Joint_Angle_Deg();
    float Get_Max_Joint_Angle_Rad();
    void Set_Min_Joint_Angle_Deg(float min_joint_angle_deg);
    void Set_Min_Joint_Angle_Rad(float min_joint_angle_rad);
    float Get_Min_Joint_Angle_Deg();
    float Get_Min_Joint_Angle_Rad();

protected:
    float Calibration_Omega = 0.2f;
    float Calibration_Locked_Torque = 0.1f;
    uint32_t Calibration_Locked_Time_Ms = 200;
    float Calibration_MIT_K_D = 1.0f;
    bool Calibration_Complete = false;
    bool Calibration_Feedback_Valid = false;
    uint32_t Calibration_Last_Feedback = 0;
    uint32_t Calibration_Feedback_Age = 50;
    bool Calibration_Control_Saved = false;
    Enum_DM_Motor_Control_Method Calibration_Previous_Method = DM_Motor_Control_Method_MIT_POSITION;
    float Calibration_Previous_K_P = 0.0f;
    float Calibration_Previous_K_D = 0.0f;
    bool Is_MIT_Control_Method();
    void Restore_Calibration_Control();
    void Calibration_Output();
    //本类偏移定义：q_joint = gain * (q_feedback - bias)，目标反算使用+bias。
    float Joint_Angle_Gain = 1.0f;
    float Joint_Angle_Bias_Deg = 0.0f;
    float Joint_Angle_Bias_Rad = 0.0f;
    float Target_Joint_Angle_Rad = 0.0f;
    float Target_Joint_Angle_Deg = 0.0f;

    float Max_Joint_Angle_Deg = 180.0f;
    float Min_Joint_Angle_Deg = -180.0f;
    float Max_Joint_Angle_Rad = 3.14159265358979323846f;
    float Min_Joint_Angle_Rad = -3.14159265358979323846f;
};


#endif // ROBOT_ARM_CLION_RBT_JOINT_H
