/**
 * @file dvc_dmmotor.h
 * @author cjw by yssickjgd
 * @brief 达妙电机配置与操作
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

#ifndef DVC_DMMOTOR_H
#define DVC_DMMOTOR_H

/* Includes ------------------------------------------------------------------*/

#include "drv_math.h"
#include "drv_can.h"
#include "alg_pid.h"
#include "main.h"
#include <cmath>
/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 电机PID极性
 */
enum Enum_PID_Polar {
    PID_Polar_Positive = 0,
    PID_Polar_Negative,
};


/**
 * @brief 达妙电机状态
 *
 */
enum Enum_DM_Motor_Status
{
    DM_Motor_Status_DISABLE = 0,
    DM_Motor_Status_ENABLE,
};

/**
 * @brief 达妙电机的ID枚举类型
 *
 */
enum Enum_DM_Motor_ID : uint8_t
{
    DM_Motor_ID_UNDEFINED = 0,
    DM_Motor_ID_0xA1,
    DM_Motor_ID_0xA2,
    DM_Motor_ID_0xA3,
    DM_Motor_ID_0xA4,
    DM_Motor_ID_0xA5,
    DM_Motor_ID_0xA6,
    DM_Motor_ID_0xA7,
    DM_Motor_ID_0xA8,
};

enum Enum_DM_Motor_ErrorCode : uint8_t
{
    ERROR_OverVoltage = 8,
    ERROR_UnderVoltage,
    ERROR_OverCurrent,
    ERROR_MOS_OverTemp,
    ERROR_Motor_OverTemp,
    ERROR_LossCAN,
    ERROR_OverLoad,
};


/**
 * @brief 达妙电机控制状态
 *
 */
enum Enum_DM_Motor_Control_Status
{
    DM_Motor_Control_Status_DISABLE = 0,
    DM_Motor_Control_Status_ENABLE,
};

/**
 * @brief 达妙电机控制方式
 *
 */
enum Enum_DM_Motor_Control_Method
{
    DM_Motor_Control_Method_MIT_POSITION = 0,
    DM_Motor_Control_Method_MIT_OMEGA,
    DM_Motor_Control_Method_MIT_TORQUE,
    DM_Motor_Control_Method_POSITION_OMEGA,
    DM_Motor_Control_Method_OMEGA,
	DM_Motor_Control_Method_MIT_IMU_Angle,
    DM_Motor_Control_Method_ONE_TO_FOUR,//达妙电机一拖四模式
    DM_Motor_Control_Method_MIT_OPENLOOP,
    DM_Motor_Control_Method_MIT_Angle, //关节电机角度控制模式
    DM_Motor_Control_Method_MIT_Encoder_Position // 电机编码器位置控制模式
};

/**
 * @brief 达妙电机源数据
 *
 */
struct Struct_DM_Motor_CAN_Rx_Data
{
    Enum_DM_Motor_ID CAN_ID;
    uint16_t Position_Reverse;
    uint8_t Omega_11_4;
    uint8_t Omega_3_0_Torque_11_8;
    uint8_t Torque_7_0;
    uint8_t MOS_Temperature;
    uint8_t Rotor_Temperature;
} __attribute__((packed));

/**
 * @brief 达妙电机经过处理的数据, 扭矩非国际单位制
 *
 */
struct Struct_DM_Motor_Rx_Data
{
    Enum_DM_Motor_ID CAN_ID;
    Enum_DM_Motor_ErrorCode ErrorCode;
    float Now_Angle;
    float Now_Radian;
    float Now_Angle_Deg;
    float Now_Angle_Rad;
    float Now_Omega_Angle;
    float Now_Omega_Radian;
    float Now_Torque;
    float Now_MOS_Temperature; //驱动MOS的平均温度
    float Now_Rotor_Temperature; // 电机内部线圈的平均温度
    uint16_t Now_Encoder_Position;
    uint16_t Pre_Position;
    int32_t Total_Position;
    int32_t Pre_Total_Position;
    int32_t Total_Round;
    uint8_t Now_Motor_Coil_Temperature; //电机线圈温度
    uint8_t Now_PCB_Temperature; //达妙板子上的PCB板温度
};

/**
 * @brief J4310无刷电机, 单片机控制输出控制帧
 * DM_Motor_Control_Method_POSITION_OMEGA模式下, 需调参助手辅助设置位置环PI参数, 空载250与0
 * 
 * PMAX值需在调参助手设置为3.141593, 即PI, 此时可在MIT模式下当舵机使用
 *
 */
class Class_DM_Motor_J4310
{
public:
    // PID角度环控制
    Class_PID PID_Angle;
    // PID角速度环控制
    Class_PID PID_Omega;
    // PID扭矩环控制
    Class_PID PID_Torque;

    void Init(FDCAN_HandleTypeDef *hcan, Enum_DM_Motor_ID __CAN_ID, Enum_DM_Motor_Control_Method __Control_Method = DM_Motor_Control_Method_MIT_POSITION, int32_t __Position_Offset = 0, float __Positon_Max = 3.141593f, float __Omega_Max = 20.94359f, float __Torque_Max = 10.0f);

    inline Enum_DM_Motor_Control_Status Get_DM_Motor_Control_Status();
    //反馈序号与驱动器状态，用于校准时排除陈旧/故障反馈。
    uint32_t Get_Feedback_Count() const { return Flag; }
    bool Get_Feedback_Enabled() const { return Data.ErrorCode == 1; }

    inline Enum_DM_Motor_Status Get_DM_Motor_Status();
    inline float Get_Now_Angle();
    inline float Get_Now_Radian();
    inline float Get_Now_Angle_Deg();
    inline float Get_Now_Angle_Rad();
    inline uint16_t Get_Now_Encoder_Position();
    inline float Get_Now_Omega();
    inline float Get_Now_Torque();
    inline float Get_Now_MOS_Temperature();
    inline float Get_Now_Rotor_Temperature();
    inline Enum_DM_Motor_Control_Method Get_Control_Method();
    inline float Get_MIT_K_P();
    inline float Get_MIT_K_D();
    inline float Get_Target_Angle();
    inline float Get_Target_Angle_Deg();
    inline float Get_Target_Angle_Rad();
    inline float Get_Target_Encoder_Position();
    inline float Get_Target_Omega();
    inline float Get_Target_Torque();
    inline float Get_FeedForward_Torque();
    inline float Get_Out();
    
    inline void Set_DM_Control_Status(Enum_DM_Motor_Control_Status __DM_Motor_Control_Status);
    inline void Set_DM_Motor_Control_Method(Enum_DM_Motor_Control_Method __DM_Motor_Control_Method);
    inline void Set_MIT_K_P(float __MIT_K_P);
    inline void Set_MIT_K_D(float __MIT_K_D);
    inline void Set_Target_Angle(float __Target_Angle);
    inline void Set_Target_Omega(float __Target_Omega);
    inline void Set_Target_Angle_DEG(float __Target_Angle_DEG);
    inline void Set_Target_Encoder_Position(float __Target_Encoder_Position);
    inline void Set_Target_Omega_DEG(float __Target_Omega_DEG);
    inline void Set_Target_Torque(float __Target_Torque);
    inline void Set_FeedForward_Torque(float __FeedForward_Torque);
    //J4310/J4340外部MIT_Angle闭环专用：电机输出轴N*m转内部输出刻度。
    inline bool Set_MIT_Angle_FeedForward_Torque_Nm(float torque_nm);
    inline void Set_Out(float __Out);
	inline void Limit_Out();

    inline void Set_Target_Real_Torque(float __Target_Real_Torque);

    inline void Set_PID_Polar(Enum_PID_Polar __PID_Polar);
    inline Enum_PID_Polar Get_PID_Polar();
    void Set_Gear_Ratio(float gear_ratio);

    void CAN_RxCpltCallback(uint8_t *Rx_Data);
    void TIM_Alive_PeriodElapsedCallback();
    void TIM_Process_PeriodElapsedCallback();
    void TIM_PID_PeriodElapsedCallback();
    void TIM_Send_PeriodElapsedCallback();
    void Clear_Error();

protected:
    //初始化相关变量

    //绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    //收数据绑定的CAN ID, 控制帧是0xxa1~0xxaf
    Enum_DM_Motor_ID CAN_ID;
    //发送缓存区
    uint8_t *CAN_Tx_Data;
    //位置反馈偏移
    uint32_t Position_Offset;
    //位置反馈最大值, 调参助手设置, 即为P_MAX，一般单圈设置为3.141593, 也就是360度
    float Position_Max;
    //最大速度, 调参助手设置, 推荐20.94359, 也就是最大转速200rpm
    float Omega_Max;
    //最大扭矩, 调参助手设置, 推荐7, 也就是最大输出7NM
    float Torque_Max;
    //输出量
    float Out=0.0f;
    //常量
    
    //一圈位置刻度
    // uint32_t Position_Max = 65536;

    //内部变量

    //最大输出扭矩
    uint16_t Output_Max = 4095;//12位的最大值除以2，最大输出10N·M,pitch是1.5N*M
    //当前时刻的电机接收flag
    uint32_t Flag = 0;
    //前一时刻的电机接收flag
    uint32_t Pre_Flag = 0;


    //读变量

    //电机状态
    Enum_DM_Motor_Status DM_Motor_Status = DM_Motor_Status_DISABLE;
    //电机对外接口信息
    Struct_DM_Motor_Rx_Data Data;

    Enum_PID_Polar PID_Polar = PID_Polar_Positive;

    //写变量

    //读写变量

    //电机控制状态
    Enum_DM_Motor_Control_Status DM_Motor_Control_Status = DM_Motor_Control_Status_DISABLE;
    //电机控制方式
    Enum_DM_Motor_Control_Method DM_Motor_Control_Method = DM_Motor_Control_Method_MIT_POSITION;
    //MIT的Kp值, 0~500, 空载6, 位置控制需要
    float MIT_K_P = 0.0f;
    //MIT的Kd值, 0~5, 空载0.2, 位置和速度控制需要
    float MIT_K_D = 0.0f;
    //目标的角度, rad
    float Target_Angle = 0.0f;
    float Target_Angle_Rad = 0.0f;
    float Target_Encoder_Position = 0.0f;
    //目标的角度, °
    float Target_Angle_Deg = 0.0f;
    //目标的速度, °/s
    float Target_Omega_DEG = 0.0f;
    //目标的速度, rad/s
    float Target_Omega = 0.0f;
    //目标的扭矩
    float Target_Torque = 0.0f;

    float FeedForward_Torque = 0.0f; //MIT_Angle下为内部输出刻度，与PID_Omega输出同单位
    // 当前仅记录减速比，不参与控制量或反馈量换算。
    float Gear_Ratio = 1.0f;

    uint16_t CAN_ID_Offset = 0x000;

    //内部函数

    void Data_Process(uint8_t* Rx_Data);
    void Output();
};

/**
 * @brief DM-S2325-1EC（DM3520-1EC 驱动器）的 CAN 控制。
 *
 * Init 的 PMAX、VMAX、TMAX 必须与驱动器参数一致。反馈位置先按电机轴
 * 单圈回绕累计，再除以 25 得到输出轴多圈角度；反馈角速度也除以 25。
 * 继承的 Get_Now_Angle_Deg/Rad 返回输出轴多圈角度，扭矩反馈保持原始解码。
 * 按实测结果，发送目标角度时不乘电机内置的 25:1 减速比。
 * CAN_ID 是驱动器配置的电机 ID，反馈帧 ID 应由任务层按 Master ID 分发。
 */
class Class_DM_Motor_S2325 : public Class_DM_Motor_J4310
{
public:
    void Init(FDCAN_HandleTypeDef *hcan, Enum_DM_Motor_ID id,
              Enum_DM_Motor_Control_Method method, int32_t position_offset,
              float position_max, float omega_max, float torque_max);

    void Set_Target_Angle(float target_angle_rad);
    void Set_Target_Angle_DEG(float target_angle_deg);
    float Get_Target_Angle_Rad();
    float Get_Target_Angle_Deg();

    void CAN_RxCpltCallback(uint8_t *rx_data);
    void TIM_Alive_PeriodElapsedCallback();
    void TIM_Process_PeriodElapsedCallback();
    void TIM_PID_PeriodElapsedCallback();
    void TIM_Send_PeriodElapsedCallback();
    void Clear_Error();

private:
    bool Initialized = false;
    bool Has_Position_Feedback = false;
    uint16_t Previous_Encoder_Position = 0;
    uint8_t Tx_Data[8] = {};
    void Pack_MIT_Frame(float position, float velocity, float kp, float kd, float torque);
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取电机状态
 *
 * @return Enum_DM_Motor_Status 电机状态
 */
Enum_DM_Motor_Status Class_DM_Motor_J4310::Get_DM_Motor_Status()
{
    return (DM_Motor_Status);
}

/**
 * @brief 获取当前的角度, rad
 *
 * @return float 当前的角度, rad
 */
float Class_DM_Motor_J4310::Get_Now_Angle()
{
    return (Data.Now_Angle);
}


/**
 * @brief 获取当前的角度, rad
 *
 * @return float 当前的角度, rad
 */
float Class_DM_Motor_J4310::Get_Now_Radian()
{
    return (Data.Now_Radian);
}

float Class_DM_Motor_J4310::Get_Now_Angle_Deg()
{
    return (Data.Now_Angle_Deg);
}

float Class_DM_Motor_J4310::Get_Now_Angle_Rad()
{
    return (Data.Now_Angle_Rad);
}

/**
 * @brief 获取当前的编码器位置
 * 
 * @return float 当前的编码器位置
 */
uint16_t Class_DM_Motor_J4310::Get_Now_Encoder_Position()
{
    return (Data.Now_Encoder_Position);
}

/**
 * @brief 获取当前的速度, rad/s
 *
 * @return float 当前的速度, rad/s
 */
float Class_DM_Motor_J4310::Get_Now_Omega()
{
    return (Data.Now_Omega_Radian);
}

/**
 * @brief 获取当前的扭矩, 直接采用反馈值
 *
 * @return float 当前的扭矩, 直接采用反馈值
 */
float Class_DM_Motor_J4310::Get_Now_Torque()
{
    return (Data.Now_Torque);
}

/**
 * @brief 获取当前MOS管的温度, 开氏度
 *
 * @return float 当前MOS管的温度, 开氏度
 */
float Class_DM_Motor_J4310::Get_Now_MOS_Temperature()
{
    return (Data.Now_MOS_Temperature);
}

/**
 * @brief 获取当前绕组的温度, 开氏度
 *
 * @return float 当前绕组的温度, 开氏度
 */
float Class_DM_Motor_J4310::Get_Now_Rotor_Temperature()
{
    return (Data.Now_Rotor_Temperature);
}

/**
 * @brief 获取电机控制方式
 *
 * @return Enum_DM_Motor_Control_Method 电机控制方式
 */
Enum_DM_Motor_Control_Method Class_DM_Motor_J4310::Get_Control_Method()
{
    return (DM_Motor_Control_Method);
}

/**
 * @brief 获取MIT的Kp值, 0~500
 *
 * @return float MIT的Kp值, 0~500
 */
float Class_DM_Motor_J4310::Get_MIT_K_P()
{
    return (MIT_K_P);
}

/**
 * @brief 获取MIT的Kd值, 0~5
 *
 * @return float MIT的Kd值, 0~5
 */
float Class_DM_Motor_J4310::Get_MIT_K_D()
{
    return (MIT_K_D);
}

/**
 * @brief 获取目标的角度, rad
 *
 * @return float 目标的角度, rad
 */
float Class_DM_Motor_J4310::Get_Target_Angle()
{
    return (Target_Angle);
}

/**
 * @brief 获取目标的速度, rad/s
 *
 * @return float 目标的速度, rad/s
 */
float Class_DM_Motor_J4310::Get_Target_Omega()
{
    return (Target_Omega);
}

/**
 * @brief 获取目标的扭矩
 *
 * @return float 目标的扭矩
 */
float Class_DM_Motor_J4310::Get_Target_Torque()
{
    return (Target_Torque);
}

/**
 * @brief 获取前馈扭矩
 *
 * @return float 前馈扭矩
 */
float Class_DM_Motor_J4310::Get_FeedForward_Torque()
{
    return (FeedForward_Torque);
}

/**
 * @brief 获取目标的输出
 *
 * @return float 目标的输出
 */
float Class_DM_Motor_J4310::Get_Out()
{
    return (Out);
}

/**
 * @brief 设定电机控制状态
 *
 * @param __DM_Motor_Control_Status 电机控制状态
 */
void Class_DM_Motor_J4310::Set_DM_Control_Status(Enum_DM_Motor_Control_Status __DM_Motor_Control_Status)
{
    DM_Motor_Control_Status = __DM_Motor_Control_Status;
}

/**
 * @brief 设定电机控制方式
 *
 * @param __Control_Method 电机控制方式
 */
void Class_DM_Motor_J4310::Set_DM_Motor_Control_Method(Enum_DM_Motor_Control_Method __Control_Method)
{
    DM_Motor_Control_Method = __Control_Method;

    switch (DM_Motor_Control_Method) {
        case DM_Motor_Control_Method_MIT_POSITION:
        case DM_Motor_Control_Method_MIT_Angle:
        case DM_Motor_Control_Method_MIT_Encoder_Position:
        case DM_Motor_Control_Method_MIT_IMU_Angle:
        case DM_Motor_Control_Method_MIT_OMEGA:
        case DM_Motor_Control_Method_MIT_OPENLOOP:
        case DM_Motor_Control_Method_MIT_TORQUE: {
            CAN_ID_Offset = 0x000;
            break;
        }
        case DM_Motor_Control_Method_POSITION_OMEGA: {
            CAN_ID_Offset = 0x100;
            break;
        }
        case DM_Motor_Control_Method_OMEGA: {
            CAN_ID_Offset = 0x200;
            break;
        }
        case DM_Motor_Control_Method_ONE_TO_FOUR:{
            CAN_ID_Offset = 0x300;
            break;
        }
    }
}

/**
 * @brief 设定MIT的Kp值, 0~500, 空载6, 位置控制需要
 *
 * @param __MIT_K_P MIT的Kp值, 0~500, 空载6, 位置控制需要
 */
void Class_DM_Motor_J4310::Set_MIT_K_P(float __MIT_K_P)
{
    MIT_K_P = __MIT_K_P;
}

/**
 * @brief 设定MIT的Kd值, 0~5, 空载0.2, 位置和速度控制需要
 *
 * @param __MIT_K_D MIT的Kd值, 0~5, 空载0.2, 位置和速度控制需要
 */
void Class_DM_Motor_J4310::Set_MIT_K_D(float __MIT_K_D)
{
    MIT_K_D = __MIT_K_D;
}

/**
 * @brief 设定目标的角度, rad
 *
 * @param __Target_Angle 目标的角度, rad
 */
void Class_DM_Motor_J4310::Set_Target_Angle(float __Target_Angle)
{
    Target_Angle = __Target_Angle;
}

/**
 * @brief 设定目标的角度, °
 *
 * @param __Target_Angle 目标的角度, °
 */
void Class_DM_Motor_J4310::Set_Target_Angle_DEG(float __Target_Angle_Deg)
{
    Target_Angle_Deg = __Target_Angle_Deg;
}

/**
 * @brief 设定目标的速度, rad/s
 *
 * @param __Target_Omega 目标的速度, rad/s
 */
void Class_DM_Motor_J4310::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

/**
 * @brief 设定目标的速度, °/s
 *
 * @param __Target_Omega 目标的速度, °/s
 */
void Class_DM_Motor_J4310::Set_Target_Omega_DEG(float __Target_Omega_DEG)
{
    Target_Omega_DEG = __Target_Omega_DEG;
}

/**
 * @brief 设定目标的扭矩
 *
 * @param __Target_Torque 目标的扭矩
 */
void Class_DM_Motor_J4310::Set_Target_Torque(float __Target_Torque)
{
    Target_Torque = __Target_Torque;
}

/**
 * @brief 设定前馈扭矩
 *
 * @param __FeedForward_Torque 前馈扭矩
 */
void Class_DM_Motor_J4310::Set_FeedForward_Torque(float __FeedForward_Torque)
{
    FeedForward_Torque = __FeedForward_Torque;
}

/**
 * @brief 电机输出轴物理力矩转换为外部角度闭环的前馈刻度
 * @note TMAX必须与驱动器设置一致。此接口仅用于J4310/J4340的MIT_Angle，
 *       不用于S2325（其FeedForward_Torque本身已使用N*m）。
 */
bool Class_DM_Motor_J4310::Set_MIT_Angle_FeedForward_Torque_Nm(float torque_nm)
{
    FeedForward_Torque = 0.0f;
    if (DM_Motor_Control_Method != DM_Motor_Control_Method_MIT_Angle ||
        !std::isfinite(torque_nm) || !std::isfinite(Torque_Max) ||
        Torque_Max <= 0.0f || Output_Max == 0)
    {
        return false;
    }

    if (torque_nm > Torque_Max)
        torque_nm = Torque_Max;
    else if (torque_nm < -Torque_Max)
        torque_nm = -Torque_Max;

    const float output = (torque_nm / Torque_Max) * static_cast<float>(Output_Max);
    //PID回调在相加后统一乘PID_Polar，这里抵消其对物理前馈方向的影响。
    FeedForward_Torque = (PID_Polar == PID_Polar_Positive) ? output : -output;
    return true;
}

/**
 * @brief 设定目标的输出
 *
 * @param __Out 目标的输出
 */

void Class_DM_Motor_J4310::Set_Out(float __Out)
{
    // 防止无效浮点数被编码成大力矩 CAN 指令。
    Out = std::isfinite(__Out) ? __Out : 0.0f;
} 

/**
 * @brief 限制目标的输出
 *
 * @param __Out 目标的输出
 */
void Class_DM_Motor_J4310::Limit_Out()
{
	if(Out>Output_Max)
		Out=Output_Max;
	else if(Out<-Output_Max)
		Out=-Output_Max;
	
	else
	{
		
	}
}

Enum_DM_Motor_Control_Status Class_DM_Motor_J4310::Get_DM_Motor_Control_Status()
{
    return DM_Motor_Control_Status;
}

inline void Class_DM_Motor_J4310::Set_PID_Polar(Enum_PID_Polar __PID_Polar) {
    PID_Polar = __PID_Polar;
}

inline Enum_PID_Polar Class_DM_Motor_J4310::Get_PID_Polar() {
    return PID_Polar;
}

inline void Class_DM_Motor_J4310::Set_Target_Real_Torque(float __Target_Real_Torque) {
    Target_Torque = __Target_Real_Torque;
}



#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
