/** @file dvc_joint_motor_dm3520.h
 * @brief DM3520 驱动器 / S2325：STM32 输出轴串级 PID，达妙 MIT 扭矩报文。
 */
#ifndef DVC_JOINT_MOTOR_DM3520_H
#define DVC_JOINT_MOTOR_DM3520_H

#include "dvc_dmmotor.h"
#include "dvc_joint_motor.h"

class Class_Joint_Motor_DM3520 : public Class_DM_Motor_J4310, public Class_Joint_Motor
{
public:
    // PMAX/VMAX/TMAX 均为减速前协议量程，必须与驱动器配置一致。
    void Init(FDCAN_HandleTypeDef *hcan, Enum_DM_Motor_ID id,
              Enum_DM_Motor_Control_Method method, int32_t offset,
              float position_max, float omega_max, float torque_max);
    bool Set_Gear_Ratio(float ratio);
    float Get_Gear_Ratio() const { return Gear_Ratio; }
    // 转子侧连续角度 / 速度 / 扭矩。Data 为输出轴：角度速度除以减速比，扭矩乘以减速比。
    float Get_Rotor_Radian() const { return Rotor_Radian; }
    float Get_Rotor_Omega() const { return Rotor_Omega; }
    float Get_Rotor_Torque() const { return Rotor_Torque; }
    float Get_Rotor_Wrapped_Radian() const { return Rotor_Wrapped_Radian; }
    float Get_Now_Radian() const override { return Data.Now_Angle_Rad; }
    float Get_Now_Angle() const { return Data.Now_Angle_Deg; }
    bool Get_Online() const override { return DM_Motor_Status == DM_Motor_Status_ENABLE; }
    void Set_Target_Radian(float radian) override { Set_Target_Angle(radian); }
    void Set_Enable(bool enable) override;
    // 输出轴指令扭矩 Nm（按理想减速比换算，无额外输出轴限幅）。
    float Get_Command_Torque() const { return Command_Torque; }
    void CAN_RxCpltCallback(uint8_t *data);
    void TIM_PID_PeriodElapsedCallback();
    void TIM_Send_PeriodElapsedCallback();
    void TIM_Alive_PeriodElapsedCallback();
    void TIM_Calculate_PeriodElapsedCallback() override;
    void Clear_Error();

private:
    float Gear_Ratio = 25.0f;
    float Rotor_Radian = 0.0f;
    float Rotor_Omega = 0.0f;
    float Rotor_Torque = 0.0f;
    float Rotor_Wrapped_Radian = 0.0f;
    bool Have_Position = false;
    int64_t Position_Wraps = 0; // 协议区间回绕次数，不是机械圈数
    // 回绕周期为 2*PMAX；相邻有效帧转角必须小于 PMAX。断电后须重新 Init。
    void Update_Joint_Feedback();
    float Command_Torque = 0.0f;
    bool Initialized = false;
    uint8_t Tx_Data[8] = {};
    void Reset_Control();
    void Pack_Torque();
    void Send_Command(uint8_t command);
};
#endif
