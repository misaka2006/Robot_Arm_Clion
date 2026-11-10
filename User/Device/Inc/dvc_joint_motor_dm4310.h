/** @file dvc_joint_motor_dm4310.h
 *  @brief DM4310 关节电机适配：直接继承设备驱动和关节接口。
 */
#ifndef DVC_JOINT_MOTOR_DM4310_H
#define DVC_JOINT_MOTOR_DM4310_H

#include "dvc_joint_motor.h"
#include "dvc_dmmotor.h"

class Class_Joint_Motor_DM4310 : public Class_Joint_Motor, public Class_DM_Motor_J4310
{
public:
    bool Init(FDCAN_HandleTypeDef *hcan, uint16_t motor_id, uint16_t master_id,
              float position_max, float omega_max, float torque_max);
    bool Get_Online() const override;
    float Get_Now_Radian() const override;
    void Set_Target_Radian(float radian) override;
    void Set_Enable(bool enable) override;
    void TIM_PID_PeriodElapsedCallback();
    void TIM_Calculate_PeriodElapsedCallback() override;

    // 按 MASTERID 分发，只接受标准数据帧、8 字节；不是按发送 CANID 分发。
    void CAN_RxCpltCallback(Struct_CAN_Rx_Buffer *message);
    uint16_t Get_Master_ID() const { return Master_ID; }
    uint16_t Get_Motor_ID() const { return Motor_ID; }
    bool Get_Configuration_Ready() const { return Ready && Setup_State == Setup_COMPLETE; }
    uint32_t Get_Control_Mode() const { return Control_Mode; }
    uint8_t Get_Last_Tx_Status() const { return Last_Tx_Status; }
    float Get_PID_Target_Omega() const { return Ready ? Target_Omega : 0.0f; }
    // 与 crt_robot_arm 一样，此值是 PID 输出量，不是以 Nm 表示的实际扭矩。
    float Get_PID_Target_Torque() const { return Ready ? Target_Torque : 0.0f; }


private:
    // 旧驱动发送函数使用固定 ID 映射，本适配类只调用自己的发送入口。
    using Class_DM_Motor_J4310::TIM_Process_PeriodElapsedCallback;
    using Class_DM_Motor_J4310::TIM_Alive_PeriodElapsedCallback;
    using Class_DM_Motor_J4310::Clear_Error;
    using Class_DM_Motor_J4310::Set_DM_Motor_Control_Method;
    using Class_DM_Motor_J4310::PID_Torque;

    // V1.3: CTRL_MODE=0x0A，PMAX/VMAX/TMAX=0x15/0x16/0x17。
    enum Enum_Setup_State
    {
        Setup_READ_MODE,
        Setup_DISABLE,
        Setup_WRITE_MODE,
        Setup_READ_PMAX,
        Setup_READ_VMAX,
        Setup_READ_TMAX,
        Setup_COMPLETE
    };
    void Set_Setup_State(Enum_Setup_State state);
    void Process_Setup(uint32_t now);
    bool Process_Parameter_Reply(const uint8_t *data);
    bool Send_Frame(uint16_t id, uint16_t length);

    uint16_t Motor_ID = 0;
    uint16_t Master_ID = 0;
    uint8_t Tx_Data[8] = {}; // CAN 字节缓冲，不是电机对象数组。
    bool Ready = false;
    bool Feedback_Received = false;
    bool Control_Changed = true;
    uint32_t Last_Rx_Tick = 0;
    uint32_t Last_Control_Tick = 0;
    Enum_Setup_State Setup_State = Setup_READ_MODE;
    bool Setup_Request_Pending = true;
    uint32_t Last_Setup_Tick = 0;
    uint32_t Control_Mode = 0;
    uint8_t Last_Tx_Status = 0;
};



#endif
