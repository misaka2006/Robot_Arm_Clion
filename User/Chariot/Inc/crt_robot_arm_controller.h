/** @file crt_robot_arm_controller.h
 *  @brief 当前机械臂实例：手动声明 J0 DM4310。
 */
#ifndef CRT_ROBOT_ARM_CONTROLLER_H
#define CRT_ROBOT_ARM_CONTROLLER_H

#include "crt_robot_arm.h"
#include "dvc_joint_motor_dm4310.h"
#include "config.h"

class Class_Robot_Arm_Controller : public Class_Robot_Arm
{
public:
    Class_Joint_Motor_DM4310 J0_Motor;

    bool Init() override;
    // 机械臂周期更新入口，由 Task1ms_TIM5_Callback 每 1 ms 调用。
    void TIM_Calculate_PeriodElapsedCallback() override;
    bool Set_Target_J0_Radian(float radian);
    bool Get_Target_J0_Radian(float &radian) const;
    bool Get_Now_J0_Radian(float &radian) const;
    void CAN_RxCpltCallback(Struct_CAN_Rx_Buffer *message);

protected:
    bool Check_Joints_Config() const override;
    bool Update_Feedback() override;
    void Sync_Target_To_Feedback() override;
    void Output() override;

private:
    Struct_Robot_Arm_Joint J0{J0_MIN_RADIAN, J0_MAX_RADIAN};
};

#endif
