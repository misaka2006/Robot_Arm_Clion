#ifndef ROBOT_ARM_CLION_RBT_JOINT_DM3520_H
#define ROBOT_ARM_CLION_RBT_JOINT_DM3520_H

#include "rbt_Joint.h"
#include "dvc_joint_motor_dm3520.h"

class Class_Joint_DM3520
{
public:
    Class_Joint_Motor_DM3520 Motor;

    void Set_Target_Radian(float radian);
    void Set_Enable(bool enable);
    bool Get_Online();
    float Get_Now_Radian();
    void TIM_Calculate_PeriodElapsedCallback();
};

#endif // ROBOT_ARM_CLION_RBT_JOINT_DM3520_H
