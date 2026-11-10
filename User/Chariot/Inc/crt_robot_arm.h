/**
 * @file crt_robot_arm.h
 * @brief 手动声明关节电机的机械臂控制基类。
 */
#ifndef CRT_ROBOT_ARM_H
#define CRT_ROBOT_ARM_H

#include <stdint.h>
#include "dvc_dmmotor.h"
#include "dvc_joint_motor_dm3520.h"

enum Enum_Robot_Arm_Control_Type : uint8_t
{
    Robot_Arm_Control_Type_DISABLE = 0,
    Robot_Arm_Control_Type_POSITION,
    Robot_Arm_Control_Type_FORCE,
};

/**
 * @brief 
 *
 * 
 */
class Class_Robot_Arm
{

    public:

    Class_DM_Motor_J4310 Joint1_Motor;
    Class_DM_Motor_J4310 Joint2_Motor;
    Class_DM_Motor_J4310 Joint3_Motor;
    Class_Joint_Motor_DM3520 Joint4_Motor;
    Class_Joint_Motor_DM3520 Joint5_Motor;
    Class_Joint_Motor_DM3520 Joint6_Motor;

    void Init();
    void Output();
    void TIM_Calculate_PeriodElapsedCallback();

    float Get_Joint1_Target_Angle_Deg();
    float Get_Joint1_Target_Angle_Rad();
    void Set_Joint1_Target_Angle_Deg(float target);
    void Set_Joint1_Target_Angle_Rad(float target);

    float Get_Joint2_Target_Angle_Deg();
    float Get_Joint2_Target_Angle_Rad();
    void Set_Joint2_Target_Angle_Deg(float target);
    void Set_Joint2_Target_Angle_Rad(float target);

    float Get_Joint3_Target_Angle_Deg();
    float Get_Joint3_Target_Angle_Rad();
    void Set_Joint3_Target_Angle_Deg(float target);
    void Set_Joint3_Target_Angle_Rad(float target);

    float Get_Joint4_Target_Angle_Deg();
    float Get_Joint4_Target_Angle_Rad();
    void Set_Joint4_Target_Angle_Deg(float target);
    void Set_Joint4_Target_Angle_Rad(float target);

    float Get_Joint5_Target_Angle_Deg();
    float Get_Joint6_Target_Angle_Deg();
    float Get_Joint5_Target_Angle_Rad();
    float Get_Joint6_Target_Angle_Rad();
    void Set_Joint5_Target_Angle_Deg(float target);
    void Set_Joint6_Target_Angle_Deg(float target);
    void Set_Joint5_Target_Angle_Rad(float target);
    void Set_Joint6_Target_Angle_Rad(float target);

private:
    Enum_Robot_Arm_Control_Type Robot_Arm_Control_Type = Robot_Arm_Control_Type_DISABLE;
    float Joint1_Target_Angle_Deg = 0.0f;
    float Joint1_Target_Angle_Rad = 0.0f;
    float Joint2_Target_Angle_Deg = 0.0f;
    float Joint2_Target_Angle_Rad = 0.0f;
    float Joint3_Target_Angle_Deg = 0.0f;
    float Joint3_Target_Angle_Rad = 0.0f;
    float Joint4_Target_Angle_Deg = 0.0f;
    float Joint4_Target_Angle_Rad = 0.0f;
    float Joint5_Target_Angle_Deg = 0.0f;
    float Joint6_Target_Angle_Deg = 0.0f;
    float Joint5_Target_Angle_Rad = 0.0f;
    float Joint6_Target_Angle_Rad = 0.0f;
};


#endif // CRT_ROBOT_ARM_H
