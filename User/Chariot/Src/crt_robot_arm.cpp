/**
 * @file crt_robot_arm.cpp
 * @brief 机械臂公共状态控制及单关节辅助函数。
 */
#include "crt_robot_arm.h"
#include "config.h"
#include <cmath>

void Class_Robot_Arm::Init()
{
    // J0 由任务层的 Joint1_DM_Motor 实例管理。
    // Joint1 与 Joint0 同型同配置，仅 CANID/MASTERID 不同
    Joint1_Motor.Init(&ARM_CAN,(Enum_DM_Motor_ID)J1_CAN_ID,DM_Motor_Control_Method_MIT_Angle,0,PI,10.0f,12.0f);
    Joint1_Motor.Set_MIT_K_D(0.0f);
    Joint1_Motor.Set_MIT_K_P(0.0f);

    Joint1_Motor.PID_Angle.Init(60.0f, 0.01f, 0.0f, 0.0f, 3.0f, 10.0f, 0.0f, 0.0f, 0.0f, 0.001f, 0.0f, PID_D_First_ENABLE);
    Joint1_Motor.PID_Omega.Init(200.0f, 0.02f, 0.00f, 0.0f, 1000, 2048, 0.0f, 0.0f, 0.0f, 0.001f, 0.5f);

    Joint1_Motor.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
    Joint1_Motor.TIM_Alive_PeriodElapsedCallback();

    // Joint2 与 Joint1 同型同配置，仅 CANID/MASTERID 不同
    Joint2_Motor.Init(&ARM_CAN,(Enum_DM_Motor_ID)J2_CAN_ID,DM_Motor_Control_Method_MIT_Angle,0,PI,10.0f,12.0f);
    Joint2_Motor.Set_MIT_K_D(0.0f);
    Joint2_Motor.Set_MIT_K_P(0.0f);

    Joint2_Motor.PID_Angle.Init(60.0f, 0.01f, 0.0f, 0.0f, 3.0f, 10.0f, 0.0f, 0.0f, 0.0f, 0.001f, 0.0f, PID_D_First_ENABLE);
    Joint2_Motor.PID_Omega.Init(200.0f, 0.02f, 0.00f, 0.0f, 1000, 2048, 0.0f, 0.0f, 0.0f, 0.001f, 0.5f);

    Joint2_Motor.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
    Joint2_Motor.TIM_Alive_PeriodElapsedCallback();

    // Joint3 与 Joint2 同型同配置，仅 CANID/MASTERID 不同
    Joint3_Motor.Init(&ARM_CAN,(Enum_DM_Motor_ID)J3_CAN_ID,DM_Motor_Control_Method_MIT_Angle,0,PI,10.0f,12.0f);
    Joint3_Motor.Set_MIT_K_D(0.0f);
    Joint3_Motor.Set_MIT_K_P(0.0f);

    Joint3_Motor.PID_Angle.Init(60.0f, 0.01f, 0.0f, 0.0f, 3.0f, 10.0f, 0.0f, 0.0f, 0.0f, 0.001f, 0.0f, PID_D_First_ENABLE);
    Joint3_Motor.PID_Omega.Init(200.0f, 0.02f, 0.00f, 0.0f, 1000, 2048, 0.0f, 0.0f, 0.0f, 0.001f, 0.5f);

    Joint3_Motor.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
    Joint3_Motor.TIM_Alive_PeriodElapsedCallback();

    // Joint4 使用 DM3520；CAN 位置、速度按减速前数据处理，除以 25 后用于 PID。
    // 当前配置 PMAX=PI、VMAX=200、TMAX=10、Gr=25，电机端参数需保持一致。
    // 映射范围不是运动限幅，角度环的速度输出限幅仍由 PID 参数独立设置。
    Joint4_Motor.Init(&ARM_CAN,(Enum_DM_Motor_ID)J4_CAN_ID,DM_Motor_Control_Method_MIT_Angle,0,PI,200.0f,10.0f);
    Joint4_Motor.Set_Gear_Ratio(25.0f);
    Joint4_Motor.Set_MIT_K_D(0.0f);
    Joint4_Motor.Set_MIT_K_P(0.0f);

    Joint4_Motor.PID_Angle.Init(5.0f, 0.01f, 0.0f, 0.0f, 3.0f, 10.0f, 0.0f, 0.0f, 0.0f, 0.001f, 0.0f, PID_D_First_ENABLE);
    // 速度死区设为 0：Kp_angle=5 时，原 0.5 rad/s 死区会吞掉约 5.7 度的位置误差。
    Joint4_Motor.PID_Omega.Init(200.0f, 0.02f, 0.00f, 0.0f, 1000, 2048, 0.0f, 0.0f, 0.0f, 0.001f, 0.0f);

    Joint4_Motor.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
    Joint4_Motor.TIM_Alive_PeriodElapsedCallback();

    // Joint5 与 Joint4 类型和参数一致，仅使用独立 CAN 地址。
    Joint5_Motor.Init(&ARM_CAN,(Enum_DM_Motor_ID)J5_CAN_ID,DM_Motor_Control_Method_MIT_Angle,0,PI,200.0f,10.0f);
    Joint5_Motor.Set_Gear_Ratio(25.0f);
    Joint5_Motor.Set_MIT_K_D(0.0f);
    Joint5_Motor.Set_MIT_K_P(0.0f);

    Joint5_Motor.PID_Angle.Init(20.0f, 0.0f, 0.0f, 0.0f, 3.0f, 10.0f, 0.0f, 0.0f, 0.0f, 0.001f, 0.0f, PID_D_First_ENABLE);
    // 速度死区设为 0：Kp_angle=5 时，原 0.5 rad/s 死区会吞掉约 5.7 度的位置误差。
    Joint5_Motor.PID_Omega.Init(20.0f, 0.0f, 0.00f, 0.0f, 1000, 2048, 0.0f, 0.0f, 0.0f, 0.001f, 0.0f);

    Joint5_Motor.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
    Joint5_Motor.TIM_Alive_PeriodElapsedCallback();

    // Joint6 复用 Joint5 当前参数，使用独立 CAN 地址。
    Joint6_Motor.Init(&ARM_CAN,(Enum_DM_Motor_ID)J6_CAN_ID,DM_Motor_Control_Method_MIT_Angle,0,PI,200.0f,10.0f);
    Joint6_Motor.Set_Gear_Ratio(25.0f);
    Joint6_Motor.Set_MIT_K_D(0.0f);
    Joint6_Motor.Set_MIT_K_P(0.0f);

    Joint6_Motor.PID_Angle.Init(20.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 2048.0f, 0.0f, 0.0f, 0.0f, 0.001f, 0.0f, PID_D_First_ENABLE);
    // 速度死区设为 0，保留小速度指令。
    Joint6_Motor.PID_Omega.Init(20.0f, 0.0f, 0.00f, 0.0f, 1000, 2048, 0.0f, 0.0f, 0.0f, 0.001f, 0.0f);

    Joint6_Motor.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
    Joint6_Motor.TIM_Alive_PeriodElapsedCallback();

    Robot_Arm_Control_Type = Robot_Arm_Control_Type_POSITION;
}

void Class_Robot_Arm::Output()
{
    if(Robot_Arm_Control_Type == Robot_Arm_Control_Type_DISABLE)
    {
        Joint1_Motor.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        Joint2_Motor.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        Joint3_Motor.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        Joint4_Motor.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        Joint5_Motor.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        Joint6_Motor.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);

    }

    if(Robot_Arm_Control_Type == Robot_Arm_Control_Type_POSITION)
    {
        Joint1_Motor.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        Joint1_Motor.Set_Target_Angle(Joint1_Target_Angle_Rad);

        Joint2_Motor.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        Joint2_Motor.Set_Target_Angle(Joint2_Target_Angle_Rad);

        Joint3_Motor.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        Joint3_Motor.Set_Target_Angle(Joint3_Target_Angle_Rad);

        Joint4_Motor.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        Joint4_Motor.Set_Target_Angle(Joint4_Target_Angle_Rad);

        Joint5_Motor.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        Joint6_Motor.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        Joint5_Motor.Set_Target_Angle(Joint5_Target_Angle_Rad);
        Joint6_Motor.Set_Target_Angle(Joint6_Target_Angle_Rad);
    }
}

void Class_Robot_Arm::TIM_Calculate_PeriodElapsedCallback()
{
    Output();

    // J1-J6 每 1ms 计算 PID；J0 由任务层计算（D_T=0.001 依赖 1kHz 调用率）。
    Joint1_Motor.TIM_PID_PeriodElapsedCallback();
    Joint2_Motor.TIM_PID_PeriodElapsedCallback();
    Joint3_Motor.TIM_PID_PeriodElapsedCallback();
    Joint4_Motor.TIM_PID_PeriodElapsedCallback();
    Joint5_Motor.TIM_PID_PeriodElapsedCallback();
    Joint6_Motor.TIM_PID_PeriodElapsedCallback();

    // 仿云台 can_priority_cnt 机制：每 1ms 只轮发一个关节的控制帧，避免 CAN1 拥塞
    static uint8_t arm_priority_cnt = 0;
    // 显式按七个时隙回绕，避免 uint8_t 溢出打乱发送周期。
    arm_priority_cnt = (arm_priority_cnt + 1) % 7;
    switch (arm_priority_cnt)
    {
    case (1):
        // J0 在任务层同一时隙发送。
        break;
    case (2):
        Joint1_Motor.TIM_Send_PeriodElapsedCallback();
        break;
    case (3):
        Joint2_Motor.TIM_Send_PeriodElapsedCallback();
        break;
    case (4):
        Joint3_Motor.TIM_Send_PeriodElapsedCallback();
        break;
    case (5):
        Joint4_Motor.TIM_Send_PeriodElapsedCallback();
        break;
    case (6):
        Joint5_Motor.TIM_Send_PeriodElapsedCallback();
        break;
    case (0):
        Joint6_Motor.TIM_Send_PeriodElapsedCallback();
        break;
    }
}

float Class_Robot_Arm::Get_Joint1_Target_Angle_Deg()
{
    return Joint1_Target_Angle_Deg;
}

float Class_Robot_Arm::Get_Joint1_Target_Angle_Rad()
{
    return Joint1_Target_Angle_Rad;
}

void Class_Robot_Arm::Set_Joint1_Target_Angle_Deg(float target)
{
    Joint1_Target_Angle_Deg = target;
    Joint1_Target_Angle_Rad = target * PI / 180.0f;
}

void Class_Robot_Arm::Set_Joint1_Target_Angle_Rad(float target)
{
    Joint1_Target_Angle_Rad = target;
    Joint1_Target_Angle_Deg = target * 180.0f / PI;
}

float Class_Robot_Arm::Get_Joint2_Target_Angle_Deg()
{
    return Joint2_Target_Angle_Deg;
}

float Class_Robot_Arm::Get_Joint2_Target_Angle_Rad()
{
    return Joint2_Target_Angle_Rad;
}

void Class_Robot_Arm::Set_Joint2_Target_Angle_Deg(float target)
{
    Joint2_Target_Angle_Deg = target;
    Joint2_Target_Angle_Rad = target * PI / 180.0f;
}

void Class_Robot_Arm::Set_Joint2_Target_Angle_Rad(float target)
{
    Joint2_Target_Angle_Rad = target;
    Joint2_Target_Angle_Deg = target * 180.0f / PI;
}

float Class_Robot_Arm::Get_Joint3_Target_Angle_Deg()
{
    return Joint3_Target_Angle_Deg;
}

float Class_Robot_Arm::Get_Joint3_Target_Angle_Rad()
{
    return Joint3_Target_Angle_Rad;
}

void Class_Robot_Arm::Set_Joint3_Target_Angle_Deg(float target)
{
    Joint3_Target_Angle_Deg = target;
    Joint3_Target_Angle_Rad = target * PI / 180.0f;
}

void Class_Robot_Arm::Set_Joint3_Target_Angle_Rad(float target)
{
    Joint3_Target_Angle_Rad = target;
    Joint3_Target_Angle_Deg = target * 180.0f / PI;
}

float Class_Robot_Arm::Get_Joint4_Target_Angle_Deg()
{
    return Joint4_Target_Angle_Deg;
}

float Class_Robot_Arm::Get_Joint4_Target_Angle_Rad()
{
    return Joint4_Target_Angle_Rad;
}

void Class_Robot_Arm::Set_Joint4_Target_Angle_Deg(float target)
{
    Joint4_Target_Angle_Deg = target;
    Joint4_Target_Angle_Rad = target * PI / 180.0f;
}

void Class_Robot_Arm::Set_Joint4_Target_Angle_Rad(float target)
{
    Joint4_Target_Angle_Rad = target;
    Joint4_Target_Angle_Deg = target * 180.0f / PI;
}

float Class_Robot_Arm::Get_Joint5_Target_Angle_Deg()
{
    return Joint5_Target_Angle_Deg;
}

float Class_Robot_Arm::Get_Joint5_Target_Angle_Rad()
{
    return Joint5_Target_Angle_Rad;
}

void Class_Robot_Arm::Set_Joint5_Target_Angle_Deg(float target)
{
    Joint5_Target_Angle_Deg = target;
    Joint5_Target_Angle_Rad = target * PI / 180.0f;
}

void Class_Robot_Arm::Set_Joint5_Target_Angle_Rad(float target)
{
    Joint5_Target_Angle_Rad = target;
    Joint5_Target_Angle_Deg = target * 180.0f / PI;
}

float Class_Robot_Arm::Get_Joint6_Target_Angle_Deg()
{
    return Joint6_Target_Angle_Deg;
}

float Class_Robot_Arm::Get_Joint6_Target_Angle_Rad()
{
    return Joint6_Target_Angle_Rad;
}

void Class_Robot_Arm::Set_Joint6_Target_Angle_Deg(float target)
{
    Joint6_Target_Angle_Deg = target;
    Joint6_Target_Angle_Rad = target * PI / 180.0f;
}

void Class_Robot_Arm::Set_Joint6_Target_Angle_Rad(float target)
{
    Joint6_Target_Angle_Rad = target;
    Joint6_Target_Angle_Deg = target * 180.0f / PI;
}
