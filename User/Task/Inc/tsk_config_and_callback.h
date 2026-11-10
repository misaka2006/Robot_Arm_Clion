#ifndef TSK_CONFIG_AND_CALLBACK_H
#define TSK_CONFIG_AND_CALLBACK_H

#ifdef __cplusplus
#include "rbt_Joint.h"
#include "alg_RNEA.h"

extern Class_Joint_DM_Motor Joint1_DM_Motor;
extern Class_Joint_DM_Motor Joint2_DM_Motor;
extern Class_Joint_DM_Motor Joint3_DM_Motor;
extern Class_Joint_DM_Motor Joint4_DM_Motor;
//机械臂MDH参数(a_i, α_i, d_i, θ_i)，下标0为基座行，长度单位m、角度单位rad
extern const Struct_MDH_Parameter Joint_MDH_Parameter[JOINT_NUM + 1];
//RNEA重力补偿观察数组(N*m)，下标1~JOINT_NUM有效，仅调试查看
extern float Joint_Gravity_Torque_RNEA_Nm[JOINT_NUM + 1];
//RNEA解算耗时统计（DWT CYCCNT，CPU 480MHz），统计范围：设置关节角 → 解算返回
extern volatile uint32_t RNEA_Calc_Start_Cycle;
extern volatile uint32_t RNEA_Calc_End_Cycle;
extern volatile uint32_t RNEA_Calc_Cycles;
extern volatile float RNEA_Calc_Time_Us;
extern volatile uint32_t RNEA_Calc_Cycles_Min;
extern volatile uint32_t RNEA_Calc_Cycles_Max;
extern volatile float RNEA_Calc_Time_Us_Min;
extern volatile float RNEA_Calc_Time_Us_Max;
extern volatile uint64_t RNEA_Calc_Cycles_Total;
extern volatile uint32_t RNEA_Calc_Count;
extern volatile float RNEA_Calc_Time_Us_Avg;
//3关节重力补偿观察数组(robotics库Class_Gravity_Comp)，电机电流单位(与Mass一致)，仅调试查看，不下发电机。
extern float Joint_Gravity_Torque_3J_Nm[3];
//Ozone调参输入：关节1~3电流系数(电机电流单位，与Mass一致)，调试器中可实时修改。
extern volatile float Joint_Gravity_Mass_3J[3];
// Joint5 使用 DM-S2325-1EC，反馈角度已经是减速后的输出轴角度。
extern Class_Joint_DM_Gear Joint5_DM_Motor;
//Ozone校准诊断：Init执行/参数已写入/当前状态/该状态持续时间。
extern volatile bool Joint5_Calibration_Initialized;
extern volatile bool Joint5_Calibration_Parameter_Valid;
extern volatile uint8_t Joint5_Calibration_Status;
//写true请求启动/重新校准，消费后自动清零。
extern volatile bool Joint5_Calibration_Start_Request;
extern volatile uint32_t Joint5_Calibration_State_Time_Ms;
//最新反馈力矩（带符号），校准判定使用该数据的绝对值。
extern volatile float Joint5_Feedback_Torque_Nm;

extern "C" {
#endif

void Task_Init(void);

#ifdef __cplusplus
}
#endif

#endif
