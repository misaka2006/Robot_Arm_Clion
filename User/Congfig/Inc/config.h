/**
 * @file config.h
 * @author cjw
 * @brief 工程配置文件
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

#ifndef CONFIG_H
#define CONFIG_H

/* Includes ------------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

// 新机械臂 J0：CAN1，调参助手 CANID=0x01、MASTERID=0x21。
// V1.3 手册建议 CANID < 16，避免与反馈 ERR 位重叠；须同步修改电机内部地址。
// ARM_CAN 使用 H7 工程实际句柄名；任务层在 CAN1 回调中分发。
#define ARM_CAN hfdcan1
#define J0_CAN_ID 0x01
#define J0_MASTER_ID 0x21
// J1 与 J0 同型电机、同一总线；CANID/MASTERID 必须与 J0 不同：
// 问询式反馈按帧 ID 低 8 位匹配，两电机 ID 相同会同时应答，反馈无法区分且总线冲突。
#define J1_CAN_ID 0x02
#define J1_MASTER_ID 0x22

#define J2_CAN_ID 0x03
#define J2_MASTER_ID 0x23

#define J3_CAN_ID 0x04
#define J3_MASTER_ID 0x24

#define J4_CAN_ID 0x05
#define J4_MASTER_ID 0x25
// J4 DM-S2325-1EC 的驱动器量程，应与调参助手中的设置保持一致。
#define J4_POSITION_MAX 3.1415926f
#define J4_OMEGA_MAX 380.0f
#define J4_TORQUE_MAX 10.0f
// 电机内部 MIT Kp/Kd 为 0；J4 使用下面的 STM32 串级 PID。
#define J4_MIT_K_P 0.0f
#define J4_MIT_K_D 0.0f
// 角度环输出 rad/s，速度环输出输出轴 Nm；以下为待实机整定的低增益起点。
#define J4_ANGLE_PID_KP 5.0f
#define J4_ANGLE_PID_KI 0.0f
#define J4_ANGLE_PID_KD 0.0f
#define J4_ANGLE_PID_MAX_OMEGA 2.0f
#define J4_OMEGA_PID_KP 1.0f
#define J4_OMEGA_PID_KI 0.0f
#define J4_OMEGA_PID_KD 0.0f
#define J4_OMEGA_PID_MAX_TORQUE 2.0f

#define J5_CAN_ID 0x06
#define J5_MASTER_ID 0x26
#define J6_CAN_ID 0x07
#define J6_MASTER_ID 0x27

// 初始化占位量程；使能前从电机读取 PMAX/VMAX/TMAX，按实机值解码反馈。
// 目前直接使用输出轴坐标，零点偏移为 0，额外减速比为 1。
#define J0_POSITION_MAX 3.141593f
#define J0_OMEGA_MAX 20.94359f
#define J0_TORQUE_MAX 10.0f
// 关节软件限位独立于反馈映射量程。
#define J0_MIN_RADIAN (-3.141593f)
#define J0_MAX_RADIAN 3.141593f
// J0 由 STM32 每 1 ms 计算角度外环 -> 速度内环，MIT 报文仅输出力矩。
// 增益为待调值：初始均为 0，不预设未经实机验证的 PID 增益。
#define J0_ANGLE_KP 0.0f
#define J0_ANGLE_KI 0.0f
#define J0_ANGLE_KD 0.0f
#define J0_ANGLE_I_OUT_MAX 0.1f       // rad/s
#define J0_OMEGA_KP 0.0f
#define J0_OMEGA_KI 0.0f
#define J0_OMEGA_KD 0.0f
#define J0_OMEGA_I_OUT_MAX 0.2f       // Nm
#define J0_PID_OMEGA_LIMIT 0.5f      // rad/s，角度环输出限幅
#define J0_PID_TORQUE_LIMIT 1.0f     // Nm，速度环输出限幅
#define J0_TARGET_DEGREE 30.0f
#define J0_PID_DEBUG_ONLY 1          // 1: 1 ms 任务只调度新 J0；0: 恢复原整车调度

//底盘或云台状态
// #define CHASSIS
#define GIMBAL

//调试或比赛状态
#define DEBUG

// 机械臂类型选择
//#define PUMA
#define SCARA
//之前写的机械臂电机测试入口
//#define MOTOR_TEST

//用于测试正逆解角度映射的入口
//#define MY_DEBUG

//底盘电机测试入口
//#define MOTOR_TEST_CHASSIS

//底盘测试入口
//#define CHASSIS_TEST

//功率控制相关
#define POWER_CONTROL 1 //启用功率控制
//#define BUFFER_LOOP

//遥控器选择
#define USE_VT13
#define USE_DR16

/* 兵种/底盘类型选择*/
//#define AGV      //舵轮底盘
//#define OMNI_WHEEL //全向轮底盘

//#define INFANTRY //步兵
//#define HERO  //英雄
//#define SENTRY //哨兵
#define ENGINEER //工程

/*轮组数据*/
#ifdef INFANTRY
#define Wheel_Diameter 0.12000000f // 轮子直径，单位为m
#endif 

#ifdef HERO
#define Wheel_Diameter 0.12000000f // 轮子直径，单位为m
#endif 

#ifdef SENTRY
#define Wheel_Diameter 0.12000000f // 轮子直径，单位为m
#define Chassis_Radius 0.46000000f // 底盘半径，单位为m
#endif

#ifdef ENGINEER
#define Wheel_Diameter 0.141000000f
#endif



/* Exported types ------------------------------------------------------------*/


/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
