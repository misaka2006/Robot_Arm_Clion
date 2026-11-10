/**
 * @file alg_gravity_comp.h
 * @brief 机械臂前三关节重力补偿，复用robotics.h/.cpp的Serial_Link<3>::rne()
 * @date 2026-10-17
 *
 * 使用示例:
 *   Class_Gravity_Comp gravity_comp;              //默认构造使用.cpp中的默认配置
 *   float q_rad[3] = {q1, q2, q3};                //关节输出轴角度，rad
 *   gravity_comp.Calculate(q_rad);                //补偿力矩已存入内部数组
 *   const float *torque_nm = gravity_comp.Get_Torque();
 *   //由调用方换算后下发，例如 joint[i].Set_Joint_FeedForward_Torque_Nm(torque_nm[i]);
 */

#ifndef ALG_GRAVITY_COMP_H
#define ALG_GRAVITY_COMP_H

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>

#include "robotics.h"

/* Exported macros -----------------------------------------------------------*/

#define GRAVITY_COMP_JOINT_NUM (3)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 前三关节动力学配置，电流系数与质心由用户自行指定
 * @note  MDH参数(顺序a, alpha, d, theta)已由内部模型固定，不可修改：
 *        关节1 (0,      -pi/2, 69.55mm, q1)
 *        关节2 (230mm,   0,    0,       -q2)
 *        关节3 (0,       pi/2, 0,       -q3)
 */
struct Struct_Gravity_Comp_Config
{
    //电流系数(电机电流单位，非物理质量kg)，三个关节的调参值，请自行填写。
    //输出 = Σ_k Mass[k] * 重力几何矩(m)，输出单位与Mass一致(如Mass填mA则输出mA)。
    float Mass[GRAVITY_COMP_JOINT_NUM] = {0.0f, 0.0f, 0.0f};
    //mm，三个关节质心在各自连杆DH坐标系下的坐标，请自行填写
    float COM_mm[GRAVITY_COMP_JOINT_NUM][3] = {{0.0f, 0.0f, 0.0f},
                                               {0.0f, 0.0f, 0.0f},
                                               {0.0f, 0.0f, 0.0f}};
};

/**
 * @brief 前三关节重力补偿器
 * @note  - 重力方向(0,-9.81,0) m/s^2，在DH坐标系0中表达；
 *        - 输入q为各关节输出轴角度(rad)；
 *        - 输出为抵消重力所需的电流值(单位与Mass一致，如Mass填mA则输出mA)：
 *          内部物理重力矩(N*m)除以g*1m归一化，输出 = Σ_k Mass[k]*几何矩(m)；
 *        - 结果先存入内部数组，不直接发给电机，由调用方读取后自行换算下发；
 *        - 仅含重力项，不含惯性、速度耦合、摩擦与外负载；
 *        - 同一实例不可由任务/中断并发调用。
 */
class Class_Gravity_Comp
{
public:
    Class_Gravity_Comp();

    //以配置重建模型；参数非法时保留旧配置并清零输出。
    bool Init(const Struct_Gravity_Comp_Config &config);

    //运行时仅更新三个关节的电流系数(Mass)并重建模型，质心保持原配置；
    //参数非法时保留旧配置并清零输出。供调试器实时调参调用。
    bool Set_Mass(const float mass[GRAVITY_COMP_JOINT_NUM]);

    //计算重力补偿力矩并存入内部数组；输入或结果非法时清零输出并返回false。
    bool Calculate(const float q_rad[GRAVITY_COMP_JOINT_NUM]);

    //读取补偿电流(单位与Mass一致)；调用方自行换算后下发
    inline float Get_Torque(uint8_t joint_index) const;
    inline const float *Get_Torque() const;

private:
    static robotics::Serial_Link<GRAVITY_COMP_JOINT_NUM> Build_Model(
        const Struct_Gravity_Comp_Config &config);

    void Clear_Out();

    Struct_Gravity_Comp_Config Config_;
    robotics::Serial_Link<GRAVITY_COMP_JOINT_NUM> Arm_;
    float Torque_Comp_[GRAVITY_COMP_JOINT_NUM] = {0.0f, 0.0f, 0.0f};
};

/* Exported function declarations --------------------------------------------*/

inline float Class_Gravity_Comp::Get_Torque(uint8_t joint_index) const
{
    return (joint_index < GRAVITY_COMP_JOINT_NUM) ? Torque_Comp_[joint_index] : 0.0f;
}

inline const float *Class_Gravity_Comp::Get_Torque() const
{
    return Torque_Comp_;
}

#endif //ALG_GRAVITY_COMP_H