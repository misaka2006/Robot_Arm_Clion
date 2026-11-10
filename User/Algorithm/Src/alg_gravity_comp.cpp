/**
 * @file alg_gravity_comp.cpp
 * @brief 机械臂前三关节重力补偿，复用robotics.h/.cpp的Serial_Link<3>::rne()
 * @date 2026-10-17
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_gravity_comp.h"

#include <cmath>

/* Private constants & helpers ------------------------------------------------*/

namespace
{
//mm转m
const float k_mm_to_m = 0.001f;
//pi/2
const float k_half_pi = 1.57079632679489661923f;
//重力加速度在DH坐标系0中的分量，m/s^2
//非const: Matrixf构造仅拷贝数据，不修改输入
float k_gravity_base[3] = {0.0f, -9.81f, 0.0f};
//输出归一化: 物理重力矩τ = Σ_k Mass[k]*g*几何矩(m)，除以g*1m后
//输出 = Σ_k Mass[k]*几何矩(m)，与Mass同单位(电机电流单位)
const float k_output_normalize = 1.0f / 9.81f;

bool Is_Finite(const float *values, uint8_t count)
{
    for (uint8_t i = 0; i < count; i++)
    {
        if (!std::isfinite(values[i]))
        {
            return false;
        }
    }
    return true;
}
}

/**
 * @brief 默认动力学配置
 * @note  请自行填写三个关节的电流系数(电机电流单位，非物理质量kg)
 *        与质心(mm，各连杆自身DH坐标系)。
 *        填写后默认构造即可直接使用；运行时也可调用Init覆盖。
 *        全零时输出恒为零。
 */
static Struct_Gravity_Comp_Config Build_Default_Config(void)
{
    Struct_Gravity_Comp_Config config;  //Mass与COM_mm默认全0

    //TODO: 关节1电流系数(电机电流单位)
    config.Mass[0] = 0.0f;
    //TODO: 关节1质心(mm，连杆1DH坐标系)
    config.COM_mm[0][0] = 0.0f;
    config.COM_mm[0][1] = 0.0f;
    config.COM_mm[0][2] = 0.0f;

    //TODO: 关节2电流系数(电机电流单位)
    config.Mass[1] = 90.0f;
    //TODO: 关节2质心(mm，连杆2DH坐标系)
    config.COM_mm[1][0] = 165.329850737474f;
    config.COM_mm[1][1] = 0.0f;
    config.COM_mm[1][2] = 0.0f;

    //TODO: 关节3电流系数(电机电流单位)
    config.Mass[2] = 30.0f;
    //TODO: 关节3质心(mm，连杆3DH坐标系)
    config.COM_mm[2][0] = 50.0f;
    config.COM_mm[2][1] = 0.0f;
    config.COM_mm[2][2] = 0.0f;

    return config;
}

/**
 * @brief 由配置构建前三关节模型
 * @note  MDH参数(顺序a, alpha, d, theta)转为robotics::Link(theta, d, a, alpha)：
 *        关节1 (0, -pi/2, 69.55mm, q1)-> theta=q1,      d=69.55mm(沿z0,对重力矩无贡献), a=0, alpha=-pi/2
 *        关节2 (230mm, 0, 0, -q2)      -> theta=-q2,     d=0,      a=230mm,   alpha=0
 *        关节3 (0, pi/2, 0, -q3)       -> theta=-q3,     d=0,      a=0,       alpha=pi/2
 *        69.55mm原为关节2的d2(沿z1)，使q2=q3=0时τ1出现-d2*(m2+m3)*sin(q1)项；
 *        修正后放到关节1的d(沿z0)。theta中的符号与偏置由调用处统一处理：
 *        模型变量q_model=(q1,-q2,-q3)，offset=(0,0,0)，即theta=q_model。
 */
robotics::Serial_Link<GRAVITY_COMP_JOINT_NUM> Class_Gravity_Comp::Build_Model(
    const Struct_Gravity_Comp_Config &config)
{
    //质心: mm -> m
    float rc_m[GRAVITY_COMP_JOINT_NUM][3];
    for (uint8_t i = 0; i < GRAVITY_COMP_JOINT_NUM; i++)
    {
        for (uint8_t j = 0; j < 3; j++)
        {
            rc_m[i][j] = config.COM_mm[i][j] * k_mm_to_m;
        }
    }

    //惯量置零: 静态计算(qv=qa=0)时rne中的惯量项全为0，不影响结果
    robotics::Link links[GRAVITY_COMP_JOINT_NUM] = {
        //关节1: theta=q1, d=69.55mm(沿z0), a=0, alpha=-pi/2
        robotics::Link(0.0f, 69.55f * k_mm_to_m, 0.0f, -k_half_pi, robotics::R,
                       0.0f, 0.0f, 0.0f, config.Mass[0],
                       Matrixf<3, 1>(rc_m[0]), matrixf::zeros<3, 3>()),
        //关节2: theta=-q2, d=0, a=230mm, alpha=0
        robotics::Link(0.0f, 0.0f, 230.0f * k_mm_to_m, 0.0f,
                       robotics::R, 0.0f, 0.0f, 0.0f, config.Mass[1],
                       Matrixf<3, 1>(rc_m[1]), matrixf::zeros<3, 3>()),
        //关节3: theta=-q3, d=0, a=0, alpha=pi/2
        robotics::Link(0.0f, 0.0f, 0.0f, k_half_pi, robotics::R,
                       0.0f, 0.0f, 0.0f, config.Mass[2],
                       Matrixf<3, 1>(rc_m[2]), matrixf::zeros<3, 3>()),
    };

    return robotics::Serial_Link<GRAVITY_COMP_JOINT_NUM>(
        links, Matrixf<3, 1>(k_gravity_base));
}

Class_Gravity_Comp::Class_Gravity_Comp()
    : Config_(Build_Default_Config()),
      Arm_(Build_Model(Config_))
{
    Clear_Out();
}

bool Class_Gravity_Comp::Init(const Struct_Gravity_Comp_Config &config)
{
    Clear_Out();
    //先整组校验，失败保留旧配置
    for (uint8_t i = 0; i < GRAVITY_COMP_JOINT_NUM; i++)
    {
        if (!std::isfinite(config.Mass[i]) || config.Mass[i] < 0.0f ||
            !Is_Finite(config.COM_mm[i], 3))
        {
            return false;
        }
    }
    Config_ = config;
    Arm_ = Build_Model(Config_);
    return true;
}

bool Class_Gravity_Comp::Set_Mass(const float mass[GRAVITY_COMP_JOINT_NUM])
{
    Clear_Out();
    //先整组校验，失败保留旧配置
    if (mass == nullptr || !Is_Finite(mass, GRAVITY_COMP_JOINT_NUM))
    {
        return false;
    }
    for (uint8_t i = 0; i < GRAVITY_COMP_JOINT_NUM; i++)
    {
        if (mass[i] < 0.0f)
        {
            return false;
        }
    }
    for (uint8_t i = 0; i < GRAVITY_COMP_JOINT_NUM; i++)
    {
        Config_.Mass[i] = mass[i];
    }
    Arm_ = Build_Model(Config_);
    return true;
}

bool Class_Gravity_Comp::Calculate(const float q_rad[GRAVITY_COMP_JOINT_NUM])
{
    Clear_Out();
    if (q_rad == nullptr || !Is_Finite(q_rad, GRAVITY_COMP_JOINT_NUM))
    {
        return false;
    }

    //模型变量: theta1=q1, theta2=-q2, theta3=pi/2-q3
    float q_model[GRAVITY_COMP_JOINT_NUM] = {q_rad[0], -q_rad[1], -q_rad[2]};

    //静态RNEA: qv=qa=0、末端负载he=0，rne()输出即抵消重力所需关节力矩(N*m)
    Matrixf<GRAVITY_COMP_JOINT_NUM, 1> tau_model =
        Arm_.rne(Matrixf<GRAVITY_COMP_JOINT_NUM, 1>(q_model));

    //换算回用户关节变量q的正方向: theta1=q1, theta2=-q2, theta3=pi/2-q3
    //再除以g*1m归一化，输出与Mass同单位(电机电流单位)
    Torque_Comp_[0] = tau_model[0][0] * k_output_normalize;
    Torque_Comp_[1] = -tau_model[1][0] * k_output_normalize;
    Torque_Comp_[2] = -tau_model[2][0] * k_output_normalize;

    if (!Is_Finite(Torque_Comp_, GRAVITY_COMP_JOINT_NUM))
    {
        Clear_Out();
        return false;
    }
    return true;
}

void Class_Gravity_Comp::Clear_Out()
{
    for (uint8_t i = 0; i < GRAVITY_COMP_JOINT_NUM; i++)
    {
        Torque_Comp_[i] = 0.0f;
    }
}