/**
 * @file rnea_gravity_example.cpp
 * @brief 静态RNEA调用示例，本文件不被固件CMake自动编译
 *
 * 1. 填写实测a1/a2/d2/d4（m）、每个活动连杆的质量（kg）。
 * 2. 一阶质量矩First_Moment = Mass * [cx,cy,cz]，单位kg*m，默认全零。
 *    c相对对应连杆DH原点表达；h=0相当于质心放在该DH原点。
 *    即使h=0，非零质量仍可通过DH连杆长度产生重力矩。
 * 3. 输入关节输出轴相对模型零位的角度（rad），不要重复添加q3的pi/2。
 * 4. 输出为抵消重力所需的关节力矩（N*m），不是电机电流或标幺值。
 *    必须按实测力矩常数、传动比和方向换算后再接控制器。
 * 5. 当前基座固定，世界重力[0,0,-9.81]在DH坐标系0中为[0,-9.81,0]。
 *    若基座倾斜，应更新Gravity_Base并重新Init。
 * 6. 静态计算不需要惯量、角速度和角加速度。工具/负载的质量及一阶矩
 *    应合并到连杆4的DH坐标系中，工具坐标点的显示偏移本身没有质量。
 * 7. Init返回false保留旧配置；Calculate_Gravity失败会清零输出。
 *    同一实例应只由一个任务调用，读取输出也应与该任务同步。
 *
 * 算法依据: https://modernrobotics.northwestern.edu/nu-gm-book-resource/8-3-newton-euler-inverse-dynamics/
 * 采用qd=qdd=0的静态特例，只做正向姿态与反向力/力矩递推。
 */

#include "alg_rnea.h"

static Class_RNEA Gravity_Compensation;

bool RNEA_Gravity_Example_Init(const float mass[RNEA_JOINT_NUM],
                             float a1, float a2, float d2, float d4)
{
    if (mass == nullptr)
    {
        return false;
    }
    Struct_RNEA_Model model;
    model.Link[0].A = a1;
    model.Link[1].A = a2;
    model.Link[1].D = d2;
    model.Link[3].D = d4;

    for (uint8_t i = 0; i < RNEA_JOINT_NUM; i++)
    {
        model.Link[i].Mass = mass[i];
        model.Link[i].First_Moment[0] = 0.0f; //后续填写m*cx
        model.Link[i].First_Moment[1] = 0.0f; //后续填写m*cy
        model.Link[i].First_Moment[2] = 0.0f; //后续填写m*cz
    }
    return Gravity_Compensation.Init(model);
}

bool RNEA_Gravity_Example_Update(const float q[RNEA_JOINT_NUM], float torque[RNEA_JOINT_NUM])
{
    if (torque == nullptr)
    {
        return false;
    }
    const bool valid = Gravity_Compensation.Calculate_Gravity(q);
    for (uint8_t i = 0; i < RNEA_JOINT_NUM; i++)
    {
        torque[i] = Gravity_Compensation.Get_Out(i);
    }
    return valid;
}
