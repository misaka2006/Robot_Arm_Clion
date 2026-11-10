//
// Created by minec on 2026/10/25.
//

#include "alg_RNEA.h"

void Class_RNEA::Init() {
    // 矩阵/数组清零，齐次变换矩阵统一建成4×4
    for (uint8_t i = 0 ; i<=JOINT_NUM ; i++) {
        Homo_Trans_Matrix_Global[i].Init(4, 4, 0.0f);
        Homo_Trans_Matrix_Local[i].Init(4, 4, 0.0f);

        Joint_Moment_Nm[i].Set(0.0f, 0.0f, 0.0f);
        Joint_Force_N[i].Set(0.0f, 0.0f, 0.0f);
        Joint_Gravity_Comp_Torque_Nm[i] = 0.0f;
    }

    // 连杆0即基坐标系，相对自身的变换为单位阵
    Homo_Trans_Matrix_Local[0].Set_Identity();
    Homo_Trans_Matrix_Global[0].Set_Identity();

    // 按当前关节角计算一次局部/全局齐次变换矩阵
    Update_Homo_Trans_Matrix();
}

void Class_RNEA::Update_Homo_Trans_Matrix() {
    // 注意：调用前须先执行Init()，此时矩阵已建成4×4且基座为单位阵
    for (uint8_t i = 1 ; i<=JOINT_NUM ; i++) {
        // Joint_Parameter[i] 存放{a_i, α_i, d_i, θ_i}
        // T_i^{i-1} 使用 a_{i-1}、α_{i-1}（取上一关节的参数）和 d_i、θ_i（取本关节的参数）
        const Struct_MDH_Parameter &mdh_prev = Joint_Parameter[i - 1].MDH_Parameter;
        Struct_MDH_Parameter &mdh = Joint_Parameter[i].MDH_Parameter;

        // 关节变量与周期回调保持一致，取当前关节角
        mdh.theta = Joint_Angle_Rad[i];

        const float cos_theta = arm_cos_f32(mdh.theta);
        const float sin_theta = arm_sin_f32(mdh.theta);
        const float cos_alpha = arm_cos_f32(mdh_prev.alpha);
        const float sin_alpha = arm_sin_f32(mdh_prev.alpha);

        // 改进DH(MDH)局部齐次变换矩阵：T_i^{i-1} = Rx(α_{i-1})·Tx(a_{i-1})·Rz(θ_i)·Tz(d_i)
        //
        //        |  cθ       -sθ        0       a_{i-1}  |
        // T_i =  |  sθ·cα     cθ·cα    -sα     -d_i·sα   |
        //        |  sθ·sα     cθ·sα     cα      d_i·cα   |
        //        |  0          0        0       1        |
        //
        // 其中 a、d 的长度单位与 Mass_Center 一致(m)。
        Homo_Trans_Matrix_Local[i] = {
            { cos_theta,             -sin_theta,             0.0f,       mdh_prev.a          },
            { sin_theta * cos_alpha,  cos_theta * cos_alpha, -sin_alpha, -mdh.d * sin_alpha },
            { sin_theta * sin_alpha,  cos_theta * sin_alpha,  cos_alpha,  mdh.d * cos_alpha },
            { 0.0f,                   0.0f,                   0.0f,       1.0f               },
        };
    }

    // 全局位姿：T_0^i = T_0^{i-1}·T_i^{i-1}
    for (uint8_t i = 1 ; i<=JOINT_NUM ; i++) {
        Homo_Trans_Matrix_Global[i] = Homo_Trans_Matrix_Global[i - 1] * Homo_Trans_Matrix_Local[i];
    }
}

void Class_RNEA::TIM_RENA_Update_PeriodElapsedCallback() {
    // 1) 用最新关节角重算各连杆的局部/全局齐次变换矩阵
    Update_Homo_Trans_Matrix();

    // 2) 从末端向基座递推重力补偿（F_{N+1}=0, M_{N+1}=0）
    Class_Vector_3D force_next(0.0f, 0.0f, 0.0f);    //F_{i+1}
    Class_Vector_3D moment_next(0.0f, 0.0f, 0.0f);   //M_{i+1}
    Class_Vector_3D position_next(0.0f, 0.0f, 0.0f); //p_{i+1}

    for (int i = JOINT_NUM ; i>=1 ; i--) {
        const Class_Matrix &T = Homo_Trans_Matrix_Global[i]; //T_0^i
        const Struct_Joint_Parameter &joint = Joint_Parameter[i];

        // 关节i的旋转中心p_i（T_0^i的平移部分）与关节轴z_i（R_i的第3列）
        // 注意用的是T_0^i，不是T_0^{i-1}
        const Class_Vector_3D position(T(0, 3), T(1, 3), T(2, 3));
        const Class_Vector_3D axis(T(0, 2), T(1, 2), T(2, 2));

        // 质心在基座系下的位置：p_c,i = R_i·c_i + p_i
        const Class_Vector_3D &center_in_link = joint.Mass_Center;
        const float com_x = center_in_link.Get_X();
        const float com_y = center_in_link.Get_Y();
        const float com_z = center_in_link.Get_Z();
        const Class_Vector_3D center(T(0, 0) * com_x + T(0, 1) * com_y + T(0, 2) * com_z + position.Get_X(),
                                     T(1, 0) * com_x + T(1, 1) * com_y + T(1, 2) * com_z + position.Get_Y(),
                                     T(2, 0) * com_x + T(2, 1) * com_y + T(2, 2) * com_z + position.Get_Z());

        // 连杆i自身的重力 m_i·g
        const Class_Vector_3D gravity_force = Gravity_RNEA * joint.Link_Mass_kg;

        // F_i = m_i·g + F_{i+1}
        const Class_Vector_3D force = gravity_force + force_next;

        // M_i = (p_c,i-p_i)×(m_i·g) + (p_{i+1}-p_i)×F_{i+1} + M_{i+1}
        const Class_Vector_3D moment = (center - position).Cross(gravity_force) +
                                       (position_next - position).Cross(force_next) +
                                       moment_next;

        // τ_i^comp = -z_i·M_i：把重力矩投影到关节轴
        Joint_Gravity_Comp_Torque_Nm[i] = -axis.Dot(moment);

        // 保存连杆i的力/力矩，供下一轮（i-1）使用
        Joint_Force_N[i] = force;
        Joint_Moment_Nm[i] = moment;

        force_next = force;
        moment_next = moment;
        position_next = position;
    }
}
