//
// Created by minec on 2026/10/25.
//

/**
 * @file alg_RNEA.h
 * @author La_Pluma
 */

#ifndef ROBOT_ARM_CLION_ALG_RNEA_H
#define ROBOT_ARM_CLION_ALG_RNEA_H
#include "alg_matrix.h"

#define JOINT_NUM (6)

typedef struct {
    float a;
    float alpha;
    float d;
    float theta;
}Struct_MDH_Parameter;

typedef struct {
    Struct_MDH_Parameter MDH_Parameter;
    float Link_Mass_kg; //该连杆的质量
    Class_Vector_3D Mass_Center; //该连杆坐标下质心的位置，m
    Class_Matrix Inertia_Tensor; //惯量张量，kg*m^2，暂时不用
}Struct_Joint_Parameter;

class Class_RNEA {
public:

    void Set_Gravity_RNEA(Class_Vector_3D g);
    void Init();
    void TIM_RENA_Update_PeriodElapsedCallback();
    void Set_RNEA_Joint_Angle_Rad(uint8_t joint_id,float angle_rad);
    void Set_RNEA_Joint_Angle_Deg(uint8_t joint_id,float angle_deg);

    //各关节电机重力补偿力矩，N*m；joint_id取1..JOINT_NUM
    float Get_Joint_Gravity_Comp_Torque_Nm(uint8_t joint_id) const;
    const float *Get_Joint_Gravity_Comp_Torque_Nm() const;
    void Set_RNEA_Joint_Parameter(uint8_t joint_id,Struct_MDH_Parameter MDH_Parameter,float link_mass,Class_Vector_3D mass_center,Class_Matrix inertia_tensor);

private:
    Class_Matrix Homo_Trans_Matrix_Global[JOINT_NUM+1]; //全局的齐次变换矩阵，描述了连杆i相对基坐标系的位姿
    Class_Matrix Homo_Trans_Matrix_Local[JOINT_NUM+1]; //局部变换矩阵，描述连杆{i}相对连杆{i_1}的位姿
    Struct_Joint_Parameter Joint_Parameter[JOINT_NUM+1]; //MDH参数与惯量参数，长度单位m
    float Joint_Angle_Rad[JOINT_NUM+1] = {0.0f}; //各关节当前角度，rad


    Class_Vector_3D Joint_Moment_Nm[JOINT_NUM+1]; //关节力矩
    float Joint_Gravity_Comp_Torque_Nm[JOINT_NUM+1] = {0.0f}; //关节电机重力补偿力矩，即关节力矩投影到z轴上
    Class_Vector_3D Joint_Force_N[JOINT_NUM+1]; //关节力



    //按当前关节角重算局部与全局齐次变换矩阵
    void Update_Homo_Trans_Matrix();

private:
    Class_Vector_3D Gravity_RNEA = {0,0,0};
};

inline void Class_RNEA::Set_Gravity_RNEA(Class_Vector_3D g) {
    Gravity_RNEA = g;
}

inline float Class_RNEA::Get_Joint_Gravity_Comp_Torque_Nm(uint8_t joint_id) const {
    return (joint_id <= JOINT_NUM) ? Joint_Gravity_Comp_Torque_Nm[joint_id] : 0.0f;
}

inline const float *Class_RNEA::Get_Joint_Gravity_Comp_Torque_Nm() const {
    return Joint_Gravity_Comp_Torque_Nm;
}

inline void Class_RNEA::Set_RNEA_Joint_Angle_Deg(uint8_t joint_id, float angle_deg) {
    Joint_Angle_Rad[joint_id] = angle_deg * PI / 180;
}

inline void Class_RNEA::Set_RNEA_Joint_Angle_Rad(uint8_t joint_id, float angle_rad) {
    Joint_Angle_Rad[joint_id] = angle_rad;
}

inline void Class_RNEA::Set_RNEA_Joint_Parameter(uint8_t joint_id, Struct_MDH_Parameter MDH_Parameter, float link_mass, Class_Vector_3D mass_center, Class_Matrix inertia_tensor) {
    //joint_id取0..JOINT_NUM，0为基座那一行(a_0, α_0)，越界直接忽略
    if (joint_id > JOINT_NUM) {
        return;
    }

    Joint_Parameter[joint_id].MDH_Parameter = MDH_Parameter;
    Joint_Parameter[joint_id].Link_Mass_kg = link_mass;
    Joint_Parameter[joint_id].Mass_Center = mass_center;
    Joint_Parameter[joint_id].Inertia_Tensor = inertia_tensor;

    //theta即当前关节角，与周期回调的同步方式保持一致
    Joint_Angle_Rad[joint_id] = MDH_Parameter.theta;
}


#endif //ROBOT_ARM_CLION_ALG_RNEA_H
