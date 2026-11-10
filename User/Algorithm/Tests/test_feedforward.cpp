#include "rbt_Joint.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

//主机测试仅替换CAN硬件初始化，换算函数使用工程中的原始实现。
void Class_DM_Motor_J4310::Init(FDCAN_HandleTypeDef *, Enum_DM_Motor_ID,
    Enum_DM_Motor_Control_Method method, int32_t, float, float, float torque_max)
{
    DM_Motor_Control_Method = method;
    Torque_Max = torque_max;
    FeedForward_Torque = 0.0f;
}

//rbt_Joint.cpp同时包含Joint5，补齐其硬件接口以便主机链接；本测试不调用Joint5。
void Class_DM_Motor_S2325::Init(FDCAN_HandleTypeDef *, Enum_DM_Motor_ID,
    Enum_DM_Motor_Control_Method, int32_t, float, float, float)
{
}

void Class_DM_Motor_S2325::Set_Target_Angle(float angle)
{
    Target_Angle = angle;
}

static void Require(bool condition, const char *message)
{
    if (!condition)
    {
        std::fprintf(stderr, "FAIL: %s\n", message);
        std::exit(1);
    }
}

static void Close(float actual, float expected, const char *message)
{
    Require(std::fabs(actual - expected) < 0.001f, message);
}

int main()
{
    Class_Joint_DM_Motor joint;
    joint.Init(nullptr, DM_Motor_ID_0xA1, 3.141593f, 10.0f, 12.0f);
    //当前关节Init默认MIT_POSITION；本测试专门验证外部MIT_Angle前馈接口。
    joint.Joint_Motor_DM_J4340.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_MIT_Angle);
    Require(joint.Set_Joint_FeedForward_Torque_Nm(2.0f), "Nm input accepted");
    Close(joint.Joint_Motor_DM_J4340.Get_FeedForward_Torque(), 682.5f, "2 Nm -> 682.5 output units");

    const float gains[] = {-28.0f / 17.0f, -1.0f, 1.0f, 0.5f};
    for (unsigned i = 0; i < sizeof(gains) / sizeof(gains[0]); i++)
    {
        joint.Set_Joint_Angle_Gain(gains[i]);
        joint.Set_Joint_Angle_Bias_Deg(50.0f);
        for (int polar = 0; polar < 2; polar++)
        {
            joint.Joint_Motor_DM_J4340.Set_PID_Polar(
                polar == 0 ? PID_Polar_Positive : PID_Polar_Negative);
            for (int sign = -1; sign <= 1; sign++)
            {
                const float joint_torque = 2.0f * sign;
                Require(joint.Set_Joint_FeedForward_Torque_Nm(joint_torque), "signed joint torque accepted");
                const float raw = joint.Joint_Motor_DM_J4340.Get_FeedForward_Torque();
                //按现有PID回调和MIT帧量程反解电机力矩，再按虚功反解关节力矩。
                const float motor_torque = raw * (polar == 0 ? 1.0f : -1.0f) / 4095.0f * 12.0f;
                Close(motor_torque / gains[i], joint_torque, "joint/motor torque round trip");
            }
        }
    }

    joint.Set_Joint_Angle_Gain(1.0f);
    joint.Joint_Motor_DM_J4340.Set_PID_Polar(PID_Polar_Positive);
    Require(joint.Set_Joint_FeedForward_Torque_Nm(100.0f), "saturating positive input accepted");
    Close(joint.Joint_Motor_DM_J4340.Get_FeedForward_Torque(), 4095.0f, "positive saturation");
    Require(joint.Set_Joint_FeedForward_Torque_Nm(-100.0f), "saturating negative input accepted");
    Close(joint.Joint_Motor_DM_J4340.Get_FeedForward_Torque(), -4095.0f, "negative saturation");
    Require(!joint.Set_Joint_FeedForward_Torque_Nm(std::numeric_limits<float>::quiet_NaN()), "NaN rejected");
    Close(joint.Joint_Motor_DM_J4340.Get_FeedForward_Torque(), 0.0f, "invalid input clears previous feedforward");

    joint.Joint_Motor_DM_J4340.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_MIT_POSITION);
    Require(!joint.Set_Joint_FeedForward_Torque_Nm(2.0f), "unsupported control mode rejected");
    Close(joint.Joint_Motor_DM_J4340.Get_FeedForward_Torque(), 0.0f, "unsupported mode clears output");
    joint.Init(nullptr, DM_Motor_ID_0xA1, 3.141593f, 10.0f, 0.0f);
    Require(!joint.Set_Joint_FeedForward_Torque_Nm(2.0f), "zero TMAX rejected");

    std::puts("PASS: Nm/output-unit conversion, transmission gains, PID polarity, saturation and invalid input.");
    return 0;
}

void Class_DM_Motor_S2325::TIM_Process_PeriodElapsedCallback() {}
void Class_DM_Motor_S2325::TIM_PID_PeriodElapsedCallback() {}
void Class_DM_Motor_S2325::TIM_Send_PeriodElapsedCallback() {}
