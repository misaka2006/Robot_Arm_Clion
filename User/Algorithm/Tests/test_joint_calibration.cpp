#include "rbt_Joint.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

//仅替换硬件接口；状态机、角度换算均链接真实rbt_Joint.cpp/alg_fsm.cpp。
static unsigned sent = 0;
void Class_DM_Motor_J4310::Init(FDCAN_HandleTypeDef *, Enum_DM_Motor_ID,
    Enum_DM_Motor_Control_Method, int32_t, float, float, float) {}

void Class_DM_Motor_S2325::Init(FDCAN_HandleTypeDef *, Enum_DM_Motor_ID,
    Enum_DM_Motor_Control_Method method, int32_t, float, float, float)
{
    DM_Motor_Control_Method = method;
    Flag = 0;
    DM_Motor_Status = DM_Motor_Status_DISABLE;
}
void Class_DM_Motor_S2325::Set_Target_Angle(float angle)
{
    Target_Angle = Target_Angle_Rad = angle;
}
void Class_DM_Motor_S2325::CAN_RxCpltCallback(uint8_t *data)
{
    float values[2];
    std::memcpy(values, data, sizeof(values));
    Data.Now_Angle_Rad = values[0];
    Data.Now_Torque = values[1];
    Data.ErrorCode = static_cast<Enum_DM_Motor_ErrorCode>(1);
    DM_Motor_Status = DM_Motor_Status_ENABLE;
    ++Flag;
}
void Class_DM_Motor_S2325::TIM_Process_PeriodElapsedCallback() { ++sent; }
void Class_DM_Motor_S2325::TIM_PID_PeriodElapsedCallback() {}
void Class_DM_Motor_S2325::TIM_Send_PeriodElapsedCallback() { ++sent; }
static void Require(bool condition, const char *message)
{
    if (!condition) { std::fprintf(stderr, "FAIL: %s\n", message); std::exit(1); }
}
static void Close(float value, float expected, const char *message)
{
    Require(std::fabs(value - expected) < 0.0001f, message);
}
static void Tick(Class_Joint_DM_Gear &joint, float angle, float torque, bool feedback = true)
{
    if (feedback)
    {
        float values[2] = {angle, torque};
        joint.Joint_Motor_DM_S2325.CAN_RxCpltCallback(reinterpret_cast<uint8_t *>(values));
    }
    joint.TIM_Process_PeriodElapsedCallback();
}
int main()
{
    Class_Joint_DM_Gear joint;
    joint.Init(nullptr, DM_Motor_ID_0xA1, 3.14f, 10.0f, 12.0f);
    auto &motor = joint.Joint_Motor_DM_S2325;
    Require(!joint.Get_Calibration_Complete(), "starts uncalibrated");
    motor.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
    const unsigned sent_before_idle = sent;
    for (int i = 0; i < 100; ++i) Tick(joint, 1, 20);
    Require(joint.FSM_Calibration.Get_Now_Status_Serial() == Joint_DM_Gear_Calibration_IDLE, "online/high torque cannot auto-start calibration");
    Require(sent == sent_before_idle, "idle sends no control frames");
    //调试器等效地将状态切到WAIT即可启动；代码接口执行完整重置。
    joint.FSM_Calibration.Set_Status(Joint_DM_Gear_Calibration_WAIT);
    Tick(joint, 1, 0);
    Require(joint.FSM_Calibration.Get_Now_Status_Serial() == Joint_DM_Gear_Calibration_ROTATE, "manual state change starts calibration");
    joint.Request_Recalibration();

    Require(joint.Set_Calibration_Parameter(0, 0.5f, 5), "zero speed accepted");
    Require(joint.Set_Calibration_Parameter(0.3f, 0.5f, 0), "zero duration accepted");
    Require(joint.Set_Calibration_Parameter(11, 12, 5, 20), "range restrictions removed");
    Require(joint.Set_Calibration_Parameter(-2, 10000, 200, 20), "user parameters accepted");
    //检查传入值确实到达MIT控制参数，不仅仅是返回true。
    joint.Joint_Motor_DM_S2325.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
    Tick(joint, 1, 0);
    Close(motor.Get_Target_Omega(), -2, "speed applied directly");
    Close(motor.Get_MIT_K_D(), 20, "kd applied directly before protocol packing");
    Tick(joint, 1, 20);
    Require(joint.FSM_Calibration.Get_Now_Status_Serial() == Joint_DM_Gear_Calibration_ROTATE, "high torque threshold applied directly");
    joint.Request_Recalibration();
    motor.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
    Require(joint.Set_Calibration_Parameter(-0.3f, 0.5f, 5, 1.2f), "test parameters restored");
    Tick(joint, 1, 0.7f);
    Require(joint.FSM_Calibration.Get_Now_Status_Serial() == Joint_DM_Gear_Calibration_OFFLINE, "disabled enters protection");
    motor.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
    motor.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_OMEGA);
    Tick(joint, 1, 0.7f);
    Require(motor.Get_Control_Method() == DM_Motor_Control_Method_OMEGA, "non MIT unchanged");
    Require(!joint.Get_Calibration_Complete(), "non MIT cannot finish");
    motor.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_MIT_POSITION);
    motor.Set_MIT_K_P(6.0f); motor.Set_MIT_K_D(0.2f);
    Tick(joint, 1, 0); //离线保护 -> 等待
    Tick(joint, 1, 0); //等待 -> 旋转
    Require(motor.Get_Control_Method() == DM_Motor_Control_Method_MIT_OMEGA, "native MIT speed mode");
    Close(motor.Get_Target_Omega(), -0.3f, "signed calibration speed");
    Close(motor.Get_MIT_K_P(), 0, "no position stiffness during rotation");
    Close(motor.Get_MIT_K_D(), 1.2f, "MIT speed damping set");
    joint.Set_Joint_Target_Angle_Rad(0.9f);
    Close(motor.Get_Target_Omega(), -0.3f, "normal target cannot override rotation");
    for (int i=0; i<4; ++i) Tick(joint, 1, -0.7f);
    Require(!joint.Get_Calibration_Complete(), "duration not yet reached");
    Tick(joint, 1, 0.5f);
    for (int i=0; i<4; ++i) Tick(joint, 1, -0.7f);
    Require(!joint.Get_Calibration_Complete(), "threshold equality resets consecutive count");
    Tick(joint, 1.25f, -0.7f);
    Require(!joint.Get_Calibration_Complete(), "FSM dwell has not yet reached five ms");
    Tick(joint, 1.25f, -0.7f);
    Require(joint.FSM_Calibration.Get_Now_Status_Serial() == Joint_DM_Gear_Calibration_SET_ZERO, "stall confirmation has its own state");
    Close(motor.Get_Target_Omega(), 0, "stop on confirmed stall");
    Tick(joint, 1.25f, -0.7f);
    Require(joint.Get_Calibration_Complete(), "negative torque magnitude completes");
    Close(joint.Get_Joint_Angle_Bias_Rad(), 1.25f, "feedback captured as positive offset");
    Close(joint.Get_Joint_Now_Angle_Rad(), 0, "calibrated feedback zero immediately");
    Close(joint.Get_Joint_Target_Angle_Rad(), 0, "joint target zero");
    Close(motor.Get_Target_Angle(), 1.25f, "raw target holds calibrated zero");
    Close(motor.Get_Target_Omega(), 0, "rotation stopped");
    Require(motor.Get_Control_Method() == DM_Motor_Control_Method_MIT_POSITION, "original mode restored");
    Close(motor.Get_MIT_K_P(), 6, "kp restored");
    Close(motor.Get_MIT_K_D(), 0.2f, "kd restored");
    Tick(joint, 1.75f, 0);
    Close(joint.Get_Joint_Now_Angle_Rad(), 0.5f, "feedback subtracts offset");
    joint.Set_Joint_Target_Angle_Rad(0.3f);
    Close(motor.Get_Target_Angle(), 1.55f, "target adds offset");
    joint.Set_Joint_Angle_Gain(-2);
    joint.Set_Joint_Target_Angle_Rad(0.4f);
    Close(motor.Get_Target_Angle(), 1.05f, "negative gain inverse consistent");
    joint.Update_Joint_Feedback();
    Close(joint.Get_Joint_Now_Angle_Rad(), -1.0f, "negative gain feedback");
    joint.Request_Recalibration();
    Tick(joint, 2, 0);
    Tick(joint, 2, 0.7f);
    for (int i=0; i<60; ++i) Tick(joint, 2, 0.7f, false);
    Require(!joint.Get_Calibration_Complete(), "stale feedback cannot finish");
    Require(joint.FSM_Calibration.Get_Now_Status_Serial() == Joint_DM_Gear_Calibration_OFFLINE, "stale feedback enters offline protection");
    Tick(joint, 2, 0);
    Tick(joint, 2, 0);
    Tick(joint, 2, 0.7f);
    motor.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);
    Tick(joint, 2, 0.7f);
    Require(motor.Get_Control_Method() == DM_Motor_Control_Method_POSITION_OMEGA, "switching out of MIT preserved");
    Require(!joint.Get_Calibration_Complete(), "mode change interrupts calibration");
    //已完成校准后离线：保护状态保留零点，重新上线恢复完成态，不重复撞限位。
    joint.Request_Recalibration();
    motor.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_MIT_POSITION);
    Require(joint.Set_Calibration_Parameter(0.3f, 0.5f, 2), "short calibration configured");
    Tick(joint, 3, 0);
    Tick(joint, 3, 0.7f);
    Tick(joint, 3, 0.7f);
    //到达持续时限的同周期力矩回落也不能完成。
    Tick(joint, 3, 0.1f);
    Require(joint.FSM_Calibration.Get_Now_Status_Serial() == Joint_DM_Gear_Calibration_ROTATE, "low torque at deadline resets FSM timer");
    Tick(joint, 3, 0.7f);
    Tick(joint, 3, 0.7f);
    Tick(joint, 3, 0.7f);
    Tick(joint, 3, 0.7f);
    Require(joint.Get_Calibration_Complete(), "recalibration complete");
    for (int i=0; i<51; ++i) Tick(joint, 3, 0, false);
    Require(joint.FSM_Calibration.Get_Now_Status_Serial() == Joint_DM_Gear_Calibration_OFFLINE, "completed joint also enters protection");
    Tick(joint, 3.2f, 0);
    Require(joint.FSM_Calibration.Get_Now_Status_Serial() == Joint_DM_Gear_Calibration_COMPLETE, "online recovers complete state");
    Close(joint.Get_Joint_Angle_Bias_Rad(), 3, "reconnection preserves calibration offset");
    Require(sent > 0, "control process invoked");
    std::puts("PASS: MIT gating, continuous stall, reset, signed torque, zero/offset, inverse target, stale feedback, restart.");
}
