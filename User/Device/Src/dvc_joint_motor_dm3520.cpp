#include "dvc_joint_motor_dm3520.h"
#include <cmath>

void Class_Joint_Motor_DM3520::Init(FDCAN_HandleTypeDef *hcan, Enum_DM_Motor_ID id,
    Enum_DM_Motor_Control_Method method, int32_t offset,
    float position_max, float omega_max, float torque_max)
{
    Initialized = false;
    if ((hcan != &hfdcan1 && hcan != &hfdcan2 && hcan != &hfdcan3) ||
        id < DM_Motor_ID_0xA1 || id > DM_Motor_ID_0xA8 ||
        !std::isfinite(position_max) || position_max <= 0.0f ||
        !std::isfinite(omega_max) || omega_max <= 0.0f ||
        !std::isfinite(torque_max) || torque_max <= 0.0f) return;
    Class_DM_Motor_J4310::Init(hcan, id, method, offset, position_max, omega_max, torque_max);
    CAN_Tx_Data = Tx_Data;
    Data = {};
    Rotor_Radian = Rotor_Omega = Rotor_Torque = Rotor_Wrapped_Radian = 0.0f;
    Have_Position = false;
    Position_Wraps = 0;
    Flag = Pre_Flag = 0;
    DM_Motor_Status = DM_Motor_Status_DISABLE;
    DM_Motor_Control_Status = DM_Motor_Control_Status_DISABLE;
    PID_Angle.Init(0.0f, 0.0f, 0.0f);
    PID_Omega.Init(0.0f, 0.0f, 0.0f);
    Initialized = true;
    Reset_Control();
    Pack_Torque();
}

bool Class_Joint_Motor_DM3520::Set_Gear_Ratio(float ratio)
{
    if (!std::isfinite(ratio) || ratio <= 0.0f) return false;
    Gear_Ratio = ratio;
    Update_Joint_Feedback();
    return true;
}

void Class_Joint_Motor_DM3520::Reset_Control()
{
    Out = Command_Torque = 0.0f;
    PID_Angle.Set_Integral_Error(0.0f);
    PID_Omega.Set_Integral_Error(0.0f);
}

void Class_Joint_Motor_DM3520::Set_Enable(bool enable)
{
    Set_DM_Control_Status(enable ? DM_Motor_Control_Status_ENABLE : DM_Motor_Control_Status_DISABLE);
    if (!enable) Reset_Control();
}

void Class_Joint_Motor_DM3520::CAN_RxCpltCallback(uint8_t *data)
{
    if (!Initialized || data == nullptr || (data[0] & 0x0f) != (CAN_ID & 0x0f)) return;
    Class_DM_Motor_J4310::CAN_RxCpltCallback(data);
    // 参考 C620：首帧只建立基准，此后根据半周期阈值判断正反向回绕。
    // 达妙位置映射为 [-PMAX,+PMAX]，其周期为 2*PMAX，不能照搬 8192 或 2*PI。
    const float wrapped = Data.Now_Angle_Rad;
    if (Have_Position)
    {
        const float delta = wrapped - Rotor_Wrapped_Radian;
        if (delta < -Position_Max) ++Position_Wraps;
        else if (delta > Position_Max) --Position_Wraps;
    }
    else
    {
        Have_Position = true;
    }
    Rotor_Wrapped_Radian = wrapped;
    Rotor_Radian = static_cast<float>(static_cast<double>(Position_Wraps) *
                                     (2.0 * Position_Max) + wrapped);
    Rotor_Omega = Data.Now_Omega_Radian;
    Rotor_Torque = Data.Now_Torque;
    Update_Joint_Feedback();
    DM_Motor_Status = DM_Motor_Status_ENABLE;
}

void Class_Joint_Motor_DM3520::Update_Joint_Feedback()
{
    Data.Now_Angle_Rad = Rotor_Radian / Gear_Ratio;
    Data.Now_Radian = Data.Now_Angle_Rad;
    Data.Now_Angle_Deg = Data.Now_Angle_Rad * 180.0f / PI;
    Data.Now_Angle = Data.Now_Angle_Deg;
    Data.Now_Omega_Radian = Rotor_Omega / Gear_Ratio;
    Data.Now_Omega_Angle = Data.Now_Omega_Radian * 180.0f / PI;
    // 理想减速箱换算（效率暂按 1），不改驱动器中的 TMAX。
    Data.Now_Torque = Rotor_Torque * Gear_Ratio;
}

void Class_Joint_Motor_DM3520::TIM_PID_PeriodElapsedCallback()
{
    if (!Initialized) return;
    if (DM_Motor_Control_Status != DM_Motor_Control_Status_ENABLE || !Get_Online() ||
        static_cast<unsigned>(Data.ErrorCode) != 1U)
    {
        Reset_Control();
        Pack_Torque();
        return;
    }
    // 仿 C620：角度外环输出 rad/s，速度内环输出归一化控制量 ±4095。
    // 接收回调已将位置/速度除以 Gear_Ratio；目标仍为关节输出轴 rad / rad/s。
    switch (DM_Motor_Control_Method)
    {
    case DM_Motor_Control_Method_MIT_Angle:
        PID_Angle.Set_Target(Target_Angle);
        PID_Angle.Set_Now(Data.Now_Angle_Rad);
        PID_Angle.TIM_Adjust_PeriodElapsedCallback();
        Target_Omega = PID_Angle.Get_Out();
        // 继续进入速度内环。
    case DM_Motor_Control_Method_MIT_OMEGA:
        PID_Omega.Set_Target(Target_Omega);
        PID_Omega.Set_Now(Data.Now_Omega_Radian);
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();
        Out = PID_Omega.Get_Out();
        break;
    case DM_Motor_Control_Method_MIT_TORQUE:
    case DM_Motor_Control_Method_MIT_OPENLOOP:
        // 开环扭矩输入是输出轴 Nm，与 C620 的开环归一化思路一致。
        Out = (Target_Torque / Gear_Ratio) / Torque_Max * Output_Max;
        break;
    default:
        Reset_Control();
        break;
    }
    if (!std::isfinite(Out)) Reset_Control();
    Pack_Torque();
}

void Class_Joint_Motor_DM3520::Pack_Torque()
{
    Limit_Out();
    // 仅保留协议输出范围 ±Output_Max，不额外限制输出轴扭矩。
    const float rotor_command = Out / Output_Max * Torque_Max;
    Command_Torque = rotor_command * Gear_Ratio;
    const uint16_t torque = Math_Float_To_Int(rotor_command, -Torque_Max, Torque_Max, 0, 4095);
    // MIT Kp=Kd=0，仅使用扭矩前馈，位置/速度字段取零附近编码。
    Tx_Data[0] = 0x7f; Tx_Data[1] = 0xff;
    Tx_Data[2] = 0x7f; Tx_Data[3] = 0xf0;
    Tx_Data[4] = 0; Tx_Data[5] = 0;
    Tx_Data[6] = (torque >> 8) & 0x0f;
    Tx_Data[7] = torque & 0xff;
}

void Class_Joint_Motor_DM3520::Send_Command(uint8_t command)
{
    if (!Initialized) return;
    uint8_t frame[8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,command};
    CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_ID, frame, 8);
}

void Class_Joint_Motor_DM3520::TIM_Send_PeriodElapsedCallback()
{
    if (!Initialized) return;
    if (DM_Motor_Control_Status != DM_Motor_Control_Status_ENABLE)
    {
        Reset_Control();
        Send_Command(0xfd);
        return;
    }
    CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_ID, Tx_Data, 8);
}

void Class_Joint_Motor_DM3520::TIM_Alive_PeriodElapsedCallback()
{
    if (!Initialized) return;
    DM_Motor_Status = Flag == Pre_Flag ? DM_Motor_Status_DISABLE : DM_Motor_Status_ENABLE;
    Pre_Flag = Flag;
    if (!Get_Online()) { Reset_Control(); Pack_Torque(); }
    Send_Command(DM_Motor_Control_Status == DM_Motor_Control_Status_ENABLE ? 0xfc : 0xfd);
}

void Class_Joint_Motor_DM3520::TIM_Calculate_PeriodElapsedCallback()
{
    TIM_PID_PeriodElapsedCallback();
    TIM_Send_PeriodElapsedCallback();
}

void Class_Joint_Motor_DM3520::Clear_Error() { Send_Command(0xfb); }
