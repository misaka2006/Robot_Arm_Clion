#include "dvc_joint_motor_dm4310.h"
#include <cmath>

bool Class_Joint_Motor_DM4310::Init(FDCAN_HandleTypeDef *hcan, uint16_t motor_id,
                                    uint16_t master_id, float position_max,
                                    float omega_max, float torque_max)
{
    Ready = false;
    Feedback_Received = false;
    Control_Changed = true;
    if ((hcan != &hfdcan1 && hcan != &hfdcan2 && hcan != &hfdcan3) ||
        motor_id == 0 || motor_id > 0x0f || master_id > 0x7ff ||
        !std::isfinite(position_max) || position_max <= 0.0f ||
        !std::isfinite(omega_max) || omega_max <= 0.0f ||
        !std::isfinite(torque_max) || torque_max <= 0.0f)
    {
        return false;
    }
    // 原 Init 只支持旧枚举的共享缓存；适配层直接绑定真实总线/ID和独立缓存。
    // 原驱动文件不变，继续复用其反馈解码和参数存储。
    CAN_Manage_Object = hcan == &hfdcan1 ? &CAN1_Manage_Object :
                        hcan == &hfdcan2 ? &CAN2_Manage_Object : &CAN3_Manage_Object;
    CAN_ID = static_cast<Enum_DM_Motor_ID>(motor_id);
    DM_Motor_Control_Method = DM_Motor_Control_Method_MIT_Angle;
    Position_Offset = 0;
    Position_Max = position_max;
    Omega_Max = omega_max;
    Torque_Max = torque_max;
    Motor_ID = motor_id;
    Master_ID = master_id;
    CAN_Tx_Data = Tx_Data;
    Data = {};
    Flag = 0;
    Pre_Flag = 0;
    DM_Motor_Status = DM_Motor_Status_DISABLE;
    DM_Motor_Control_Status = DM_Motor_Control_Status_DISABLE;
    Set_Target_Angle(0.0f);
    Set_Target_Omega(0.0f);
    Set_Target_Torque(0.0f);
    MIT_K_P = MIT_K_D = 0.0f;
    // 与 crt_robot_arm.cpp 中 Joint0~Joint3 的 PID 参数保持一致。
    PID_Angle.Init(60.0f, 0.01f, 0.0f, 0.0f, 3.0f, 10.0f,
                   0.0f, 0.0f, 0.0f, 0.001f, 0.0f, PID_D_First_ENABLE);
    PID_Omega.Init(200.0f, 0.02f, 0.0f, 0.0f, 1000.0f, 2048.0f,
                   0.0f, 0.0f, 0.0f, 0.001f, 0.0f);
    Last_Rx_Tick = HAL_GetTick();
    Last_Control_Tick = Last_Rx_Tick;
    Last_Setup_Tick = Last_Rx_Tick;
    Control_Mode = 0;
    Last_Tx_Status = 0;
    Set_Setup_State(Setup_READ_MODE);
    Ready = true;
    return true;
}

bool Class_Joint_Motor_DM4310::Get_Online() const
{
    // 原 Alive 回调使用旧 ID 发使能帧，故在适配层单独维护超时。
    if (!Get_Configuration_Ready() || !Feedback_Received)
    {
        return false;
    }
    const uint8_t status = static_cast<uint8_t>(Data.ErrorCode);
    // V1.3 的 3/4/5 也是故障；只有失能(0)和使能(1)属于正常反馈。
    return static_cast<uint32_t>(HAL_GetTick() - Last_Rx_Tick) < 100U &&
           (status == 0U || status == 1U);
}

float Class_Joint_Motor_DM4310::Get_Now_Radian() const
{
    // 使用按 P_MAX 解码的带符号位置，供 STM32 角度环使用。
    return Ready ? Data.Now_Angle_Rad : 0.0f;
}

void Class_Joint_Motor_DM4310::Set_Target_Radian(float radian)
{
    // 目标只交给 STM32 角度环；MIT 电机内部位置/速度 PD 不参与控制。
    if (Ready && std::isfinite(radian))
    {
        Set_Target_Angle(radian);
    }
}

void Class_Joint_Motor_DM4310::Set_Enable(bool enable)
{
    const Enum_DM_Motor_Control_Status status = enable && Get_Online()
        ? DM_Motor_Control_Status_ENABLE : DM_Motor_Control_Status_DISABLE;
    if (DM_Motor_Control_Status != status)
    {
        Control_Changed = true;
    }
    Set_DM_Control_Status(status);
    if (status == DM_Motor_Control_Status_DISABLE)
    {
        Target_Omega = 0.0f;
        Target_Torque = 0.0f;
        Set_Out(0.0f);
        PID_Angle.Set_Integral_Error(0.0f);
        PID_Omega.Set_Integral_Error(0.0f);
    }
}

void Class_Joint_Motor_DM4310::CAN_RxCpltCallback(Struct_CAN_Rx_Buffer *message)
{
    if (!Ready || message == nullptr || message->Header.Identifier != Get_Master_ID() ||
        message->Header.IdType != FDCAN_STANDARD_ID ||
        message->Header.RxFrameType != FDCAN_DATA_FRAME ||
        message->Header.DataLength != FDCAN_DLC_BYTES_8)
    {
        return;
    }
    // 寄存器应答也使用 MASTERID，必须在运动反馈解码之前分流。
    if (Process_Parameter_Reply(message->Data))
    {
        return;
    }
    if ((message->Data[0] & 0x0fU) != Motor_ID)
    {
        return;
    }
    Class_DM_Motor_J4310::CAN_RxCpltCallback(message->Data);
    Last_Rx_Tick = HAL_GetTick();
    Feedback_Received = true;
}

void Class_Joint_Motor_DM4310::TIM_Calculate_PeriodElapsedCallback()
{
    if (!Ready)
    {
        return;
    }
    const uint32_t now = HAL_GetTick();
    if (!Get_Configuration_Ready())
    {
        Set_Enable(false);
        DM_Motor_Status = DM_Motor_Status_DISABLE;
        Process_Setup(now);
        return;
    }
    if (!Get_Online())
    {
        Set_Enable(false);
    }
    if (DM_Motor_Control_Status == DM_Motor_Control_Status_ENABLE)
    {
        TIM_PID_PeriodElapsedCallback();
    }
    DM_Motor_Status = Get_Online() ? DM_Motor_Status_ENABLE : DM_Motor_Status_DISABLE;
    // MIT 控制帧直接使用调参助手 CANID，没有 0x100 偏移。
    const uint16_t tx_id = Motor_ID;
    // 状态切换立即发送，平时每 50 ms 重发一次使能/失能。
    if (Control_Changed || static_cast<uint32_t>(now - Last_Control_Tick) >= 50U)
    {
        memset(Tx_Data, 0xff, sizeof(Tx_Data));
        Tx_Data[7] = DM_Motor_Control_Status == DM_Motor_Control_Status_ENABLE ? 0xfc : 0xfd;
        if (!Send_Frame(tx_id, 8))
        {
            // 使能/失能未入队时下周期重试，不继续发送力矩命令。
            Control_Changed = true;
            return;
        }
        Last_Control_Tick = now;
        Control_Changed = false;
    }
    // 只有使能时才发送 STM32 PID 算出的控制帧。
    if (DM_Motor_Control_Status == DM_Motor_Control_Status_ENABLE)
    {
        // 上面的使能帧复用了 Tx_Data，发送前要重新打包 PID 输出。
        Output();
        Send_Frame(tx_id, 8);
    }
}

void Class_Joint_Motor_DM4310::TIM_PID_PeriodElapsedCallback()
{
    // 与 crt_robot_arm.h 中 DM4340 的角度环、速度环计算顺序一致。
    PID_Angle.Set_Target(Target_Angle);
    PID_Angle.Set_Now(Data.Now_Angle_Rad);
    PID_Angle.TIM_Adjust_PeriodElapsedCallback();
    Target_Omega = PID_Angle.Get_Out();

    PID_Omega.Set_Target(Target_Omega);
    PID_Omega.Set_Now(Data.Now_Omega_Radian);
    PID_Omega.TIM_Adjust_PeriodElapsedCallback();
    Target_Torque = PID_Omega.Get_Out();

    // 防止无效反馈或 PID 输出进入 CAN 打包函数。
    if (!std::isfinite(Target_Omega) || !std::isfinite(Target_Torque))
    {
        Set_Enable(false);
        return;
    }

    Target_Torque += FeedForward_Torque;

    if (PID_Polar == PID_Polar_Positive)  Target_Torque = Target_Torque;
    else Target_Torque = -Target_Torque;

    Set_Out(Target_Torque);
    Output();
}

bool Class_Joint_Motor_DM4310::Send_Frame(uint16_t id, uint16_t length)
{
    Last_Tx_Status = CAN_Send_Data(CAN_Manage_Object->CAN_Handler, id, Tx_Data, length);
    return Last_Tx_Status == 0U; // HAL_OK；入队成功不等于已经收到电机应答。
}

void Class_Joint_Motor_DM4310::Set_Setup_State(Enum_Setup_State state)
{
    Setup_State = state;
    Setup_Request_Pending = true;
}

void Class_Joint_Motor_DM4310::Process_Setup(uint32_t now)
{
    if (!Setup_Request_Pending && static_cast<uint32_t>(now - Last_Setup_Tick) < 50U)
    {
        return;
    }
    if (Setup_State == Setup_DISABLE)
    {
        // 先按电机实际模式失能，下一周期才写模式。FIFO 保证报文发送顺序。
        memset(Tx_Data, 0xff, sizeof(Tx_Data));
        Tx_Data[7] = 0xfd;
        const uint16_t id = Motor_ID + static_cast<uint16_t>((Control_Mode - 1U) * 0x100U);
        if (Send_Frame(id, 8))
        {
            Set_Setup_State(Control_Mode == 1U ? Setup_READ_PMAX : Setup_WRITE_MODE);
        }
        return;
    }

    memset(Tx_Data, 0, sizeof(Tx_Data));
    Tx_Data[0] = static_cast<uint8_t>(Motor_ID);
    Tx_Data[1] = static_cast<uint8_t>(Motor_ID >> 8);
    Tx_Data[2] = Setup_State == Setup_WRITE_MODE ? 0x55 : 0x33;
    switch (Setup_State)
    {
    case Setup_READ_MODE:
    case Setup_WRITE_MODE:
        Tx_Data[3] = 0x0a;
        Tx_Data[4] = 1; // uint32 little-endian: MIT 模式，STM32 负责位置/速度 PID。
        break;
    case Setup_READ_PMAX: Tx_Data[3] = 0x15; break;
    case Setup_READ_VMAX: Tx_Data[3] = 0x16; break;
    case Setup_READ_TMAX: Tx_Data[3] = 0x17; break;
    default: return;
    }
    // 读参数为 4 字节，写参数为 8 字节；不发送存储参数指令，不写 Flash。
    if (Send_Frame(0x7ff, Setup_State == Setup_WRITE_MODE ? 8 : 4))
    {
        Last_Setup_Tick = now;
        Setup_Request_Pending = false;
    }
}

bool Class_Joint_Motor_DM4310::Process_Parameter_Reply(const uint8_t *data)
{
    if (data[0] != static_cast<uint8_t>(Motor_ID) ||
        data[1] != static_cast<uint8_t>(Motor_ID >> 8) ||
        (data[2] != 0x33 && data[2] != 0x55))
    {
        return false;
    }

    // 未请求/迟到的寄存器应答也不能当成角度数据或刷新反馈超时。
    if (Setup_Request_Pending)
    {
        return true;
    }
    uint32_t mode;
    memcpy(&mode, data + 4, sizeof(mode));
    if (Setup_State == Setup_READ_MODE && data[2] == 0x33 && data[3] == 0x0a)
    {
        if (mode >= 1U && mode <= 4U)
        {
            Control_Mode = mode;
            Set_Setup_State(Setup_DISABLE);
        }
    }
    else if (Setup_State == Setup_WRITE_MODE && data[2] == 0x55 && data[3] == 0x0a)
    {
        if (mode == 1U)
        {
            // 写应答之后再读回确认，不把本地控制枚举视作电机已切换。
            Set_Setup_State(Setup_READ_MODE);
        }
    }
    else if (data[2] == 0x33)
    {
        float range;
        memcpy(&range, data + 4, sizeof(range));
        if (!std::isfinite(range) || range <= 0.0f)
        {
            return true;
        }
        if (Setup_State == Setup_READ_PMAX && data[3] == 0x15)
        {
            Position_Max = range;
            Set_Setup_State(Setup_READ_VMAX);
        }
        else if (Setup_State == Setup_READ_VMAX && data[3] == 0x16)
        {
            Omega_Max = range;
            Set_Setup_State(Setup_READ_TMAX);
        }
        else if (Setup_State == Setup_READ_TMAX && data[3] == 0x17)
        {
            Torque_Max = range;
            // 丢弃旧量程下的反馈，先通过失能帧查询一帧新反馈，再允许使能。
            Feedback_Received = false;
            Control_Changed = true;
            Set_Setup_State(Setup_COMPLETE);
        }
    }
    return true;
}
