#include "rbt_Joint.h"

static const float Joint_Deg_To_Rad = 3.14159265358979323846f / 180.0f;
static const float Joint_Rad_To_Deg = 180.0f / 3.14159265358979323846f;

void Class_Joint_DM_Motor::Init(FDCAN_HandleTypeDef *hcan, Enum_DM_Motor_ID can_id, float position_max, float omega_max, float torque_max)
{
    Joint_Motor_DM_J4340.Init(hcan, can_id, DM_Motor_Control_Method_MIT_POSITION, 0, position_max, omega_max, torque_max);
}

bool Class_Joint_DM_Motor::Set_Joint_FeedForward_Torque_Nm(float torque_nm)
{
    if (!std::isfinite(Joint_Angle_Gain) || Joint_Angle_Gain == 0.0f)
    {
        Joint_Motor_DM_J4340.Set_FeedForward_Torque(0.0f);
        return false;
    }
    //q_joint = gain*q_motor + offset，根据虚功关系tau_motor = gain*tau_joint。
    //电机内置减速器已包含在输出轴单位中，只处理外部传动的角度增益。
    return Joint_Motor_DM_J4340.Set_MIT_Angle_FeedForward_Torque_Nm(
        Joint_Angle_Gain * torque_nm);
}

void Class_Joint_DM_Motor::Set_Joint_Target_Angle_Deg(float target_angle_deg)
{
    if (target_angle_deg > Max_Joint_Angle_Deg)
    {
        target_angle_deg = Max_Joint_Angle_Deg;
    }
    else if (target_angle_deg < Min_Joint_Angle_Deg)
    {
        target_angle_deg = Min_Joint_Angle_Deg;
    }

    Target_Joint_Angle_Deg = target_angle_deg;
    Target_Joint_Angle_Rad = target_angle_deg * Joint_Deg_To_Rad;
    Joint_Motor_DM_J4340.Set_Target_Angle((target_angle_deg / Joint_Angle_Gain - Joint_Angle_Bias_Deg) * Joint_Deg_To_Rad);
}

void Class_Joint_DM_Motor::Set_Joint_Target_Angle_Rad(float target_angle_rad)
{
    if (target_angle_rad > Max_Joint_Angle_Rad)
    {
        target_angle_rad = Max_Joint_Angle_Rad;
    }
    else if (target_angle_rad < Min_Joint_Angle_Rad)
    {
        target_angle_rad = Min_Joint_Angle_Rad;
    }

    Target_Joint_Angle_Rad = target_angle_rad;
    Target_Joint_Angle_Deg = target_angle_rad * Joint_Rad_To_Deg;
    Joint_Motor_DM_J4340.Set_Target_Angle(target_angle_rad / Joint_Angle_Gain - Joint_Angle_Bias_Rad);
}

float Class_Joint_DM_Motor::Get_Joint_Target_Angle_Deg()
{
    return Target_Joint_Angle_Deg;
}

float Class_Joint_DM_Motor::Get_Joint_Target_Angle_Rad()
{
    return Target_Joint_Angle_Rad;
}

float Class_Joint_DM_Motor::Get_Joint_Now_Angle_Deg()
{
    return Now_Joint_Angle_Deg;
}

float Class_Joint_DM_Motor::Get_Joint_Now_Angle_Rad()
{
    return Now_Joint_Angle_Rad;
}

void Class_Joint_DM_Motor::Set_Joint_Angle_Gain(float joint_angle_gain)
{
    // 增益用于反算电机目标角度，不能为零；负值可用于反向传动。
    if (joint_angle_gain == 0.0f)
    {
        return;
    }

    Joint_Angle_Gain = joint_angle_gain;
    Joint_Motor_DM_J4340.Set_Target_Angle(Target_Joint_Angle_Rad / Joint_Angle_Gain - Joint_Angle_Bias_Rad);
}

float Class_Joint_DM_Motor::Get_Joint_Angle_Gain()
{
    return Joint_Angle_Gain;
}

void Class_Joint_DM_Motor::Set_Joint_Angle_Bias_Deg(float joint_angle_bias_deg)
{
    Joint_Angle_Bias_Deg = joint_angle_bias_deg;
    Joint_Angle_Bias_Rad = joint_angle_bias_deg * Joint_Deg_To_Rad;
    Joint_Motor_DM_J4340.Set_Target_Angle(Target_Joint_Angle_Rad / Joint_Angle_Gain - Joint_Angle_Bias_Rad);
}

void Class_Joint_DM_Motor::Set_Joint_Angle_Bias_Rad(float joint_angle_bias_rad)
{
    Joint_Angle_Bias_Rad = joint_angle_bias_rad;
    Joint_Angle_Bias_Deg = joint_angle_bias_rad * Joint_Rad_To_Deg;
    Joint_Motor_DM_J4340.Set_Target_Angle(Target_Joint_Angle_Rad / Joint_Angle_Gain - Joint_Angle_Bias_Rad);
}

float Class_Joint_DM_Motor::Get_Joint_Angle_Bias_Deg()
{
    return Joint_Angle_Bias_Deg;
}

float Class_Joint_DM_Motor::Get_Joint_Angle_Bias_Rad()
{
    return Joint_Angle_Bias_Rad;
}

void Class_Joint_DM_Motor::Set_Max_Joint_Angle_Deg(float max_joint_angle_deg)
{
    if (max_joint_angle_deg < Min_Joint_Angle_Deg)
    {
        return;
    }

    Max_Joint_Angle_Deg = max_joint_angle_deg;
    Max_Joint_Angle_Rad = max_joint_angle_deg * Joint_Deg_To_Rad;
    Set_Joint_Target_Angle_Deg(Target_Joint_Angle_Deg);
}

void Class_Joint_DM_Motor::Set_Max_Joint_Angle_Rad(float max_joint_angle_rad)
{
    if (max_joint_angle_rad < Min_Joint_Angle_Rad)
    {
        return;
    }

    Max_Joint_Angle_Rad = max_joint_angle_rad;
    Max_Joint_Angle_Deg = max_joint_angle_rad * Joint_Rad_To_Deg;
    Set_Joint_Target_Angle_Rad(Target_Joint_Angle_Rad);
}

float Class_Joint_DM_Motor::Get_Max_Joint_Angle_Deg()
{
    return Max_Joint_Angle_Deg;
}

float Class_Joint_DM_Motor::Get_Max_Joint_Angle_Rad()
{
    return Max_Joint_Angle_Rad;
}

void Class_Joint_DM_Motor::Set_Min_Joint_Angle_Deg(float min_joint_angle_deg)
{
    if (min_joint_angle_deg > Max_Joint_Angle_Deg)
    {
        return;
    }

    Min_Joint_Angle_Deg = min_joint_angle_deg;
    Min_Joint_Angle_Rad = min_joint_angle_deg * Joint_Deg_To_Rad;
    Set_Joint_Target_Angle_Deg(Target_Joint_Angle_Deg);
}

void Class_Joint_DM_Motor::Set_Min_Joint_Angle_Rad(float min_joint_angle_rad)
{
    if (min_joint_angle_rad > Max_Joint_Angle_Rad)
    {
        return;
    }

    Min_Joint_Angle_Rad = min_joint_angle_rad;
    Min_Joint_Angle_Deg = min_joint_angle_rad * Joint_Rad_To_Deg;
    Set_Joint_Target_Angle_Rad(Target_Joint_Angle_Rad);
}

float Class_Joint_DM_Motor::Get_Min_Joint_Angle_Deg()
{
    return Min_Joint_Angle_Deg;
}

float Class_Joint_DM_Motor::Get_Min_Joint_Angle_Rad()
{
    return Min_Joint_Angle_Rad;
}

void Class_Joint_DM_Gear::Init(FDCAN_HandleTypeDef *hcan, Enum_DM_Motor_ID can_id,
                                float position_max, float omega_max, float torque_max)
{
    // 内置 25:1 减速比已由电机处理，目标角度直接使用输出轴弧度。
    Joint_Motor_DM_S2325.Init(hcan, can_id, DM_Motor_Control_Method_MIT_POSITION,
                               0, position_max, omega_max, torque_max);
    FSM_Calibration.Joint = this;
    Calibration_Control_Saved = false;
    FSM_Calibration.Init(Joint_DM_Gear_Calibration_STATUS_NUMBER, Joint_DM_Gear_Calibration_IDLE);
    Calibration_Complete = false;
    Calibration_Feedback_Valid = false;
    Calibration_Last_Feedback = Joint_Motor_DM_S2325.Get_Feedback_Count();
    Calibration_Feedback_Age = 50;
}

void Class_Joint_DM_Gear::Set_Joint_Target_Angle_Deg(float target_angle_deg)
{
    if (target_angle_deg > Max_Joint_Angle_Deg)
    {
        target_angle_deg = Max_Joint_Angle_Deg;
    }
    else if (target_angle_deg < Min_Joint_Angle_Deg)
    {
        target_angle_deg = Min_Joint_Angle_Deg;
    }

    Target_Joint_Angle_Deg = target_angle_deg;
    Target_Joint_Angle_Rad = target_angle_deg * Joint_Deg_To_Rad;
    if (Get_Calibration_Complete()) Joint_Motor_DM_S2325.Set_Target_Angle((target_angle_deg / Joint_Angle_Gain + Joint_Angle_Bias_Deg) * Joint_Deg_To_Rad);
}

void Class_Joint_DM_Gear::Set_Joint_Target_Angle_Rad(float target_angle_rad)
{
    if (target_angle_rad > Max_Joint_Angle_Rad)
    {
        target_angle_rad = Max_Joint_Angle_Rad;
    }
    else if (target_angle_rad < Min_Joint_Angle_Rad)
    {
        target_angle_rad = Min_Joint_Angle_Rad;
    }

    Target_Joint_Angle_Rad = target_angle_rad;
    Target_Joint_Angle_Deg = target_angle_rad * Joint_Rad_To_Deg;
    if (Get_Calibration_Complete()) Joint_Motor_DM_S2325.Set_Target_Angle(target_angle_rad / Joint_Angle_Gain + Joint_Angle_Bias_Rad);
}

float Class_Joint_DM_Gear::Get_Joint_Target_Angle_Deg()
{
    return Target_Joint_Angle_Deg;
}

float Class_Joint_DM_Gear::Get_Joint_Target_Angle_Rad()
{
    return Target_Joint_Angle_Rad;
}

float Class_Joint_DM_Gear::Get_Joint_Now_Angle_Deg()
{
    return Now_Joint_Angle_Deg;
}

float Class_Joint_DM_Gear::Get_Joint_Now_Angle_Rad()
{
    return Now_Joint_Angle_Rad;
}

void Class_Joint_DM_Gear::Set_Joint_Angle_Gain(float joint_angle_gain)
{
    // 增益用于反算电机目标角度，不能为零；负值可用于反向传动。
    if (joint_angle_gain == 0.0f)
    {
        return;
    }

    Joint_Angle_Gain = joint_angle_gain;
    if (Get_Calibration_Complete()) Joint_Motor_DM_S2325.Set_Target_Angle(Target_Joint_Angle_Rad / Joint_Angle_Gain + Joint_Angle_Bias_Rad);
}

float Class_Joint_DM_Gear::Get_Joint_Angle_Gain()
{
    return Joint_Angle_Gain;
}

void Class_Joint_DM_Gear::Set_Joint_Angle_Bias_Deg(float joint_angle_bias_deg)
{
    Joint_Angle_Bias_Deg = joint_angle_bias_deg;
    Joint_Angle_Bias_Rad = joint_angle_bias_deg * Joint_Deg_To_Rad;
    if (Get_Calibration_Complete()) Joint_Motor_DM_S2325.Set_Target_Angle(Target_Joint_Angle_Rad / Joint_Angle_Gain + Joint_Angle_Bias_Rad);
}

void Class_Joint_DM_Gear::Set_Joint_Angle_Bias_Rad(float joint_angle_bias_rad)
{
    Joint_Angle_Bias_Rad = joint_angle_bias_rad;
    Joint_Angle_Bias_Deg = joint_angle_bias_rad * Joint_Rad_To_Deg;
    if (Get_Calibration_Complete()) Joint_Motor_DM_S2325.Set_Target_Angle(Target_Joint_Angle_Rad / Joint_Angle_Gain + Joint_Angle_Bias_Rad);
}

float Class_Joint_DM_Gear::Get_Joint_Angle_Bias_Deg()
{
    return Joint_Angle_Bias_Deg;
}

float Class_Joint_DM_Gear::Get_Joint_Angle_Bias_Rad()
{
    return Joint_Angle_Bias_Rad;
}

void Class_Joint_DM_Gear::Set_Max_Joint_Angle_Deg(float max_joint_angle_deg)
{
    if (max_joint_angle_deg < Min_Joint_Angle_Deg)
    {
        return;
    }

    Max_Joint_Angle_Deg = max_joint_angle_deg;
    Max_Joint_Angle_Rad = max_joint_angle_deg * Joint_Deg_To_Rad;
    Set_Joint_Target_Angle_Deg(Target_Joint_Angle_Deg);
}

void Class_Joint_DM_Gear::Set_Max_Joint_Angle_Rad(float max_joint_angle_rad)
{
    if (max_joint_angle_rad < Min_Joint_Angle_Rad)
    {
        return;
    }

    Max_Joint_Angle_Rad = max_joint_angle_rad;
    Max_Joint_Angle_Deg = max_joint_angle_rad * Joint_Rad_To_Deg;
    Set_Joint_Target_Angle_Rad(Target_Joint_Angle_Rad);
}

float Class_Joint_DM_Gear::Get_Max_Joint_Angle_Deg()
{
    return Max_Joint_Angle_Deg;
}

float Class_Joint_DM_Gear::Get_Max_Joint_Angle_Rad()
{
    return Max_Joint_Angle_Rad;
}

void Class_Joint_DM_Gear::Set_Min_Joint_Angle_Deg(float min_joint_angle_deg)
{
    if (min_joint_angle_deg > Max_Joint_Angle_Deg)
    {
        return;
    }

    Min_Joint_Angle_Deg = min_joint_angle_deg;
    Min_Joint_Angle_Rad = min_joint_angle_deg * Joint_Deg_To_Rad;
    Set_Joint_Target_Angle_Deg(Target_Joint_Angle_Deg);
}

void Class_Joint_DM_Gear::Set_Min_Joint_Angle_Rad(float min_joint_angle_rad)
{
    if (min_joint_angle_rad > Max_Joint_Angle_Rad)
    {
        return;
    }

    Min_Joint_Angle_Rad = min_joint_angle_rad;
    Min_Joint_Angle_Deg = min_joint_angle_rad * Joint_Rad_To_Deg;
    Set_Joint_Target_Angle_Rad(Target_Joint_Angle_Rad);
}

float Class_Joint_DM_Gear::Get_Min_Joint_Angle_Deg()
{
    return Min_Joint_Angle_Deg;
}

float Class_Joint_DM_Gear::Get_Min_Joint_Angle_Rad()
{
    return Min_Joint_Angle_Rad;
}

void Class_Joint_DM_Motor::Set_Real_Target_Torque(float real_torque_Nm) {
    Joint_Motor_DM_J4340.Set_Target_Real_Torque(real_torque_Nm);
}




bool Class_Joint_DM_Gear::Is_MIT_Control_Method()
{
    switch (Joint_Motor_DM_S2325.Get_Control_Method())
    {
    case DM_Motor_Control_Method_MIT_POSITION:
    case DM_Motor_Control_Method_MIT_OMEGA:
    case DM_Motor_Control_Method_MIT_TORQUE:
    case DM_Motor_Control_Method_MIT_Angle:
    case DM_Motor_Control_Method_MIT_OPENLOOP:
    case DM_Motor_Control_Method_MIT_IMU_Angle:
    case DM_Motor_Control_Method_MIT_Encoder_Position:
        return true;
    default:
        return false;
    }
}

bool Class_Joint_DM_Gear::Set_Calibration_Parameter(float omega, float locked_torque,
                                                  uint32_t locked_time_ms, float mit_kd)
{
    //直接采用调用方参数，不做数值或量程校验。
    Calibration_Omega = omega;
    Calibration_Locked_Torque = locked_torque;
    Calibration_Locked_Time_Ms = locked_time_ms;
    Calibration_MIT_K_D = mit_kd;
    if (FSM_Calibration.Get_Now_Status_Serial() == Joint_DM_Gear_Calibration_STALL_DETECT)
    {
        FSM_Calibration.Set_Status(Joint_DM_Gear_Calibration_ROTATE);
    }
    return true;
}

bool Class_Joint_DM_Gear::Get_Calibration_Complete()
{
    return Calibration_Complete;
}

void Class_Joint_DM_Gear::Update_Joint_Feedback()
{
    Now_Joint_Angle_Rad = Joint_Angle_Gain *
        (Joint_Motor_DM_S2325.Get_Now_Angle_Rad() - Joint_Angle_Bias_Rad);
    Now_Joint_Angle_Deg = Now_Joint_Angle_Rad * Joint_Rad_To_Deg;
}

void Class_Joint_DM_Gear::Restore_Calibration_Control()
{
    if (!Calibration_Control_Saved) return;
    Joint_Motor_DM_S2325.Set_MIT_K_P(Calibration_Previous_K_P);
    Joint_Motor_DM_S2325.Set_MIT_K_D(Calibration_Previous_K_D);
    Joint_Motor_DM_S2325.Set_DM_Motor_Control_Method(Calibration_Previous_Method);
    Joint_Motor_DM_S2325.Set_Target_Omega(0.0f);
    Joint_Motor_DM_S2325.Set_Target_Torque(0.0f);
    Joint_Motor_DM_S2325.Set_FeedForward_Torque(0.0f);
    Joint_Motor_DM_S2325.Set_Out(0.0f);
    Calibration_Control_Saved = false;
}

void Class_Joint_DM_Gear::Request_Recalibration()
{
    Restore_Calibration_Control();
    Calibration_Complete = false;
    Calibration_Feedback_Valid = false;
    Calibration_Feedback_Age = 50;
    Calibration_Last_Feedback = Joint_Motor_DM_S2325.Get_Feedback_Count();
    FSM_Calibration.Init(Joint_DM_Gear_Calibration_STATUS_NUMBER, Joint_DM_Gear_Calibration_WAIT);
}

void Class_Joint_DM_Gear::TIM_Process_PeriodElapsedCallback(bool send_control)
{
    FSM_Calibration.Reload_TIM_Status_PeriodElapsedCallback();
    Update_Joint_Feedback();
    if (!send_control || FSM_Calibration.Get_Now_Status_Serial() == Joint_DM_Gear_Calibration_IDLE ||
        FSM_Calibration.Get_Now_Status_Serial() == Joint_DM_Gear_Calibration_OFFLINE)
    {
        return;
    }
    if (Is_MIT_Control_Method() && (!Calibration_Feedback_Valid ||
        FSM_Calibration.Get_Now_Status_Serial() == Joint_DM_Gear_Calibration_WAIT ||
        FSM_Calibration.Get_Now_Status_Serial() == Joint_DM_Gear_Calibration_OFFLINE))
    {
        return;
    }
    //S2325的原生MIT模式由Process直接发帧，外部PID模式仍沿用PID/Send链路。
    const Enum_DM_Motor_Control_Method method = Joint_Motor_DM_S2325.Get_Control_Method();
    if (method == DM_Motor_Control_Method_MIT_Angle ||
        method == DM_Motor_Control_Method_MIT_OPENLOOP)
    {
        Joint_Motor_DM_S2325.TIM_PID_PeriodElapsedCallback();
        Joint_Motor_DM_S2325.TIM_Send_PeriodElapsedCallback();
    }
    else
    {
        Joint_Motor_DM_S2325.TIM_Process_PeriodElapsedCallback();
    }
}

/**
 * @brief 校准阶段的MIT速度输出，正常/疑似堵转状态共同调用。
 */
void Class_Joint_DM_Gear::Calibration_Output()
{
    Joint_Motor_DM_S2325.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_MIT_OMEGA);
    Joint_Motor_DM_S2325.Set_MIT_K_P(0.0f);
    Joint_Motor_DM_S2325.Set_MIT_K_D(Calibration_MIT_K_D);
    Joint_Motor_DM_S2325.Set_Target_Omega(Calibration_Omega);
    Joint_Motor_DM_S2325.Set_Target_Torque(0.0f);
    Joint_Motor_DM_S2325.Set_FeedForward_Torque(0.0f);
}

/**
 * @brief 关节校准状态转移；参照Chariot离线保护与Booster卡弹状态机。
 * @note 每1ms调用，疑似堵转持续时间直接使用Class_FSM的Status[].Time。
 */
void Class_FSM_Joint_DM_Gear_Calibration::Reload_TIM_Status_PeriodElapsedCallback()
{
    if (Joint == nullptr)
    {
        return;
    }
    Status[Now_Status_Serial].Time++;

    Class_DM_Motor_S2325 &motor = Joint->Joint_Motor_DM_S2325;
    const uint32_t feedback = motor.Get_Feedback_Count();
    const bool new_feedback = feedback != Joint->Calibration_Last_Feedback;
    if (new_feedback)
    {
        Joint->Calibration_Last_Feedback = feedback;
        Joint->Calibration_Feedback_Age = 0;
    }
    else if (Joint->Calibration_Feedback_Age < 50)
    {
        ++Joint->Calibration_Feedback_Age;
    }
    Joint->Calibration_Feedback_Valid = Joint->Is_MIT_Control_Method() &&
        motor.Get_DM_Motor_Status() == DM_Motor_Status_ENABLE &&
        motor.Get_DM_Motor_Control_Status() == DM_Motor_Control_Status_ENABLE &&
        motor.Get_Feedback_Enabled() && Joint->Calibration_Feedback_Age < 50 &&
        std::isfinite(motor.Get_Now_Angle_Rad()) && std::isfinite(motor.Get_Now_Torque());

    switch (Now_Status_Serial)
    {
        //上电未触发：持续观测反馈，保持空闲，不自动进入校准或离线恢复。
        case (Joint_DM_Gear_Calibration_IDLE):
        {
            break;
        }

        //等待电机上线，保存进入校准前的控制参数。
        case (Joint_DM_Gear_Calibration_WAIT):
        {
            if (!Joint->Calibration_Feedback_Valid)
            {
                Set_Status(Joint_DM_Gear_Calibration_OFFLINE);
            }
            else
            {
                Joint->Calibration_Previous_Method = motor.Get_Control_Method();
                Joint->Calibration_Previous_K_P = motor.Get_MIT_K_P();
                Joint->Calibration_Previous_K_D = motor.Get_MIT_K_D();
                Joint->Calibration_Control_Saved = true;
                Joint->Calibration_Output();
                Set_Status(Joint_DM_Gear_Calibration_ROTATE);
            }
        }
        break;

        //正常旋转，首次检测到大力矩时进入堵转嫌疑状态。
        case (Joint_DM_Gear_Calibration_ROTATE):
        {
            if (!Joint->Calibration_Feedback_Valid)
            {
                Set_Status(Joint_DM_Gear_Calibration_OFFLINE);
            }
            else
            {
                Joint->Calibration_Output();
                if (std::fabs(motor.Get_Now_Torque()) > Joint->Calibration_Locked_Torque)
                {
                    Set_Status(Joint_DM_Gear_Calibration_STALL_DETECT);
                }
            }
        }
        break;

        //疑似堵转：持续旋转，用状态计时确认；力矩回落则退出并清零计时。
        case (Joint_DM_Gear_Calibration_STALL_DETECT):
        {
            if (!Joint->Calibration_Feedback_Valid)
            {
                Set_Status(Joint_DM_Gear_Calibration_OFFLINE);
            }
            else
            {
                Joint->Calibration_Output();
                //先检查力矩回落，避免刚好达到时限的低力矩被误判为堵转。
                if (std::fabs(motor.Get_Now_Torque()) <= Joint->Calibration_Locked_Torque)
                {
                    Set_Status(Joint_DM_Gear_Calibration_ROTATE);
                }
                else if (new_feedback && Status[Now_Status_Serial].Time >= Joint->Calibration_Locked_Time_Ms)
                {
                    motor.Set_Target_Omega(0.0f);
                    Set_Status(Joint_DM_Gear_Calibration_SET_ZERO);
                }
            }
        }
        break;

        //堵转确认后的处理状态：记录偏移，停止寻零，将关节当前角和目标角置0。
        case (Joint_DM_Gear_Calibration_SET_ZERO):
        {
            if (!Joint->Calibration_Feedback_Valid)
            {
                Set_Status(Joint_DM_Gear_Calibration_OFFLINE);
            }
            else
            {
                Joint->Joint_Angle_Bias_Rad = motor.Get_Now_Angle_Rad();
                Joint->Joint_Angle_Bias_Deg = Joint->Joint_Angle_Bias_Rad * Joint_Rad_To_Deg;
                Joint->Target_Joint_Angle_Rad = 0.0f;
                Joint->Target_Joint_Angle_Deg = 0.0f;
                Joint->Restore_Calibration_Control();
                motor.Set_Target_Angle(Joint->Joint_Angle_Bias_Rad);
                Joint->Update_Joint_Feedback();
                Joint->Calibration_Complete = true;
                Set_Status(Joint_DM_Gear_Calibration_COMPLETE);
            }
        }
        break;

        //完成后交回正常角度控制；掉线时进入保护，但保留已标定的零点。
        case (Joint_DM_Gear_Calibration_COMPLETE):
        {
            if (!Joint->Calibration_Feedback_Valid)
            {
                Set_Status(Joint_DM_Gear_Calibration_OFFLINE);
            }
        }
        break;

        //离线/失能/故障/非MIT模式保护，恢复后按是否已完成校准决定去向。
        case (Joint_DM_Gear_Calibration_OFFLINE):
        {
            if (Joint->Calibration_Control_Saved)
            {
                const Enum_DM_Motor_Control_Method method = motor.Get_Control_Method();
                const bool mit = Joint->Is_MIT_Control_Method();
                Joint->Restore_Calibration_Control();
                if (!mit)
                {
                    motor.Set_DM_Motor_Control_Method(method);
                }
            }
            if (Joint->Calibration_Feedback_Valid)
            {
                if (Joint->Calibration_Complete)
                {
                    Set_Status(Joint_DM_Gear_Calibration_COMPLETE);
                }
                else
                {
                    Set_Status(Joint_DM_Gear_Calibration_WAIT);
                }
            }
        }
        break;
    }
    if (Now_Status_Serial == Joint_DM_Gear_Calibration_OFFLINE && Joint->Calibration_Control_Saved)
    {
        motor.Set_Target_Omega(0.0f);
        motor.Set_Target_Torque(0.0f);
        motor.Set_FeedForward_Torque(0.0f);
    }

}
