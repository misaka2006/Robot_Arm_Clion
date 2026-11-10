/**
 * @file dvc_dmmotor.cpp
 * @author cjw by yssickjgd
 * @brief 达妙电机配置与操作
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_dmmotor.h"
#include <cmath>
#include <cstring>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// 清除电机错误信息
uint8_t DM_Motor_CAN_Message_Clear_Error[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb};
// 使能电机
uint8_t DM_Motor_CAN_Message_Enter[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
// 失能电机
uint8_t DM_Motor_CAN_Message_Exit[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
// 保存当前电机位置为零点
uint8_t DM_Motor_CAN_Message_Save_Zero[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe};

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 分配CAN发送缓冲区
 *
 * @param hcan CAN编号
 * @param __CAN_ID CAN ID
 * @return uint8_t* 缓冲区指针
 */
uint8_t *allocate_tx_data(FDCAN_HandleTypeDef *hcan, Enum_DM_Motor_ID __CAN_ID)
{
    uint8_t *tmp_tx_data_ptr = nullptr;
    if (hcan == &hfdcan1)
    {
        // 同一发送缓冲区兼容 0x01 和 0xA1 两种电机 ID 写法。
        switch (static_cast<uint8_t>(__CAN_ID) & 0x0f)
        {
        case (1):
        {
            tmp_tx_data_ptr = CAN1_0xxf1_Tx_Data;
            break;
        }
        case (2):
        {
            tmp_tx_data_ptr = CAN1_0xxf2_Tx_Data;
        }
        break;
        case (3):
        {
            tmp_tx_data_ptr = CAN1_0xxf3_Tx_Data;
        }
        break;
        case (4):
        {
            tmp_tx_data_ptr = CAN1_0xxf4_Tx_Data;
        }
        break;
        case (5):
        {
            tmp_tx_data_ptr = CAN1_0xxf5_Tx_Data;
        }
        break;
        case (6):
        {
            tmp_tx_data_ptr = CAN1_0xxf6_Tx_Data;
        }
        break;
        case (7):
        {
            tmp_tx_data_ptr = CAN1_0xxf7_Tx_Data;
        }
        break;
        case (8):
        {
            tmp_tx_data_ptr = CAN1_0xxf8_Tx_Data;
        }
        break;
        }
    }
    else if (hcan == &hfdcan2)
    {
        switch (static_cast<uint8_t>(__CAN_ID) & 0x0f)
        {
        case (1):
        {
            tmp_tx_data_ptr = CAN2_0xxf1_Tx_Data;
        }
        break;
        case (2):
        {
            tmp_tx_data_ptr = CAN2_0xxf2_Tx_Data;
        }
        break;
        case (3):
        {
            tmp_tx_data_ptr = CAN2_0xxf3_Tx_Data;
        }
        break;
        case (4):
        {
            tmp_tx_data_ptr = CAN2_0xxf4_Tx_Data;
        }
        break;
        case (5):
        {
            tmp_tx_data_ptr = CAN2_0xxf5_Tx_Data;
        }
        break;
        case (6):
        {
            tmp_tx_data_ptr = CAN2_0xxf6_Tx_Data;
        }
        break;
        case (7):
        {
            tmp_tx_data_ptr = CAN2_0xxf7_Tx_Data;
        }
        break;
        case (8):
        {
            tmp_tx_data_ptr = CAN2_0xxf8_Tx_Data;
        }
        break;
        }
    }
    else if (hcan == &hfdcan3)
    {
        switch (static_cast<uint8_t>(__CAN_ID) & 0x0f)
        {
        case (1):
        {
            tmp_tx_data_ptr = CAN3_0xxf1_Tx_Data;
        }
        break;
        case (2):
        {
            tmp_tx_data_ptr = CAN3_0xxf2_Tx_Data;
        }
        break;
        case (3):
        {
            tmp_tx_data_ptr = CAN3_0xxf3_Tx_Data;
        }
        break;
        case (4):
        {
            tmp_tx_data_ptr = CAN3_0xxf4_Tx_Data;
        }
        break;
        case (5):
        {
            tmp_tx_data_ptr = CAN3_0xxf5_Tx_Data;
        }
        break;
        case (6):
        {
            tmp_tx_data_ptr = CAN3_0xxf6_Tx_Data;
        }
        break;
        case (7):
        {
            tmp_tx_data_ptr = CAN3_0xxf7_Tx_Data;
        }
        break;
        case (8):
        {
            tmp_tx_data_ptr = CAN3_0xxf8_Tx_Data;
        }
        break;
        }
    }
    return (tmp_tx_data_ptr);
}

/**
 * @brief 电机初始化
 *
 * @param hcan 绑定的CAN总线
 * @param __CAN_ID 绑定的CAN ID
 * @param __Control_Method 电机控制方式, 默认角度
 * @param __Position_Offset 编码器偏移, 默认0, 也可以用于统一调参软件和实际下位机的位置反馈
 * @param __Positon_Max 位置反馈最大值, 调参助手设置, 即为P_MAX，一般单圈设置为3.141593, 也就是360度
 * @param __Omega_Max 最大速度, 调参助手设置
 * @param __Torque_Max 最大扭矩, 调参助手设置,推荐设置为电机的额定扭矩，例如8009P是20N·M
 */
void Class_DM_Motor_J4310::Init(FDCAN_HandleTypeDef *hcan, Enum_DM_Motor_ID __CAN_ID, Enum_DM_Motor_Control_Method __Control_Method, int32_t __Position_Offset, float __Positon_Max, float __Omega_Max, float __Torque_Max)
{
    if (hcan->Instance == FDCAN1)
    {
        CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == FDCAN2)
    {
        CAN_Manage_Object = &CAN2_Manage_Object;
    }
    else if (hcan->Instance == FDCAN3)
    {
        CAN_Manage_Object = &CAN3_Manage_Object;
    }
    CAN_ID = __CAN_ID;
    DM_Motor_Control_Method = __Control_Method;
    Position_Offset = __Position_Offset;
    Position_Max = __Positon_Max;
    Omega_Max = __Omega_Max;
    Torque_Max = __Torque_Max;
    CAN_Tx_Data = allocate_tx_data(hcan, __CAN_ID);
    switch (DM_Motor_Control_Method) {
        case DM_Motor_Control_Method_MIT_POSITION:
        case DM_Motor_Control_Method_MIT_Angle:
        case DM_Motor_Control_Method_MIT_Encoder_Position:
        case DM_Motor_Control_Method_MIT_IMU_Angle:
        case DM_Motor_Control_Method_MIT_OMEGA:
        case DM_Motor_Control_Method_MIT_OPENLOOP:
        case DM_Motor_Control_Method_MIT_TORQUE: {
            CAN_ID_Offset = 0x000;
            break;
        }
        case DM_Motor_Control_Method_POSITION_OMEGA: {
            CAN_ID_Offset = 0x100;
            break;
        }
        case DM_Motor_Control_Method_OMEGA: {
            CAN_ID_Offset = 0x200;
            break;
        }
        case DM_Motor_Control_Method_ONE_TO_FOUR:{
            CAN_ID_Offset = 0x300;
            break;
        }
    }
}

/**
 * @brief 数据处理过程
 *
 * @param  DM_Rx_Data 达妙电机源数据
 * @param  ID 表示控制器的 ID，取 CAN_ID 的低 8 位
 * @param  ERR 表示故障，对应故障类型为：8-超压，9-欠压，A-过电流，B-MOS 过温，C-电机线圈过温，D-通讯丢失，E-过载
 * @param  POS 表示电机的位置信息
 * @param  VEL 表示电机的速度信息
 * @param  T 表示电机的扭矩信息
 * @param  T_MOS 表示驱动上 MOS 的平均温度，单位℃
 * @param  T_Rotor 表示电机内部线圈的平均温度，单位℃
 *
 * |-------------------------------------------------------------------------------------------|
 * |   D[0]    |    D[1]   |   D[2]   |    D[3]   |      D[4]       |  D[5]  |  D[6] |  D[7]   |
 * |-------------------------------------------------------------------------------------------|
 * | ID|ERR<<4 | POS[15:8] | POS[7:0] | VEL[11:4] | VEL[3:0]T[11:8] | T[7:0] | T_MOS | T_Rotor |
 * |-------------------------------------------------------------------------------------------|
 */
void Class_DM_Motor_J4310::Data_Process(uint8_t *Rx_Data)
{

    int16_t delta_position = 0;
    int16_t tmp_omega, tmp_torque;
    int16_t tmp_position;
    uint16_t tmp_position_;
    uint8_t DM_Rx_Data[8];
    memcpy(DM_Rx_Data, Rx_Data, 8);

    tmp_position = (DM_Rx_Data[1] << 8) | (DM_Rx_Data[2]);
    tmp_position_ = (DM_Rx_Data[1] << 8) | DM_Rx_Data[2];
    tmp_omega = (DM_Rx_Data[3] << 4) | (DM_Rx_Data[4] >> 4);
    tmp_torque = ((DM_Rx_Data[4] & 0x0f) << 8) | (DM_Rx_Data[5]);

    Data.CAN_ID = (Enum_DM_Motor_ID)(DM_Rx_Data[0] & 0x0f);
    Data.ErrorCode = (Enum_DM_Motor_ErrorCode)(DM_Rx_Data[0] >> 4);

    // 计算圈数与总角度值
    // delta_position = (float)tmp_position / 65536.0f * 360.0f - Data.Pre_Position;

    // 计算电机本身信息
    Data.Now_Angle = (float)tmp_position / 65536.0f * 360.0f;
    Data.Now_Radian = (float)tmp_position / 65536.0f * 2.0f * PI;
    Data.Now_Angle_Rad = Math_Int_To_Float(tmp_position_, 0, (1 << 16) - 1, -Position_Max, Position_Max);
    Data.Now_Angle_Deg = Data.Now_Angle_Rad * 180.0f / PI;
    Data.Now_Encoder_Position = tmp_position_;
    Data.Now_Omega_Radian = Math_Int_To_Float(tmp_omega, 0, (1 << 12) - 1, -Omega_Max, Omega_Max);
    Data.Now_Torque = Math_Int_To_Float(tmp_torque, 0, (1 << 12) - 1, -Torque_Max, Torque_Max);
    Data.Now_MOS_Temperature = DM_Rx_Data[6];
    Data.Now_Rotor_Temperature = DM_Rx_Data[7];

    // 存储预备信息
    Data.Pre_Position = Data.Now_Angle;
}

/**
 * @brief CAN通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 */
void Class_DM_Motor_J4310::CAN_RxCpltCallback(uint8_t *Rx_Data)
{
    // 滑动窗口, 判断电机是否在线
    this->Flag += 1;

    Data_Process(Rx_Data);
}

/**
 * @brief TIM定时器中断定期检测电机是否存活
 *
 */
void Class_DM_Motor_J4310::TIM_Alive_PeriodElapsedCallback()
{
    // 判断该时间段内是否接收过电机数据
    if (Flag == Pre_Flag)
    {
        // 电机断开连接
        DM_Motor_Status = DM_Motor_Status_DISABLE;
    }
    else
    {
        // 电机保持连接
        DM_Motor_Status = DM_Motor_Status_ENABLE;
    }
    // 控制电机使能或失能
    switch (DM_Motor_Control_Status)
    {
    case (DM_Motor_Control_Status_DISABLE):
    {
        switch (DM_Motor_Control_Method)
        {
        case (DM_Motor_Control_Method_MIT_POSITION):
        {
            CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_ID, DM_Motor_CAN_Message_Exit, 8);
        }
        break;
        case (DM_Motor_Control_Method_MIT_OMEGA):
        {
            CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_ID, DM_Motor_CAN_Message_Exit, 8);
        }
        break;
        case (DM_Motor_Control_Method_MIT_TORQUE):
        {
            CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_ID, DM_Motor_CAN_Message_Exit, 8);
        }
        break;
        case (DM_Motor_Control_Method_POSITION_OMEGA):
        {
            CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_ID, DM_Motor_CAN_Message_Exit, 8);
            //    CAN_Send_Data(CAN_Manage_Object->CAN_Handler, static_cast<Enum_DM_Motor_ID>(CAN_ID) + 0x100, DM_Motor_CAN_Message_Exit, 8);
        }
        break;
        case (DM_Motor_Control_Method_OMEGA):
        {
            CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_ID, DM_Motor_CAN_Message_Exit, 8);
        }
        break;
        case (DM_Motor_Control_Method_MIT_IMU_Angle):
        {
            CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_ID, DM_Motor_CAN_Message_Exit, 8);
        }
        break;
        case (DM_Motor_Control_Method_MIT_Angle):
        {
            // DM4340 的 MIT 控制帧和失能帧均使用电机 CAN ID。
            CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_ID, DM_Motor_CAN_Message_Exit, 8);
        }
        break;
        case (DM_Motor_Control_Method_MIT_Encoder_Position):
        {
            CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_ID, DM_Motor_CAN_Message_Exit, 8);
        }
        break;
        }
    }
    break;
        case (DM_Motor_Control_Status_ENABLE):
        {
            switch (DM_Motor_Control_Method)
            {
                case (DM_Motor_Control_Method_MIT_POSITION):
                {
                    CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_ID, DM_Motor_CAN_Message_Enter, 8);
                }
                    break;
                case (DM_Motor_Control_Method_MIT_OMEGA):
                {
                    CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_ID, DM_Motor_CAN_Message_Enter, 8);
                }
                    break;
                case (DM_Motor_Control_Method_MIT_TORQUE):
                {
                    CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_ID, DM_Motor_CAN_Message_Enter, 8);
                }
                    break;
                case (DM_Motor_Control_Method_POSITION_OMEGA):
                {
                    CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_ID, DM_Motor_CAN_Message_Enter, 8);
                }
                    break;
                case (DM_Motor_Control_Method_OMEGA):
                {
                    CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_ID, DM_Motor_CAN_Message_Enter, 8);
                }
                    break;
                case (DM_Motor_Control_Method_MIT_IMU_Angle):
                {
                    CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_ID, DM_Motor_CAN_Message_Enter, 8);
                }
                    break;
                case (DM_Motor_Control_Method_MIT_Angle):
                {
                    CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_ID, DM_Motor_CAN_Message_Enter, 8);
                }
                    break;
                case (DM_Motor_Control_Method_MIT_Encoder_Position):
                {
                    CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_ID, DM_Motor_CAN_Message_Enter, 8);
                }
                    break;
            }
        }
    break;
    }

    Pre_Flag = Flag;
}

/**
 * @brief TIM定时器中断发送出去的回调函数
 *
 */
void Class_DM_Motor_J4310::TIM_Process_PeriodElapsedCallback()
{
    switch (DM_Motor_Control_Method)
    {
    case (DM_Motor_Control_Method_MIT_POSITION):
    {
        uint16_t tmp_position = Math_Float_To_Int(Target_Angle, -Position_Max, Position_Max, 0, (1 << 16) - 1);
        uint16_t tmp_velocity = Math_Float_To_Int(Target_Omega, -Omega_Max, Omega_Max, 0, (1 << 12) - 1);
        uint16_t tmp_k_p = Math_Float_To_Int(MIT_K_P, 0.0f, 500.0f, 0, (1 << 12) - 1);
        uint16_t tmp_k_d = Math_Float_To_Int(MIT_K_D, 0.0f, 5.0f, 0, (1 << 12) - 1);
        uint16_t tmp_torque = Math_Float_To_Int(Target_Torque, -Torque_Max, Torque_Max, 0, (1 << 12) - 1);

        Math_Endian_Reverse_16(&tmp_position);
        memcpy(&CAN_Tx_Data[0], &tmp_position, sizeof(uint16_t));

        uint8_t tmp_velocity_11_4 = tmp_velocity >> 4;
        memcpy(&CAN_Tx_Data[2], &tmp_velocity_11_4, sizeof(uint8_t));

        uint8_t tmp_velocity_3_0_k_p_11_8 = ((tmp_velocity & 0x0f) << 4) | (tmp_k_p >> 8);
        memcpy(&CAN_Tx_Data[3], &tmp_velocity_3_0_k_p_11_8, sizeof(uint8_t));

        uint8_t tmp_k_p_7_0 = tmp_k_p;
        memcpy(&CAN_Tx_Data[4], &tmp_k_p_7_0, sizeof(uint8_t));

        uint8_t tmp_k_d_11_4 = tmp_k_d >> 4;
        memcpy(&CAN_Tx_Data[5], &tmp_k_d_11_4, sizeof(uint8_t));

        uint8_t tmp_k_d_3_0_torque_11_8 = ((tmp_k_d & 0x0f) << 4) | (tmp_torque >> 8);
        memcpy(&CAN_Tx_Data[6], &tmp_k_d_3_0_torque_11_8, sizeof(uint8_t));

        uint8_t tmp_torque_7_0 = tmp_torque;
        memcpy(&CAN_Tx_Data[7], &tmp_torque_7_0, sizeof(uint8_t));

        CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_ID + CAN_ID_Offset, CAN_Tx_Data, 8);
    }
    break;
    case (DM_Motor_Control_Method_MIT_OMEGA):
    {
        uint16_t tmp_position = Math_Float_To_Int(Target_Angle, -Position_Max, Position_Max, 0, (1 << 16) - 1);
        uint16_t tmp_velocity = Math_Float_To_Int(Target_Omega, -Omega_Max, Omega_Max, 0, (1 << 12) - 1);
        uint16_t tmp_k_p = 0;
        uint16_t tmp_k_d = Math_Float_To_Int(MIT_K_D, 0.0f, 5.0f, 0, (1 << 12) - 1);
        uint16_t tmp_torque = Math_Float_To_Int(Target_Torque, -Torque_Max, Torque_Max, 0, (1 << 12) - 1);

        Math_Endian_Reverse_16(&tmp_position);
        memcpy(&CAN_Tx_Data[0], &tmp_position, sizeof(uint16_t));

        uint8_t tmp_velocity_11_4 = tmp_velocity >> 4;
        memcpy(&CAN_Tx_Data[2], &tmp_velocity_11_4, sizeof(uint8_t));

        uint8_t tmp_velocity_3_0_k_p_11_8 = ((tmp_velocity & 0x0f) << 4) | (tmp_k_p >> 8);
        memcpy(&CAN_Tx_Data[3], &tmp_velocity_3_0_k_p_11_8, sizeof(uint8_t));

        uint8_t tmp_k_p_7_0 = tmp_k_p;
        memcpy(&CAN_Tx_Data[4], &tmp_k_p_7_0, sizeof(uint8_t));

        uint8_t tmp_k_d_11_4 = tmp_k_d >> 4;
        memcpy(&CAN_Tx_Data[5], &tmp_k_d_11_4, sizeof(uint8_t));

        uint8_t tmp_k_d_3_0_torque_11_8 = ((tmp_k_d & 0x0f) << 4) | (tmp_torque >> 8);
        memcpy(&CAN_Tx_Data[6], &tmp_k_d_3_0_torque_11_8, sizeof(uint8_t));

        uint8_t tmp_torque_7_0 = tmp_torque;
        memcpy(&CAN_Tx_Data[7], &tmp_torque_7_0, sizeof(uint8_t));

        CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_ID + CAN_ID_Offset, CAN_Tx_Data, 8);
    }
    break;
    case (DM_Motor_Control_Method_MIT_TORQUE):
    {
        uint16_t tmp_position = Math_Float_To_Int(Target_Angle, -Position_Max, Position_Max, 0, (1 << 16) - 1);
        uint16_t tmp_velocity = Math_Float_To_Int(Target_Omega, -Omega_Max, Omega_Max, 0, (1 << 12) - 1);
        uint16_t tmp_k_p = 0;
        uint16_t tmp_k_d = 0;
        uint16_t tmp_torque = Math_Float_To_Int(Target_Torque, -Torque_Max, Torque_Max, 0, (1 << 12) - 1);

        Math_Endian_Reverse_16(&tmp_position);
        memcpy(&CAN_Tx_Data[0], &tmp_position, sizeof(uint16_t));

        uint8_t tmp_velocity_11_4 = tmp_velocity >> 4;
        memcpy(&CAN_Tx_Data[2], &tmp_velocity_11_4, sizeof(uint8_t));

        uint8_t tmp_velocity_3_0_k_p_11_8 = ((tmp_velocity & 0x0f) << 4) | (tmp_k_p >> 8);
        memcpy(&CAN_Tx_Data[3], &tmp_velocity_3_0_k_p_11_8, sizeof(uint8_t));

        uint8_t tmp_k_p_7_0 = tmp_k_p;
        memcpy(&CAN_Tx_Data[4], &tmp_k_p_7_0, sizeof(uint8_t));

        uint8_t tmp_k_d_11_4 = tmp_k_d >> 4;
        memcpy(&CAN_Tx_Data[5], &tmp_k_d_11_4, sizeof(uint8_t));

        uint8_t tmp_k_d_3_0_torque_11_8 = ((tmp_k_d & 0x0f) << 4) | (tmp_torque >> 8);
        memcpy(&CAN_Tx_Data[6], &tmp_k_d_3_0_torque_11_8, sizeof(uint8_t));

        uint8_t tmp_torque_7_0 = tmp_torque;
        memcpy(&CAN_Tx_Data[7], &tmp_torque_7_0, sizeof(uint8_t));

        CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_ID + CAN_ID_Offset, CAN_Tx_Data, 8);
    }
    break;
    case (DM_Motor_Control_Method_POSITION_OMEGA):
    {
        memcpy(&CAN_Tx_Data[0], &Target_Angle, sizeof(float));

        memcpy(&CAN_Tx_Data[4], &Target_Omega, sizeof(float));

        CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_ID + CAN_ID_Offset, CAN_Tx_Data, 8);
    }
    break;
    case (DM_Motor_Control_Method_OMEGA):
    {
        memcpy(&CAN_Tx_Data[0], &Target_Omega, sizeof(float));

        CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_ID + CAN_ID_Offset, CAN_Tx_Data, 4);
    }
    break;
    default:
    {
    }
    break;
    }
}

/**
 * @brief 输出函数
 *
 */
uint16_t test_data;
void Class_DM_Motor_J4310::Output()
{
    switch (DM_Motor_Control_Method)
    {
    case (DM_Motor_Control_Method_ONE_TO_FOUR):
    {
        CAN_Tx_Data[0] = (int16_t)Out >> 8;
        CAN_Tx_Data[1] = (int16_t)Out;
    }
    break;

    case (DM_Motor_Control_Method_MIT_IMU_Angle):
    {
        Limit_Out();

        uint16_t tmp_out = Math_Float_To_Int(Out, -Output_Max, Output_Max, 0, (1 << 12) - 1);

        uint8_t tmp_torque_6 = (tmp_out >> 8) & 0x0F;
        memcpy(&CAN_Tx_Data[6], &tmp_torque_6, sizeof(uint8_t));

        uint8_t tmp_torque_7 = tmp_out & 0xFF;
        memcpy(&CAN_Tx_Data[7], &tmp_torque_7, sizeof(uint8_t));

        uint8_t tmp_torque[6] = {0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00};
        for (uint8_t i = 0; i < 6; i++)
        {
            memcpy(&CAN_Tx_Data[i], &tmp_torque[i], sizeof(uint8_t));
        }
    }
    break;

    case (DM_Motor_Control_Method_MIT_OPENLOOP):
    {
        Limit_Out();

        uint16_t tmp_out = Math_Float_To_Int(Out, -Output_Max, Output_Max, 0, (1 << 12) - 1);

        uint8_t tmp_torque_6 = (tmp_out >> 8) & 0x0F;
        memcpy(&CAN_Tx_Data[6], &tmp_torque_6, sizeof(uint8_t));

        uint8_t tmp_torque_7 = tmp_out & 0xFF;
        memcpy(&CAN_Tx_Data[7], &tmp_torque_7, sizeof(uint8_t));

        uint8_t tmp_torque[6] = {0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00};
        for (uint8_t i = 0; i < 6; i++)
        {
            memcpy(&CAN_Tx_Data[i], &tmp_torque[i], sizeof(uint8_t));
        }
    }
    break;

    case (DM_Motor_Control_Method_MIT_Angle):
    {
        Limit_Out();

        uint16_t tmp_out = Math_Float_To_Int(Out, -Output_Max, Output_Max, 0, (1 << 12) - 1);

        uint8_t tmp_torque_6 = (tmp_out >> 8) & 0x0F;
        memcpy(&CAN_Tx_Data[6], &tmp_torque_6, sizeof(uint8_t));

        uint8_t tmp_torque_7 = tmp_out & 0xFF;
        memcpy(&CAN_Tx_Data[7], &tmp_torque_7, sizeof(uint8_t));

        uint8_t tmp_torque[6] = {0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00};
        for (uint8_t i = 0; i < 6; i++)
        {
            memcpy(&CAN_Tx_Data[i], &tmp_torque[i], sizeof(uint8_t));
        }
    }
    break;

    case (DM_Motor_Control_Method_MIT_Encoder_Position):
    {
        Limit_Out();

        uint16_t tmp_out = Math_Float_To_Int(Out, -Output_Max, Output_Max, 0, (1 << 12) - 1);

        uint8_t tmp_torque_6 = (tmp_out >> 8) & 0x0F;
        memcpy(&CAN_Tx_Data[6], &tmp_torque_6, sizeof(uint8_t));

        uint8_t tmp_torque_7 = tmp_out & 0xFF;
        memcpy(&CAN_Tx_Data[7], &tmp_torque_7, sizeof(uint8_t));

        uint8_t tmp_torque[6] = {0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00};
        for (uint8_t i = 0; i < 6; i++)
        {
            memcpy(&CAN_Tx_Data[i], &tmp_torque[i], sizeof(uint8_t));
        }
    }
    break;
    }
}

void Class_DM_Motor_J4310::Clear_Error()
{
    switch (DM_Motor_Control_Method)
    {
    case (DM_Motor_Control_Method_MIT_POSITION):
    {
        CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_ID, DM_Motor_CAN_Message_Clear_Error, 8);
    }
    break;
    case (DM_Motor_Control_Method_MIT_OMEGA):
    {
        CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_ID, DM_Motor_CAN_Message_Clear_Error, 8);
    }
    break;
    case (DM_Motor_Control_Method_MIT_TORQUE):
    {
        CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_ID, DM_Motor_CAN_Message_Clear_Error, 8);
    }
    break;
    case (DM_Motor_Control_Method_POSITION_OMEGA):
    {
        CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_ID, DM_Motor_CAN_Message_Clear_Error, 8);
        //    CAN_Send_Data(CAN_Manage_Object->CAN_Handler, static_cast<Enum_DM_Motor_ID>(CAN_ID) + 0x100, DM_Motor_CAN_Message_Clear_Error, 8);
    }
    break;
    case (DM_Motor_Control_Method_OMEGA):
    {
        CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_ID, DM_Motor_CAN_Message_Clear_Error, 8);
    }
    break;
    case (DM_Motor_Control_Method_MIT_IMU_Angle):
    {
        CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_ID, DM_Motor_CAN_Message_Clear_Error, 8);
    }
    break;
    case (DM_Motor_Control_Method_MIT_Angle):
    {
        CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_ID, DM_Motor_CAN_Message_Clear_Error, 8);
    }
    break;
    case (DM_Motor_Control_Method_MIT_Encoder_Position):
    {
        CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_ID, DM_Motor_CAN_Message_Clear_Error, 8);
    }
    break;
    }
}
void Class_DM_Motor_J4310::Set_Gear_Ratio(float gear_ratio)
{
    Gear_Ratio = gear_ratio;
}

void Class_DM_Motor_J4310::TIM_PID_PeriodElapsedCallback()
{
    switch (DM_Motor_Control_Method)
    {
    case (DM_Motor_Control_Method_MIT_Angle):
    {
        // 角度环：使用电机编码器反馈（带符号弧度，映射 ±Position_Max）
        PID_Angle.Set_Target(Target_Angle);
        PID_Angle.Set_Now(Data.Now_Angle_Rad);
        PID_Angle.TIM_Adjust_PeriodElapsedCallback();

        Target_Omega = PID_Angle.Get_Out();

        // 速度环：使用编码器差分角速度（rad/s，映射 ±Omega_Max）
        PID_Omega.Set_Target(Target_Omega);
        PID_Omega.Set_Now(Data.Now_Omega_Radian);
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();
        Target_Torque += FeedForward_Torque;
        Target_Torque = (PID_Polar == PID_Polar_Positive ? Target_Torque : -Target_Torque);
        Set_Out(Target_Torque);
    }
    break;

    case (DM_Motor_Control_Method_ONE_TO_FOUR):
    {
        // PID_Angle.Set_Target(Target_Angle);
        // if (IMU->Get_IMU_Status() != IMU_Status_DISABLE)
        // {
        //     // 角度环
        //     PID_Angle.Set_Now(-True_Angle_Pitch); // 在R超前的情况下
        //     PID_Angle.TIM_Adjust_PeriodElapsedCallback();

        //     Target_Omega_Angle = PID_Angle.Get_Out();

        //     // 速度环
        //     PID_Omega.Set_Target(Target_Omega_Angle);
        //     PID_Omega.Set_Now(-True_Gyro_Pitch * 57.3);
        // }
        // else
        // {
        //     PID_Angle.Set_Now(Data.Now_Angle);
        //     PID_Angle.TIM_Adjust_PeriodElapsedCallback();

        //     Target_Omega_Angle = PID_Angle.Get_Out();

        //     // 速度环
        //     PID_Omega.Set_Target(Target_Omega_Angle);
        //     PID_Omega.Set_Now(Data.Now_Omega_Angle);
        // }

        // PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        // Target_Torque = PID_Omega.Get_Out();

        // Set_Out(Target_Torque); // 补偿重力
    }
    break;
    case (DM_Motor_Control_Method_MIT_OPENLOOP):
    {
        Out = Out;
    }
    default:
    {
        Set_Out(0.0);
    }
    break;
    }
    Output(); // 进入父类中进行输出（打包进 CAN_Tx_Data，不发送）
}

void Class_DM_Motor_J4310::TIM_Send_PeriodElapsedCallback()
{
    // 发送时机由任务层的定时回调调度。
    // MIT 模式：控制帧 ID 低字节须等于电机 CAN ID（手册低8位校验，高3位忽略），直接使用 CAN_ID 发送
    if (DM_Motor_Control_Method == DM_Motor_Control_Method_MIT_Angle)
    {
        CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_ID + CAN_ID_Offset, CAN_Tx_Data, 8);
    }
}

// S2325 的协议量程由驱动器中的 PMAX、VMAX、TMAX 决定。
static const float S2325_Feedback_Gear_Ratio = 25.0f;
// 先限幅再转换，避免 Math_Float_To_Int 在越界时产生无效的 12/16 位编码。
static float S2325_Limit(float value, float minimum, float maximum)
{
    if (!std::isfinite(value)) return 0.0f;
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

void Class_DM_Motor_S2325::Init(FDCAN_HandleTypeDef *hcan, Enum_DM_Motor_ID id,
                                Enum_DM_Motor_Control_Method method, int32_t position_offset,
                                float position_max, float omega_max, float torque_max)
{
    Initialized = false;
    if ((hcan != &hfdcan1 && hcan != &hfdcan2 && hcan != &hfdcan3) ||
        id == DM_Motor_ID_UNDEFINED ||
        !std::isfinite(position_max) || position_max <= 0.0f ||
        !std::isfinite(omega_max) || omega_max <= 0.0f ||
        !std::isfinite(torque_max) || torque_max <= 0.0f) return;

    Class_DM_Motor_J4310::Init(hcan, id, method, position_offset,
                               position_max, omega_max, torque_max);
    // 与 J4310 的共享发送数组隔离；S2325 的 CAN ID 可以是驱动器配置的任意非零低字节。
    CAN_Tx_Data = Tx_Data;
    std::memset(Tx_Data, 0, sizeof(Tx_Data));
    Data = {};
    Has_Position_Feedback = false;
    Previous_Encoder_Position = 0;
    Flag = Pre_Flag = 0;
    Out = Target_Angle = Target_Angle_Rad = Target_Angle_Deg = 0.0f;
    Target_Omega = Target_Torque = FeedForward_Torque = 0.0f;
    DM_Motor_Status = DM_Motor_Status_DISABLE;
    DM_Motor_Control_Status = DM_Motor_Control_Status_DISABLE;
    Initialized = true;
    Pack_MIT_Frame(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

void Class_DM_Motor_S2325::Set_Target_Angle(float target_angle_rad)
{
    Target_Angle = Target_Angle_Rad = target_angle_rad;
    Target_Angle_Deg = target_angle_rad * 180.0f / PI;
}

void Class_DM_Motor_S2325::Set_Target_Angle_DEG(float target_angle_deg)
{
    Target_Angle_Deg = target_angle_deg;
    Target_Angle = Target_Angle_Rad = target_angle_deg * PI / 180.0f;
}

float Class_DM_Motor_S2325::Get_Target_Angle_Rad()
{
    return Target_Angle;
}

float Class_DM_Motor_S2325::Get_Target_Angle_Deg()
{
    return Target_Angle * 180.0f / PI;
}

void Class_DM_Motor_S2325::CAN_RxCpltCallback(uint8_t *rx_data)
{
    if (!Initialized || rx_data == nullptr ||
        (rx_data[0] & 0x0f) != (static_cast<uint8_t>(CAN_ID) & 0x0f)) return;

    // 基类先按驱动器量程解码原始反馈；位置与速度仍是电机轴数值。
    Class_DM_Motor_J4310::CAN_RxCpltCallback(rx_data);
    const uint16_t encoder_position = Data.Now_Encoder_Position;
    if (Has_Position_Feedback)
    {
        // 与 M2006 一样，通过半圈阈值识别正反方向的回绕。
        // S2325 的反馈位置是 16 位，差值需用 int32_t 才能容纳 ±65535。
        const int32_t delta_position = static_cast<int32_t>(encoder_position) -
                                       static_cast<int32_t>(Previous_Encoder_Position);
        if (delta_position < -32768)
        {
            ++Data.Total_Round;
        }
        else if (delta_position > 32768)
        {
            --Data.Total_Round;
        }
    }
    Has_Position_Feedback = true;
    Previous_Encoder_Position = encoder_position;
    Data.Pre_Position = encoder_position;
    Data.Pre_Total_Position = Data.Total_Position;
    Data.Total_Position = Data.Total_Round * 65536 + encoder_position;

    // 先累计电机轴多圈角度，再换算为减速后的输出轴角度和角速度。
    Data.Now_Angle_Rad = (Data.Now_Angle_Rad +
                          static_cast<float>(Data.Total_Round) * 2.0f * Position_Max) /
                         S2325_Feedback_Gear_Ratio;
    Data.Now_Angle_Deg = Data.Now_Angle_Rad * 180.0f / PI;
    Data.Now_Radian = Data.Now_Angle_Rad;
    Data.Now_Angle = Data.Now_Angle_Deg;
    Data.Now_Omega_Radian /= S2325_Feedback_Gear_Ratio;
    Data.Now_Omega_Angle = Data.Now_Omega_Radian * 180.0f / PI;
}

void Class_DM_Motor_S2325::TIM_Alive_PeriodElapsedCallback()
{
    if (!Initialized) return;
    DM_Motor_Status = Flag == Pre_Flag ? DM_Motor_Status_DISABLE : DM_Motor_Status_ENABLE;
    Pre_Flag = Flag;

    // 使能、失能及清错帧都发往电机 CAN ID，与控制模式无关。
    uint8_t *command = DM_Motor_Control_Status == DM_Motor_Control_Status_ENABLE
                           ? DM_Motor_CAN_Message_Enter : DM_Motor_CAN_Message_Exit;
    CAN_Send_Data(CAN_Manage_Object->CAN_Handler, static_cast<uint8_t>(CAN_ID), command, 8);
}

void Class_DM_Motor_S2325::Pack_MIT_Frame(float position, float velocity,
                                           float kp, float kd, float torque)
{
    if (!Initialized) return;
    // 用户实测修正：目标位置直接是输出轴目标 rad，绝不乘内置减速比 25。
    const uint16_t p = Math_Float_To_Int(S2325_Limit(position, -Position_Max, Position_Max),
                                          -Position_Max, Position_Max, 0, 65535);
    const uint16_t v = Math_Float_To_Int(S2325_Limit(velocity, -Omega_Max, Omega_Max),
                                          -Omega_Max, Omega_Max, 0, 4095);
    const uint16_t k_p = Math_Float_To_Int(S2325_Limit(kp, 0.0f, 500.0f),
                                            0.0f, 500.0f, 0, 4095);
    const uint16_t k_d = Math_Float_To_Int(S2325_Limit(kd, 0.0f, 5.0f),
                                            0.0f, 5.0f, 0, 4095);
    const uint16_t t = Math_Float_To_Int(S2325_Limit(torque, -Torque_Max, Torque_Max),
                                          -Torque_Max, Torque_Max, 0, 4095);

    Tx_Data[0] = static_cast<uint8_t>(p >> 8);
    Tx_Data[1] = static_cast<uint8_t>(p);
    Tx_Data[2] = static_cast<uint8_t>(v >> 4);
    Tx_Data[3] = static_cast<uint8_t>(((v & 0x0f) << 4) | (k_p >> 8));
    Tx_Data[4] = static_cast<uint8_t>(k_p);
    Tx_Data[5] = static_cast<uint8_t>(k_d >> 4);
    Tx_Data[6] = static_cast<uint8_t>(((k_d & 0x0f) << 4) | (t >> 8));
    Tx_Data[7] = static_cast<uint8_t>(t);
}

void Class_DM_Motor_S2325::TIM_Process_PeriodElapsedCallback()
{
    if (!Initialized || DM_Motor_Control_Status != DM_Motor_Control_Status_ENABLE ||
        DM_Motor_Status != DM_Motor_Status_ENABLE || Data.ErrorCode != 1) return;

    const uint16_t id = static_cast<uint8_t>(CAN_ID);
    switch (DM_Motor_Control_Method)
    {
    case DM_Motor_Control_Method_MIT_POSITION:
        Pack_MIT_Frame(Target_Angle, Target_Omega, MIT_K_P, MIT_K_D,
                       Target_Torque + FeedForward_Torque);
        CAN_Send_Data(CAN_Manage_Object->CAN_Handler, id, Tx_Data, 8);
        break;
    case DM_Motor_Control_Method_MIT_OMEGA:
        Pack_MIT_Frame(0.0f, Target_Omega, 0.0f, MIT_K_D,
                       Target_Torque + FeedForward_Torque);
        CAN_Send_Data(CAN_Manage_Object->CAN_Handler, id, Tx_Data, 8);
        break;
    case DM_Motor_Control_Method_MIT_TORQUE:
        Pack_MIT_Frame(0.0f, 0.0f, 0.0f, 0.0f,
                       Target_Torque + FeedForward_Torque);
        CAN_Send_Data(CAN_Manage_Object->CAN_Handler, id, Tx_Data, 8);
        break;
    case DM_Motor_Control_Method_POSITION_OMEGA:
    {
        if (!std::isfinite(Target_Angle) || !std::isfinite(Target_Omega)) return;
        // 手册示例：3.14159 rad、10 rad/s -> CF 0F 49 40 00 00 20 42。
        // STM32H723 为小端 IEEE754 float，直接复制即低字节先发。
        const float position = Target_Angle; // 不乘内置 25:1 减速比
        const float max_velocity = Target_Omega;
        std::memcpy(&Tx_Data[0], &position, sizeof(float));
        std::memcpy(&Tx_Data[4], &max_velocity, sizeof(float));
        CAN_Send_Data(CAN_Manage_Object->CAN_Handler, 0x100 + id, Tx_Data, 8);
        break;
    }
    case DM_Motor_Control_Method_OMEGA:
        if (!std::isfinite(Target_Omega)) return;
        std::memcpy(&Tx_Data[0], &Target_Omega, sizeof(float));
        CAN_Send_Data(CAN_Manage_Object->CAN_Handler, 0x200 + id, Tx_Data, 4);
        break;
    default:
        break;
    }
}

void Class_DM_Motor_S2325::TIM_PID_PeriodElapsedCallback()
{
    if (!Initialized) return;
    if (DM_Motor_Control_Method != DM_Motor_Control_Method_MIT_Angle &&
        DM_Motor_Control_Method != DM_Motor_Control_Method_MIT_OPENLOOP) return;

    if (DM_Motor_Control_Status != DM_Motor_Control_Status_ENABLE ||
        DM_Motor_Status != DM_Motor_Status_ENABLE || Data.ErrorCode != 1)
    {
        Out = Target_Omega = Target_Torque = 0.0f;
        PID_Angle.Set_Integral_Error(0.0f);
        PID_Omega.Set_Integral_Error(0.0f);
        Pack_MIT_Frame(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        return;
    }

    if (DM_Motor_Control_Method == DM_Motor_Control_Method_MIT_Angle)
    {
        PID_Angle.Set_Target(Target_Angle);
        PID_Angle.Set_Now(Data.Now_Angle_Rad);
        PID_Angle.TIM_Adjust_PeriodElapsedCallback();
        Target_Omega = PID_Angle.Get_Out();

        PID_Omega.Set_Target(Target_Omega);
        PID_Omega.Set_Now(Data.Now_Omega_Radian);
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();
        Out = PID_Omega.Get_Out() + FeedForward_Torque; // 输出轴 Nm
        if (PID_Polar == PID_Polar_Negative) Out = -Out;
    }
    // MIT_OPENLOOP 下 Set_Out() 直接指定输出轴扭矩 Nm。
    if (!std::isfinite(Out)) Out = 0.0f;
    Out = S2325_Limit(Out, -Torque_Max, Torque_Max);
    Target_Torque = Out;
    Pack_MIT_Frame(0.0f, 0.0f, 0.0f, 0.0f, Out);
}

void Class_DM_Motor_S2325::TIM_Send_PeriodElapsedCallback()
{
    if (!Initialized || DM_Motor_Control_Status != DM_Motor_Control_Status_ENABLE ||
        DM_Motor_Status != DM_Motor_Status_ENABLE || Data.ErrorCode != 1) return;
    if (DM_Motor_Control_Method == DM_Motor_Control_Method_MIT_Angle ||
        DM_Motor_Control_Method == DM_Motor_Control_Method_MIT_OPENLOOP)
        CAN_Send_Data(CAN_Manage_Object->CAN_Handler, static_cast<uint8_t>(CAN_ID), Tx_Data, 8);
}

void Class_DM_Motor_S2325::Clear_Error()
{
    if (!Initialized) return;
    CAN_Send_Data(CAN_Manage_Object->CAN_Handler, static_cast<uint8_t>(CAN_ID),
                  DM_Motor_CAN_Message_Clear_Error, 8);
}


/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
