#include "tsk_config_and_callback.h"

#include "config.h"
#include "alg_gravity_comp.h"
#include "drv_can.h"
#include "drv_dwt.h"
#include "drv_tim.h"
#include "alg_RNEA.h"
#include "fdcan.h"
#include "tim.h"

// 关节实例按机械臂编号为 1～5；J0～J4 CAN 宏仍对应原来的五个电机地址。
Class_Joint_DM_Motor Joint1_DM_Motor;
Class_Joint_DM_Motor Joint2_DM_Motor;
Class_Joint_DM_Motor Joint3_DM_Motor;
Class_Joint_DM_Motor Joint4_DM_Motor;
Class_Joint_DM_Gear Joint5_DM_Motor;

//Ozone直接观测，避免依赖调试器解析派生类的基类成员。
volatile bool Joint5_Calibration_Initialized = false;
volatile bool Joint5_Calibration_Parameter_Valid = false;
volatile uint8_t Joint5_Calibration_Status = Joint_DM_Gear_Calibration_IDLE;
//Ozone写true触发一次校准，任务消费后自动清零。
volatile bool Joint5_Calibration_Start_Request = false;
volatile uint32_t Joint5_Calibration_State_Time_Ms = 0;
//J5最新CAN反馈力矩，保留符号；单位与校准阈值一致。
volatile float Joint5_Feedback_Torque_Nm = 0.0f;


//机械臂MDH参数(a_i, α_i, d_i, θ_i)，下标0为基座行，长度单位m。
//关节1~3取自原alg_gravity_comp.cpp中的模型（69.55mm、230mm已换算为m）：
//  关节1 (a=0,     α=-pi/2, d=69.55mm)
//  关节2 (a=230mm, α=0,     d=0)
//  关节3 (a=0,     α=pi/2,  d=0)
//关节4~6暂无参数，先填0。
//注意：θ只是初值，运行时由Set_RNEA_Joint_Angle_Rad()/周期回调按关节角刷新；
//      原模型关节2、3的θ符号为-q2、-q3，设置角度时需保持同样的符号约定。
const Struct_MDH_Parameter Joint_MDH_Parameter[JOINT_NUM + 1] = {
    /* 0 基座 */ {0.0f, 0.0f, 0.0f, 0.0f},
    /* 1      */ {0.0f, -PI/2, 0.06955f, 0.0f},
    /* 2      */ {0.230f, 0.0f, 0.0f, 0.0f},
    /* 3      */ {0.0f, PI/2, 0.0f, 0.0f},
    /* 4      */ {0.0f, 0.0f, 0.0f, 0.0f},
    /* 5      */ {0.0f, 0.0f, 0.0f, 0.0f},
    /* 6      */ {0.0f, 0.0f, 0.0f, 0.0f},
};

//3关节重力补偿(robotics库Class_Gravity_Comp)：电流系数/质心在alg_gravity_comp.cpp
//的Build_Default_Config中填写；解算结果存入观察数组，由调用处按需下发。
static Class_Gravity_Comp Joint_Gravity_Comp_3J;
float Joint_Gravity_Torque_3J_Nm[GRAVITY_COMP_JOINT_NUM] = {};
//Ozone调参输入：关节1~3电流系数(电机电流单位，与Mass一致)，初值与
//alg_gravity_comp.cpp的Build_Default_Config一致，修改时需同步。
//每周期快照读取，变化时重建3关节模型；复位恢复初值。
volatile float Joint_Gravity_Mass_3J[GRAVITY_COMP_JOINT_NUM] = {0.0f, 20.0f, 30.0f};
static float Last_Gravity_Mass_3J[GRAVITY_COMP_JOINT_NUM] = {0.0f, 20.0f, 30.0f};

// 保留 2 s 启动等待；测试角度默认为 0°，不会自动扫描。
volatile float test_j1_angle_deg = 0.0f;
volatile float test_j2_angle_deg = 0.0f;
volatile float test_j3_angle_deg = 0.0f;
volatile float test_j4_angle_deg = 0.0f;
// S2325 堵转校准完成后，以机械限位处为0；此角度为校准后的关节目标角。
float test_j5_angle_deg = 0.0f;

Class_RNEA RNEA;
//RNEA重力补偿观察数组(N*m)，下标1~JOINT_NUM有效，仅调试查看，不直接下发
float Joint_Gravity_Torque_RNEA_Nm[JOINT_NUM + 1] = {};

//RNEA解算耗时统计（DWT CYCCNT，CPU频率与Task_Init里的DWT_Init(480)一致）
//统计范围：设置关节角 → TIM_RENA_Update_PeriodElapsedCallback()返回
volatile uint32_t RNEA_Calc_Start_Cycle = 0;        //本次计时起始CYCCNT
volatile uint32_t RNEA_Calc_End_Cycle = 0;          //本次计时结束CYCCNT
volatile uint32_t RNEA_Calc_Cycles = 0;             //本次解算周期数
volatile float RNEA_Calc_Time_Us = 0.0f;            //本次解算耗时，us
volatile uint32_t RNEA_Calc_Cycles_Min = 0xFFFFFFFFu;//最小周期数（Ozone里写回0xFFFFFFFF可重新统计）
volatile uint32_t RNEA_Calc_Cycles_Max = 0;         //最大周期数
volatile float RNEA_Calc_Time_Us_Min = 0.0f;        //最小耗时，us
volatile float RNEA_Calc_Time_Us_Max = 0.0f;        //最大耗时，us
volatile uint64_t RNEA_Calc_Cycles_Total = 0;       //累计周期数
volatile uint32_t RNEA_Calc_Count = 0;              //统计次数
volatile float RNEA_Calc_Time_Us_Avg = 0.0f;        //平均耗时，us
//与Task_Init里的DWT_Init(480)保持一致
static const float kRneaCpuFreqMHz = 480.0f;



void Joint_CAN1_Callback(Struct_CAN_Rx_Buffer *message)
{
    if (message->Header.Identifier == J0_MASTER_ID)
    {
        Joint1_DM_Motor.Joint_Motor_DM_J4340.CAN_RxCpltCallback(message->Data);
        Joint1_DM_Motor.Now_Joint_Angle_Deg = Joint1_DM_Motor.Get_Joint_Angle_Gain() *
            (Joint1_DM_Motor.Joint_Motor_DM_J4340.Get_Now_Angle_Deg() + Joint1_DM_Motor.Get_Joint_Angle_Bias_Deg());
        Joint1_DM_Motor.Now_Joint_Angle_Rad = Joint1_DM_Motor.Get_Joint_Angle_Gain() *
            (Joint1_DM_Motor.Joint_Motor_DM_J4340.Get_Now_Angle_Rad() + Joint1_DM_Motor.Get_Joint_Angle_Bias_Rad());
    }
    else if (message->Header.Identifier == J1_MASTER_ID)
    {
        Joint2_DM_Motor.Joint_Motor_DM_J4340.CAN_RxCpltCallback(message->Data);
        Joint2_DM_Motor.Now_Joint_Angle_Deg = Joint2_DM_Motor.Get_Joint_Angle_Gain() *
            (Joint2_DM_Motor.Joint_Motor_DM_J4340.Get_Now_Angle_Deg() + Joint2_DM_Motor.Get_Joint_Angle_Bias_Deg());
        Joint2_DM_Motor.Now_Joint_Angle_Rad = Joint2_DM_Motor.Get_Joint_Angle_Gain() *
            (Joint2_DM_Motor.Joint_Motor_DM_J4340.Get_Now_Angle_Rad() + Joint2_DM_Motor.Get_Joint_Angle_Bias_Rad());
    }
    else if (message->Header.Identifier == J2_MASTER_ID)
    {
        Joint3_DM_Motor.Joint_Motor_DM_J4340.CAN_RxCpltCallback(message->Data);
        Joint3_DM_Motor.Now_Joint_Angle_Deg = Joint3_DM_Motor.Get_Joint_Angle_Gain() *
            (Joint3_DM_Motor.Joint_Motor_DM_J4340.Get_Now_Angle_Deg() + Joint3_DM_Motor.Get_Joint_Angle_Bias_Deg());
        Joint3_DM_Motor.Now_Joint_Angle_Rad = Joint3_DM_Motor.Get_Joint_Angle_Gain() *
            (Joint3_DM_Motor.Joint_Motor_DM_J4340.Get_Now_Angle_Rad() + Joint3_DM_Motor.Get_Joint_Angle_Bias_Rad());
    }
    else if (message->Header.Identifier == J3_MASTER_ID)
    {
        Joint4_DM_Motor.Joint_Motor_DM_J4340.CAN_RxCpltCallback(message->Data);
        Joint4_DM_Motor.Now_Joint_Angle_Deg = Joint4_DM_Motor.Get_Joint_Angle_Gain() *
            (Joint4_DM_Motor.Joint_Motor_DM_J4340.Get_Now_Angle_Deg() + Joint4_DM_Motor.Get_Joint_Angle_Bias_Deg());
        Joint4_DM_Motor.Now_Joint_Angle_Rad = Joint4_DM_Motor.Get_Joint_Angle_Gain() *
            (Joint4_DM_Motor.Joint_Motor_DM_J4340.Get_Now_Angle_Rad() + Joint4_DM_Motor.Get_Joint_Angle_Bias_Rad());
    }
    else if (message->Header.Identifier == J4_MASTER_ID)
    {
        Joint5_DM_Motor.Joint_Motor_DM_S2325.CAN_RxCpltCallback(message->Data);
        Joint5_Feedback_Torque_Nm = Joint5_DM_Motor.Joint_Motor_DM_S2325.Get_Now_Torque();
        Joint5_DM_Motor.Update_Joint_Feedback();
    }
}

//每个1 ms控制周期调用一次，使用最新实际关节角。
float scaler1 = -1.0f;
float scaler2 = 1.0f;
float scaler3 = -1.0f;

int motor_cnt = 0;


void Joint_TIM5_Callback()
{
    static uint32_t startup_ticks = 0;
    static uint8_t alive_ticks = 0;
    bool heartbeat_slot = false;

    // 分开提交心跳帧，避免连续五帧挤满 FDCAN 发送 FIFO。
    ++alive_ticks;
    if (alive_ticks == 25)
    {
        heartbeat_slot = true;
        Joint1_DM_Motor.Joint_Motor_DM_J4340.TIM_Alive_PeriodElapsedCallback();
        //Joint1_DM_Motor.Joint_Motor_DM_J4340.TIM_Process_PeriodElapsedCallback();
        Joint2_DM_Motor.Joint_Motor_DM_J4340.TIM_Alive_PeriodElapsedCallback();
        //Joint2_DM_Motor.Joint_Motor_DM_J4340.TIM_Process_PeriodElapsedCallback();
        Joint5_DM_Motor.Joint_Motor_DM_S2325.TIM_Alive_PeriodElapsedCallback();
        //Joint5_DM_Motor.Joint_Motor_DM_S2325.TIM_Process_PeriodElapsedCallback();
    }
    else if (alive_ticks == 50)
    {
        heartbeat_slot = true;
        alive_ticks = 0;
        Joint3_DM_Motor.Joint_Motor_DM_J4340.TIM_Alive_PeriodElapsedCallback();
        //Joint3_DM_Motor.Joint_Motor_DM_J4340.TIM_Process_PeriodElapsedCallback();
        Joint4_DM_Motor.Joint_Motor_DM_J4340.TIM_Alive_PeriodElapsedCallback();
        //Joint4_DM_Motor.Joint_Motor_DM_J4340.TIM_Process_PeriodElapsedCallback();
    }

    if (startup_ticks < 2000)
    {
        ++startup_ticks;
        return;
    }

    //J1~J4轮询发送MIT控制帧，J5由校准状态机的周期函数独立处理。
    switch ((motor_cnt++)%4) {
        case 0: {
            Joint1_DM_Motor.Joint_Motor_DM_J4340.TIM_Process_PeriodElapsedCallback();
            break;
        }
        case 1: {
            Joint2_DM_Motor.Joint_Motor_DM_J4340.TIM_Process_PeriodElapsedCallback();
            break;
        }
        case 2: {
            Joint3_DM_Motor.Joint_Motor_DM_J4340.TIM_Process_PeriodElapsedCallback();
            break;
        }
        case 3: {
            Joint4_DM_Motor.Joint_Motor_DM_J4340.TIM_Process_PeriodElapsedCallback();
            break;
        }
    }

    //Joint_Gravity_Update();

    Joint1_DM_Motor.Set_Joint_Target_Angle_Deg(test_j1_angle_deg);

    Joint2_DM_Motor.Set_Joint_Target_Angle_Deg(test_j2_angle_deg);

    Joint3_DM_Motor.Set_Joint_Target_Angle_Deg(test_j3_angle_deg);

    Joint4_DM_Motor.Set_Joint_Target_Angle_Deg(test_j4_angle_deg);

    if (Joint5_Calibration_Start_Request)
    {
        Joint5_Calibration_Start_Request = false;
        Joint5_DM_Motor.Request_Recalibration();
    }

    //校准期间FSM独占MIT指令，完成后才接收正常位置目标。
    if (Joint5_DM_Motor.Get_Calibration_Complete())
        Joint5_DM_Motor.Set_Joint_Target_Angle_Deg(test_j5_angle_deg);
    Joint5_DM_Motor.TIM_Process_PeriodElapsedCallback(!heartbeat_slot);
    Joint5_Calibration_Status = Joint5_DM_Motor.FSM_Calibration.Get_Now_Status_Serial();
    Joint5_Calibration_State_Time_Ms =
        Joint5_DM_Motor.FSM_Calibration.Status[Joint5_Calibration_Status].Time;


    //统计"设置关节角 → 解算完成"的耗时（DWT CYCCNT@480MHz）
    const uint32_t rnea_start_cycle = DWT->CYCCNT;
    // RNEA.Set_RNEA_Joint_Angle_Deg(1,test_j1_angle_deg);
    // RNEA.Set_RNEA_Joint_Angle_Deg(2,test_j2_angle_deg);
    // RNEA.Set_RNEA_Joint_Angle_Deg(3,test_j3_angle_deg);
    RNEA.TIM_RENA_Update_PeriodElapsedCallback();
    const uint32_t rnea_end_cycle = DWT->CYCCNT;

    const uint32_t rnea_cycles = rnea_end_cycle - rnea_start_cycle;
    const float rnea_time_us = static_cast<float>(rnea_cycles) / kRneaCpuFreqMHz;
    RNEA_Calc_Start_Cycle = rnea_start_cycle;
    RNEA_Calc_End_Cycle = rnea_end_cycle;
    RNEA_Calc_Cycles = rnea_cycles;
    RNEA_Calc_Time_Us = rnea_time_us;
    if (rnea_cycles < RNEA_Calc_Cycles_Min) {
        RNEA_Calc_Cycles_Min = rnea_cycles;
        RNEA_Calc_Time_Us_Min = rnea_time_us;
    }
    if (rnea_cycles > RNEA_Calc_Cycles_Max) {
        RNEA_Calc_Cycles_Max = rnea_cycles;
        RNEA_Calc_Time_Us_Max = rnea_time_us;
    }
    RNEA_Calc_Cycles_Total += rnea_cycles;
    ++RNEA_Calc_Count;
    RNEA_Calc_Time_Us_Avg = static_cast<float>(RNEA_Calc_Cycles_Total) /
                            (static_cast<float>(RNEA_Calc_Count) * kRneaCpuFreqMHz);

    //把解算结果拷到观察数组，方便Ozone里直接看（RNEA为全局对象，其内部数组也能展开查看）
    for (uint8_t i = 0 ; i<=JOINT_NUM ; i++) {
        Joint_Gravity_Torque_RNEA_Nm[i] = RNEA.Get_Joint_Gravity_Comp_Torque_Nm(i);
    }

    RNEA.Set_RNEA_Joint_Angle_Rad(1,Joint1_DM_Motor.Get_Joint_Now_Angle_Rad());
    RNEA.Set_RNEA_Joint_Angle_Rad(2,Joint2_DM_Motor.Get_Joint_Now_Angle_Rad());
    RNEA.Set_RNEA_Joint_Angle_Rad(3,Joint3_DM_Motor.Get_Joint_Now_Angle_Rad());



    Joint1_DM_Motor.Set_Real_Target_Torque(-RNEA.Get_Joint_Gravity_Comp_Torque_Nm(1) / Joint1_DM_Motor.Get_Joint_Angle_Gain() * scaler1);
    Joint2_DM_Motor.Set_Real_Target_Torque(-RNEA.Get_Joint_Gravity_Comp_Torque_Nm(2) * scaler2);
    Joint3_DM_Motor.Set_Real_Target_Torque(-RNEA.Get_Joint_Gravity_Comp_Torque_Nm(3) * scaler3);
    // 心跳周期只发使能帧；其他周期五个关节均按 1 ms 控制节奏发送。
    if (heartbeat_slot)
        return;

}

extern "C" void Task_Init(void)
{
    DWT_Init(480);
    CAN_Init(&hfdcan1, Joint_CAN1_Callback);

    Joint1_DM_Motor.Init(&ARM_CAN, (Enum_DM_Motor_ID)J0_CAN_ID, 3.141593f, 10.0f, 10.0f);
    Joint1_DM_Motor.Joint_Motor_DM_J4340.Set_MIT_K_D(10.0f);
    Joint1_DM_Motor.Joint_Motor_DM_J4340.Set_MIT_K_P(70.0f);
    Joint1_DM_Motor.Set_Joint_Angle_Bias_Deg(0.0f);
    Joint1_DM_Motor.Set_Joint_Angle_Gain(-17.0f/28);
    Joint1_DM_Motor.Set_Max_Joint_Angle_Deg(180.0f);
    Joint1_DM_Motor.Set_Min_Joint_Angle_Deg(-180.0f);
    Joint1_DM_Motor.Joint_Motor_DM_J4340.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
    Joint1_DM_Motor.Joint_Motor_DM_J4340.TIM_Alive_PeriodElapsedCallback();


    Joint2_DM_Motor.Init(&ARM_CAN, (Enum_DM_Motor_ID)J1_CAN_ID, 3.141593f, 10.0f, 14.0f);
    Joint2_DM_Motor.Joint_Motor_DM_J4340.Set_MIT_K_D(5.0f);
    Joint2_DM_Motor.Joint_Motor_DM_J4340.Set_MIT_K_P(60.0f);
    Joint2_DM_Motor.Set_Joint_Angle_Bias_Deg(-80.0009079);
    Joint2_DM_Motor.Set_Joint_Angle_Gain(-1.0f);
    Joint2_DM_Motor.Joint_Motor_DM_J4340.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
    Joint2_DM_Motor.Set_Max_Joint_Angle_Deg(0.0f);
    Joint2_DM_Motor.Set_Min_Joint_Angle_Deg(-90.0f);
    Joint2_DM_Motor.Joint_Motor_DM_J4340.TIM_Alive_PeriodElapsedCallback();

    Joint3_DM_Motor.Init(&ARM_CAN, (Enum_DM_Motor_ID)J2_CAN_ID, 3.141593f, 10.0f, 12.0f);
    Joint3_DM_Motor.Joint_Motor_DM_J4340.Set_MIT_K_D(5.0f);
    Joint3_DM_Motor.Joint_Motor_DM_J4340.Set_MIT_K_P(40.0f);
    Joint3_DM_Motor.Set_Joint_Angle_Bias_Deg(0.0f);
    Joint3_DM_Motor.Set_Joint_Angle_Gain(1.0f);
    Joint3_DM_Motor.Joint_Motor_DM_J4340.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
    Joint3_DM_Motor.Set_Max_Joint_Angle_Deg(0.0f);
    Joint3_DM_Motor.Set_Min_Joint_Angle_Deg(-90.0f);
    Joint3_DM_Motor.Joint_Motor_DM_J4340.TIM_Alive_PeriodElapsedCallback();

    Joint4_DM_Motor.Init(&ARM_CAN, (Enum_DM_Motor_ID)J3_CAN_ID, 3.141593f, 10.0f, 14.0f);
    Joint4_DM_Motor.Joint_Motor_DM_J4340.Set_MIT_K_D(2.0f);
    Joint4_DM_Motor.Joint_Motor_DM_J4340.Set_MIT_K_P(30.0f);
    Joint4_DM_Motor.Set_Joint_Angle_Bias_Deg(-20.0f);
    Joint4_DM_Motor.Set_Joint_Angle_Gain(1.0f);
    Joint4_DM_Motor.Joint_Motor_DM_J4340.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
    Joint4_DM_Motor.Set_Max_Joint_Angle_Deg(30.0f);
    Joint4_DM_Motor.Set_Min_Joint_Angle_Deg(-30.0f);
    Joint4_DM_Motor.Joint_Motor_DM_J4340.TIM_Alive_PeriodElapsedCallback();

    Joint5_DM_Motor.Init(&ARM_CAN, static_cast<Enum_DM_Motor_ID>(J4_CAN_ID),
                         J4_POSITION_MAX, J4_OMEGA_MAX, J4_TORQUE_MAX);
    //上电寻零：MIT目标速度rad/s（正负指定方向）、反馈力矩阈值、持续时间ms、MIT Kd。
    Joint5_Calibration_Initialized = Joint5_DM_Motor.FSM_Calibration.Joint == &Joint5_DM_Motor;
    Joint5_Calibration_Status = Joint5_DM_Motor.FSM_Calibration.Get_Now_Status_Serial();
    Joint5_Calibration_State_Time_Ms = 0;
    //参数直接写入，不做校验；返回true表示已写入。底层MIT帧量程由驱动处理。
    Joint5_Calibration_Parameter_Valid =
        Joint5_DM_Motor.Set_Calibration_Parameter(-3.0f, 1.0f, 20, 5.0f);
    Joint5_DM_Motor.Set_Joint_Angle_Bias_Deg(0.0f);
    Joint5_DM_Motor.Set_Joint_Angle_Gain(1.0f);
    Joint5_DM_Motor.Joint_Motor_DM_S2325.Set_MIT_K_P(10.0f);
    Joint5_DM_Motor.Joint_Motor_DM_S2325.Set_MIT_K_D(2.0f);
    Joint5_DM_Motor.Joint_Motor_DM_S2325.Set_Target_Omega(0.0f);
    Joint5_DM_Motor.Joint_Motor_DM_S2325.Set_Target_Torque(0.0f);
    Joint5_DM_Motor.Joint_Motor_DM_S2325.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);

    Joint5_DM_Motor.Joint_Motor_DM_S2325.TIM_Alive_PeriodElapsedCallback();

    // 用顶部的MDH参数表配置RNEA模型（质量kg、质心m、惯量张量待标定后填入，这里先给0）
    //RNEA.Set_RNEA_Joint_Parameter(0,Joint_MDH_Parameter[0],0.365,{})
    RNEA.Set_Gravity_RNEA(Class_Vector_3D(0.0f, -9.81f, 0.0f));
    RNEA.Set_RNEA_Joint_Parameter(0,Joint_MDH_Parameter[0],0,{0,0,0},Class_Matrix::Zero(3, 3));
    RNEA.Set_RNEA_Joint_Parameter(1,Joint_MDH_Parameter[1],0.365278890190267f,{0,0,0.0273186084008064},Class_Matrix::Zero(3, 3));
    RNEA.Set_RNEA_Joint_Parameter(2,Joint_MDH_Parameter[2],0.102925119440542f,{0.153476162801064,0,0},Class_Matrix::Zero(3, 3));
    RNEA.Set_RNEA_Joint_Parameter(3,Joint_MDH_Parameter[3],1.4f,{0.140,0,0},Class_Matrix::Zero(3, 3));

    //J4~J6暂无质量/质心，保持0；需要时按上面同样的写法补上
    RNEA.Init();

    TIM_Init(&htim5, Joint_TIM5_Callback);
    HAL_TIM_Base_Start_IT(&htim5);
}
