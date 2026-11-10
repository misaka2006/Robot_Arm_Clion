# Robot_Arm_Clion 机械臂控制固件

> 基于 **STM32H723（Cortex-M7 @ 480 MHz）** 的 5 自由度机械臂关节控制固件：通过 FDCAN 驱动达妙（DM）电机，1 kHz 控制周期，使用 **改进 DH 运动学 + 静态 RNEA 重力补偿**，并支持用 J-Link/Ozone 在线观测中间量、计时和参数调试。

---

## 目录

1. [项目概述](#1-项目概述)
2. [快速开始](#2-快速开始)
3. [硬件与工具链](#3-硬件与工具链)
4. [目录结构](#4-目录结构)
5. [软件架构](#5-软件架构)
6. [核心算法](#6-核心算法)
7. [参数配置](#7-参数配置)
8. [调试与观测](#8-调试与观测)
9. [当前状态与限制](#9-当前状态与限制)
10. [扩展与优化建议](#10-扩展与优化建议)
11. [参考](#11-参考)

---

## 1. 项目概述

- **执行机构**：J1~J4 为达妙 `DM-J4340`，J5 为达妙 `DM-S2325`（内置 25:1 减速，反馈角度为输出轴角度）。
- **通信**：FDCAN1（`hfdcan1`，宏 `ARM_CAN`），经典 CAN 帧，达妙 MIT 协议（8 字节）。
- **控制节拍**：TIM5 更新中断，1 kHz；上电后有 2 s 启动等待。
- **核心功能**：
  - 关节位置目标下发（MIT 位置模式）；
  - 基于改进 DH 的静态重力补偿（RNEA 反向递推）；
  - 关节力矩 → 电机 12 位 MIT 力矩字段的换算；
  - 运行状态、补偿力矩、解算耗时等变量的在线观测。
- **代码基座**：沿用 RoboMaster 风格的 `Driver / Device / Algorithm / Chariot / Interaction / Task` 分层框架。底盘、云台、发射机构、裁判系统等模块保留在仓库中，但**当前实际接入的是机械臂任务层**（`User/Task`）。

---

## 2. 快速开始

### 2.1 依赖

| 工具 | 说明 |
|---|---|
| `arm-none-eabi-g++` | GCC ARM 交叉工具链（工程内 `cmake/gcc-arm-none-eabi.cmake` 指定） |
| CMake ≥ 3.22 + Ninja | 构建系统（`CMakePresets.json` 提供 Debug/Release） |
| CLion | 推荐 IDE（也可纯命令行） |
| J-Link + Ozone | 烧录与在线调试（工程根目录 `Robot_Arm_Clion.jdebug`） |

### 2.2 构建

```bash
cmake --preset Debug
cmake --build --preset Debug
```

产物（以 Debug 为例）：

- `build/Debug/Robot_Arm_Clion.axf`：ELF，带调试符号，供 Ozone 使用；
- `build/Debug/Robot_Arm_Clion.hex`：Intel HEX，供烧录。

Release 用 `--preset Release`（`-Os`）。

### 2.3 烧录与运行

1. 用 Ozone 打开 `Robot_Arm_Clion.jdebug`，下载 `Robot_Arm_Clion.axf`；
2. 复位运行，**等待约 2 s**（`startup_ticks < 2000` 的启动门限）；
3. 在 Live Watch 里观察 `Joint_Gravity_Torque_RNEA_Nm[]`、`RNEA_Calc_Time_Us*`、`test_j*_angle_deg` 等变量（见 §8）。

> 当前 J1~J4 的控制帧发送仍被注释，RNEA 补偿不会真正下发。J5 已接入手动触发的堵转校准，上电保持IDLE，收到校准请求后发送原生 MIT 速度帧，并在完成后接收正常位置目标（见 §6.7）。

---

## 3. 硬件与工具链

| 项目 | 内容 |
|---|---|
| MCU | STM32H723VGTx，Cortex-M7，最高 480 MHz |
| 时钟 | HSE 24 MHz → PLL（M=2，N=40，P=1）→ SYSCLK 480 MHz；HCLK 240 MHz |
| 存储 | 1 MB FLASH；128 KB DTCM（栈预留 32 KB，堆预留 24 KB）；另有 D1/D2/D3 RAM |
| 浮点/DSP | FPv5-D16 硬浮点（`-mfpu=fpv5-d16`）；CMSIS-DSP 单精度库 `libarm_cortexM7lfsp_math.a`（`ARM_MATH_CM7`） |
| 总线 | FDCAN1 连接达妙电机；UART/USB 等外设保留 |
| 编译标准 | C11 / C++11，`-fno-exceptions -fno-rtti -fno-threadsafe-statics` |
| 构建类型 | Debug：`-O0 -g3`；Release：`-Os -g0` |
| 调试 | J-Link + Ozone；DWT `CYCCNT` 计时（480 MHz） |

代码放在 FLASH（`.text @ 0x0800_0000`），全局变量、栈、堆放在 DTCM（`.bss/.data @ 0x2000_0000`）。目前**未开启 I-Cache/D-Cache**（见 §10 的性能建议）。

---

## 4. 目录结构

```
Robot_Arm_Clion/
├── Core/                       # STM32CubeMX 生成的 HAL 初始化（main.c、tim.c、fdcan.c…）
├── Drivers/                    # STM32H7 HAL + CMSIS（DSP Include/Lib）
├── Middlewares/                # USB Device 库
├── cmake/                      # CubeMX 生成的 CMake、交叉编译工具链
├── RobotArm/                   # 机械臂关节抽象层（rbt_Joint 及 DM4310/DM3520 版本）
├── User/
│   ├── Algorithm/              # 矩阵/矢量、RNEA、robotics、PID、滤波、FSM、功率限制…
│   ├── Chariot/                # 底盘/云台/发射机构/机械臂模型等整车库（当前大部分未接入）
│   ├── Congfig/                # config.h：CAN ID、限幅、默认参数
│   ├── Device/                 # 电机、IMU、遥控器、裁判系统、超电、TOF 等设备驱动
│   ├── Driver/                 # CAN/UART/SPI/I2C/ADC/TIM/DWT 等 BSP
│   ├── Interaction/            # 整车人机交互/FSM（当前未接入）
│   ├── Task/                   # ★ 当前应用的装配层：实例、回调、参数、观测变量
│   └── UI/                     # RoboMaster UI 绘图框架
├── build/                      # CMake 构建目录（Debug/Release）
├── CMakeLists.txt              # 工程构建入口
├── CMakePresets.json           # Debug/Release 预设
├── STM32H723xG_flash.ld        # 链接脚本
├── Robot_Arm_Clion.ioc         # CubeMX 配置
└── Robot_Arm_Clion.jdebug      # Ozone 调试工程
```

构建时实际参与编译的源码：

- 显式加入：`RobotArm/Src/rbt_Joint.cpp`、`User/Algorithm/Src/alg_matrix.cpp`、`User/Algorithm/Src/alg_RNEA.cpp`；
- 自动 glob：`User/*/Src/*.c`、`User/*/Src/*.cpp`；
- **排除**：`User/Chariot/Src/crt_robot_arm.cpp`（旧整臂实现，当前不编译，见 `CMakeLists.txt`）。

### 4.1 关键文件速查

| 想改什么 | 看这里 |
|---|---|
| 控制节拍、回调、模型参数、观测变量 | `User/Task/Src/tsk_config_and_callback.cpp` |
| RNEA 运动学 + 重力递推 | `User/Algorithm/Src/alg_RNEA.cpp` |
| 矩阵/矢量运算 | `User/Algorithm/Src/alg_matrix.cpp` |
| 关节角/前馈力矩换算 | `RobotArm/Src/rbt_Joint.cpp` |
| 达妙电机协议、12 位 MIT 帧 | `User/Device/Src/dvc_dmmotor.cpp` |
| CAN 接收中断与分发 | `User/Driver/Src/drv_can.cpp` |
| CAN ID、限幅、默认参数 | `User/Congfig/Inc/config.h` |
| 内存布局、栈/堆大小 | `STM32H723xG_flash.ld` |

---

## 5. 软件架构

### 5.1 分层

| 层 | 目录 | 职责 |
|---|---|---|
| 板级/启动 | `Core/` | 时钟、GPIO、FDCAN、TIM、UART、USB 等 HAL 初始化；`main()` |
| 驱动层 | `User/Driver/` | CAN/UART/SPI/I2C/ADC/TIM/DWT 的初始化与中断封装 |
| 设备层 | `User/Device/` | 达妙/大疆/LK 电机、IMU、遥控器、裁判系统、超电等 |
| 关节层 | `RobotArm/` | 关节角度↔电机角度换算、前馈力矩换算、限幅 |
| 算法层 | `User/Algorithm/` | 矩阵/矢量、运动学（robotics）、RNEA 重力补偿、PID、滤波、FSM 等 |
| 机器人层 | `User/Chariot/`、`User/Interaction/` | 底盘/云台/发射机构/整车 FSM（当前未接入） |
| 应用层 | `User/Task/` | 关节实例、CAN 回调、1 kHz 控制回调、模型参数、观测变量 |
| 配置 | `User/Congfig/` | CAN ID、限幅、默认 PID 等宏 |

### 5.2 启动流程

```c
main()
 ├─ HAL_Init()
 ├─ SystemClock_Config()          // 480 MHz
 ├─ MX_GPIO_Init / MX_DMA_Init
 ├─ MX_FDCAN1/2/3_Init
 ├─ MX_SPI2/TIM3/TIM4/TIM5/TIM2/TIM12_Init
 ├─ MX_USART/UART/USB_Init
 ├─ Task_Init()                   // ★ 用户应用初始化
 └─ while (1) { }                 // 所有实时工作在中断里完成
```

`Task_Init()`（`User/Task/Src/tsk_config_and_callback.cpp`）主要做：

1. `DWT_Init(480)`：初始化周期计数器（用于计时）；
2. `CAN_Init(&hfdcan1, Joint_CAN1_Callback)`：注册 CAN1 接收回调；
3. 5 个关节实例的初始化：`Init(..., torque_max)`、角度增益/偏置、限位、使能、心跳；
4. 配置 RNEA：重力向量、MDH 表、质量/质心，然后 `RNEA.Init()`；
5. `TIM_Init(&htim5, Joint_TIM5_Callback)` 并启动 1 kHz 中断。

### 5.3 中断与数据流

```
CAN 反馈帧
  └─ FDCAN1 RX 中断：Joint_CAN1_Callback()
       ├─ 按 CAN ID 分发到 5 个关节
       ├─ motor.CAN_RxCpltCallback()
       └─ 更新 Now_Joint_Angle_Rad

TIM5 1 kHz 中断：Joint_TIM5_Callback()
  ├─ 心跳/使能帧（每 25/50 ms）
  ├─ 2 s 启动门限
  ├─ 轮流处理一个电机的 Process
  ├─ 设定测试目标角
  ├─ RNEA 位姿更新 + 重力递推
  ├─ DWT 计时
  ├─ 拷贝力矩到观测数组
  ├─ 更新 RNEA 关节角
  └─ 写 Target_Torque（scalers）

J1~J4 控制帧发送（当前被注释）；J5 原生 MIT 控制由关节周期函数直接发帧
```

要点：

- **无 RTOS**：全部实时逻辑由中断驱动，`main()` 只保留空循环；
- CAN 接收回调负责把电机反馈换算成关节角；
- 1 kHz 控制回调负责心跳、目标角、重力补偿、观测变量；
- J1~J4 控制帧发送仍被注释；J5 通过关节周期函数执行校准并发送 MIT 控制帧，心跳周期跳过发送但仍更新 FSM。

---

## 6. 核心算法

### 6.1 矩阵/矢量库（`alg_matrix.h/.cpp`）

- `Class_Matrix`：运行期指定行列数、编译期固定容量的 float 矩阵（默认上限 8×8，宏 `ALG_MATRIX_MAX_ROWS/COLS`），行主序存储，内部持有 `arm_matrix_instance_f32`，可直接交给 CMSIS-DSP。
- `Class_Vector_3D`：3×1 列矢量，内部同样用 `arm_matrix_instance_f32` 表示。
- 所有数学运算走 CMSIS-DSP：`arm_mat_mult_f32`、`arm_mat_trans_f32`、`arm_mat_scale_f32`、`arm_add_f32`、`arm_dot_prod_f32`、`arm_sqrt_f32` 等。
- 不动态申请内存；拷贝赋值会重新绑定 `arm_matrix_instance_f32` 到自身缓冲区；尺寸不匹配/越界在 Debug 下用 `assert` 定位。

### 6.2 改进 DH 运动学（`Class_RNEA`）

参数定义（`Struct_MDH_Parameter`）：

```c
typedef struct { float a; float alpha; float d; float theta; } Struct_MDH_Parameter;
```

`Joint_Parameter[i]` 存放 `{a_i, α_i, d_i, θ_i}`（`i = 0` 为基座行），而 `T_i^{i-1}` 使用：

- `a_{i-1}`、`α_{i-1}`（上一行）
- `d_i`、`θ_i`（本行）

局部变换矩阵（改进 DH / Craig）：

```
        |  cθ       -sθ        0       a_{i-1} |
T_i =   |  sθ·cα     cθ·cα    -sα     -d_i·sα  |
        |  sθ·sα     cθ·sα     cα      d_i·cα  |
        |  0          0        0       1       |
```

其中 `cθ = cos θ_i`、`sθ = sin θ_i`、`cα = cos α_{i-1}`、`sα = sin α_{i-1}`，等价于：

```
T_i^{i-1} = Rx(α_{i-1}) · Tx(a_{i-1}) · Rz(θ_i) · Tz(d_i)
```

全局位姿：

```
T_0^0 = I
T_0^i = T_0^{i-1} · T_i^{i-1}
```

从 `T_0^i` 提取：

- 关节轴：`z_i = R_i · [0, 0, 1]^T`（旋转矩阵第 3 列）；
- 关节旋转中心：`p_i = T_0^i` 的平移部分（**不是**局部矩阵的平移）；
- 连杆质心在基座系：`p_{c,i} = R_i · c_i + p_i`。

实现见 `User/Algorithm/Src/alg_RNEA.cpp` 的 `Update_Homo_Trans_Matrix()`。

### 6.3 静态 RNEA 重力补偿

对每个关节 `i`，从末端向基座递推总重力和总重力矩（`F_{N+1} = 0`，`M_{N+1} = 0`）：

```
F_i = m_i·g + F_{i+1}
M_i = (p_{c,i} − p_i) × (m_i·g)
    + (p_{i+1} − p_i) × F_{i+1}
    + M_{i+1}
τ_i^comp = − z_i · M_i
```

- `F_i`：连杆 i 及其下游连杆的总重力（N）；
- `M_i`：关于关节 i 旋转中心的总重力矩（N·m）；
- `τ_i^comp`：投影到关节轴的补偿力矩（N·m）；
- `p_{i+1}`：下一关节的旋转中心（`i = N` 时该项因 `F_{N+1} = 0` 自动消失）。

实现位置：`Class_RNEA::TIM_RENA_Update_PeriodElapsedCallback()`（`alg_RNEA.cpp`）。

特点与假设：

- 只做**静态**重力补偿，不含惯性、科氏力、摩擦和末端外载；
- 复杂度 `O(N)`，`N = JOINT_NUM`（当前 6 个槽位）；
- 假设关节均为旋转关节，重力向量在基座系中给定；
- 计算中间量 `F_i`、`M_i`、`τ_i` 均保存在对象内，便于调试观测。

### 6.4 关节角与力矩链

**角度**（CAN 反馈回调中）：

```
q_joint = gain × (q_motor + bias)
```

`gain`/`bias` 由 `Class_Joint_DM_Motor` 的 `Set_Joint_Angle_Gain/Bias` 配置。

**力矩**：

- RNEA 输出的是**关节侧** N·m；
- `Set_Real_Target_Torque()` 写入的是**电机侧** `Target_Torque`，不会自动乘关节传动比；
- 达妙 MIT 帧的力矩字段是 12 位无符号整数，编码时按 `Torque_Max` 线性映射：

```
code = Math_Float_To_Int(Target_Torque, −Torque_Max, +Torque_Max, 0, 4095)
```

- 量化步长 `2·Torque_Max / 4095`；`Torque_Max` **必须与驱动器内部 TMAX 一致**，否则会变成整条力矩链的增益误差；
- `Set_Joint_FeedForward_Torque_Nm()` 会按虚功关系乘 `Joint_Angle_Gain`，但它要求 `MIT_Angle` 模式；当前 J1~J4 是 `MIT_POSITION`，该接口会返回 `false`。

当前任务层用 `Set_Real_Target_Torque(-τ * scaler)` 下发，`scaler` 用于补偿传动比和符号（属于调试参数）。

### 6.5 旧版重力补偿（legacy，当前未启用）

`User/Algorithm/Src/alg_gravity_comp.cpp` 中的 `Class_Gravity_Comp`：

- 只做前 3 个关节；
- 复用 `robotics.h` 的**标准 DH** `Serial_Link<3>::rne()`；
- `Mass` 是“电机电流单位”的系数，输出按 `1/g` 归一化；
- 定时回调中的调用 `Joint_Gravity_Update()` 目前已被注释。

它与 §6.2/6.3 的 RNEA 使用**不同的 DH 约定和单位**，不要混用参数。

### 6.6 其他算法模块

| 文件 | 功能 |
|---|---|
| `alg_pid.h/.cpp`、`PID.h/.cpp` | 位置/速度环 PID |
| `alg_filter.h/.cpp`、`kalman_filter.*`、`my_kalman.*` | 滤波/Kalman |
| `alg_MahonyAHRS.*`、`QuaternionEKF.*` | 姿态解算（互补滤波、四元数 EKF） |
| `alg_fsm.*` | 有限状态机（整车交互用） |
| `alg_slope.*` | 斜坡/速度规划 |
| `alg_power_limit.*`、`alg_new_power_limit.*` | 功率限制 |
| `alg_SMC_Control.*` | 滑模控制 |
| `matrix.h`、`Matrix.hpp`、`robotics.*` | 旧版矩阵与机器人学库（标准 DH、正逆解、RNE） |
| `RLS.hpp` | 递推最小二乘 |
| `utils.*`、`user_lib.*` | 通用数学/工具函数 |

---

### 6.7 J5 堵转寻零状态机

上电初态为 `6 IDLE`（未触发），保持反馈观测，不发送J5运动控制帧。Ozone中把 `Joint5_Calibration_Start_Request` 写为 `true`，任务会清零该请求并调用 `Joint5_DM_Motor.Request_Recalibration()` 开始校准。其它代码也可直接调用该接口（与1ms任务串行执行）。仅第一次启动时，也可将实际成员 `Joint5_DM_Motor.FSM_Calibration.Now_Status_Serial` 从6改为0；全局 `Joint5_Calibration_Status` 是观测镜像，不是控制入口。原0..5状态编号不变，持续时间判定保留。


`Joint5_Feedback_Torque_Nm` 是供Ozone直接读取的带符号反馈力矩，每次J5 CAN反馈解码后更新，数据来自 `Get_Now_Torque()`。校准使用其同源数据的绝对值与阈值比较；反馈停止时此变量保留最后值。

Ozone可直接添加全局变量 `Joint5_Calibration_Initialized`（初始化绑定已执行）、`Joint5_Calibration_Parameter_Valid`（最近一次启动参数是否已写入）、`Joint5_Calibration_Status`（状态0..6）和 `Joint5_Calibration_State_Time_Ms`（当前状态计时）。它们在Init及周期任务中更新，无需调用成员函数。启动等待2s内状态计时为0；参数接口不做校验，直接写入调用方数值；保留的bool返回值始终为true。底层MIT帧打包仍按协议量程处理。


`Class_Joint_DM_Gear::FSM_Calibration` 沿用 Gimbal 的写法：继承 `Class_FSM`、保存关节对象指针、声明友元，在 1 ms 回调中通过 `Set_Status()` 转移。

- 状态 `0 WAIT`：等待可用的 MIT 控制与电机反馈，保存校准前的控制模式和增益。
- 状态 `1 ROTATE`：MIT速度旋转，反馈力矩绝对值超过阈值后进入疑似堵转状态。
- 状态 `2 STALL_DETECT`：保持旋转，用 `Status[Now_Status_Serial].Time` 累计持续时间；力矩回落立即返回旋转状态，`Set_Status()` 自动清零离开状态的计时。即使刚好达到时限，也优先检查力矩回落。
- 状态 `3 SET_ZERO`：记录反馈角为偏移，当前关节角和目标关节角置0，恢复校准前的模式和增益。
- 状态 `4 COMPLETE`：校准完成，允许正常位置目标。
- 状态 `5 OFFLINE`：离线、失能、故障或非MIT模式保护，停止校准输出；恢复后，尚未校准则返回等待，已经校准则返回完成并保留零点。

状态机采用 Chariot 遥控器离线保护的显式保护/恢复分支，以及 Booster 堵转状态机的“正常 → 疑似堵转 → 确认处理”结构。没有独立的堵转计数器。

`Task_Init()` 中配置：

```cpp
Joint5_DM_Motor.Set_Calibration_Parameter(0.2f, 0.1f, 200, 1.0f);
// MIT目标速度rad/s（符号决定方向）、反馈力矩阈值、持续时间ms、MIT Kd
```

MIT速度控制在堵转时产生的力矩约为 `Kd * abs(omega)`，阈值需小于可达到的堵转反馈力矩。力矩阈值单位与 `Get_Now_Torque()` 相同。参数修改后连续计时重新开始。

CAN回调在驱动解码后调用 `Update_Joint_Feedback()`；任务每1 ms调用 `Joint5_DM_Motor.TIM_Process_PeriodElapsedCallback(!heartbeat_slot)`，其内部执行FSM和电机控制。反馈50 ms未更新、失能或故障会进入离线保护，打断未完成校准的连续计时。堵转确认要求本周期有新反馈。

本类角度定义改为 `q_joint = gain * (feedback - bias)`，目标反算为 `feedback_target = q_target / gain + bias`。这里只改变关节坐标零点，不发送电机硬件零位保存命令。底层 S2325 的已有多圈/25:1反馈换算保持原样。

Ozone可观察 `Joint5_DM_Motor.FSM_Calibration.Now_Status_Serial`（0..6）、`FSM_Calibration.Status[2].Time`、`Joint_Angle_Bias_Rad`、`Now_Joint_Angle_Rad`。调用 `Request_Recalibration()` 可重新寻零。校准期间常规关节目标不会覆盖寻零指令；J5初始使用原生 `MIT_POSITION` 模式，但FSM保持IDLE直到收到请求。

主机验证：`python User/Algorithm/Tests/validate_joint_calibration.py --compiler <本机g++路径>`。

---

## 7. 参数配置

| 参数 | 位置 | 说明 |
|---|---|---|
| `Joint_MDH_Parameter[]` | `User/Task/Src/tsk_config_and_callback.cpp` | 改进 DH 表（`{a, α, d, θ}`，下标 0 为基座），长度单位 m |
| `RNEA.Set_RNEA_Joint_Parameter(...)` | `Task_Init()` | 写入各连杆质量（kg）、质心（m）、惯量（暂未用） |
| `RNEA.Set_Gravity_RNEA(...)` | `Task_Init()` | 基座系重力向量，当前 `(0, −9.81, 0)` |
| `Joint.Init(..., torque_max)` | `Task_Init()` | 电机力矩满量程 `Torque_Max`，需与驱动器 TMAX 一致 |
| `Set_Joint_Angle_Gain/Bias` | `Task_Init()` | 关节角 ↔ 电机角换算、零位 |
| `scaler1..3` | `User/Task/Src/tsk_config_and_callback.cpp` | 关节力矩 → 电机力矩的缩放/符号（调试参数） |
| `test_j*_angle_deg` | 同上 | 台架测试目标角（volatile，可在调试器里改） |
| `J*_CAN_ID` / 限幅宏 | `User/Congfig/Inc/config.h` | CAN ID、位置/速度/力矩上限 |

单位约定：

- 长度 m、质量 kg、力矩 N·m、角度 rad；
- mDH 的 `a、d` 与 `Mass_Center` 单位一致（m）；
- 旧版 `Class_Gravity_Comp` 的 `Mass` 是电流单位，不要与 RNEA 的 kg 混用。

---

## 8. 调试与观测

在 Ozone 的 **Live Watch** 中可观察：

| 变量 | 含义 |
|---|---|
| `Joint_Gravity_Torque_RNEA_Nm[1..6]` | RNEA 重力补偿力矩（N·m） |
| `RNEA_Calc_Time_Us` | 本次解算耗时（µs） |
| `RNEA_Calc_Time_Us_Min/Max/Avg` | 最小/最大/平均耗时 |
| `RNEA_Calc_Count` | 统计次数 |
| `RNEA.Joint_Parameter[]` | 各连杆 MDH/质量/质心（可核对配置） |
| `RNEA.Homo_Trans_Matrix_Global[]` | 各连杆全局位姿 |
| `RNEA.Joint_Force_N[]`、`RNEA.Joint_Moment_Nm[]` | 递推中间量 |
| `test_j*_angle_deg` | 测试目标角（改这些变量可做静态姿态实验） |
| `Joint_Gravity_Torque_3J_Nm` | 旧版重力补偿观察数组（当前未启用） |

注意事项：

- 控制回调在 **1 kHz 中断**里运行，**不要在里面打断点**，否则会不断停机；用 Live Watch/采样；
- 上电后约 2 s 才开始解算（启动门限）；
- 解算耗时统计用 DWT `CYCCNT`，按 480 MHz 换算（`us = cycles / 480`）；
- 当前 RNEA 的关节角是在解算**之后**用实测角更新的，所以解算结果对应上一毫秒的姿态（1 ms 滞后）；
- J4~J6 的关节角未接入，保持 0。

---

## 9. 当前状态与限制

1. **已接入 5 个关节**（J1~J4 DM-J4340、J5 DM-S2325），RNEA 预留 6 个槽位。
2. **RNEA 模型目前只配置了 J1~J3** 的质量/质心；J4、J5 质量/质心为 0，J4~J6 的 MDH 为 0。J1/J2 的补偿力矩会少算远端连杆重量。
3. **MDH 表沿用旧模型数值**：这些数值原本用于标准 DH（`robotics::Link`），而 RNEA 使用改进 DH（`a_{i-1}/α_{i-1}` 索引）。两者约定不同，做精确补偿前需要核对/重新标定。
4. **J1~J4 控制帧未下发**：RNEA 补偿只写入 `Target_Torque`。J5 的手动触发校准已接入原生 MIT 控制，触发后实际发帧。
5. **`scalers` 仍是调试值**，J1 还按传动比做了除法，最终符号/增益需要实测确认。
6. **性能现状**：Debug（`-O0`）下 RNEA 解算约 200 µs/周期（实测），约占 1 kHz 周期的 20%；工程未开 I-Cache，且小矩阵/小向量大量走 CMSIS 库函数调用，仍有较大优化空间（见 §10）。
7. **旧模块保留但未接入**：底盘、云台、发射机构、裁判系统、整车 FSM（`Chariot`、`Interaction`）仍在仓库中，当前任务层没有调用；`crt_robot_arm.cpp` 未参与编译。
8. **主机测试**：`User/Algorithm/Tests/` 下有 Python + 本机 g++ 的验证脚本；其中部分脚本（如 `validate_rnea.py`、`test_alg_rnea.cpp`）针对的是早期 RNEA 接口，使用前请确认与当前 `Class_RNEA` API 一致。

---

## 10. 扩展与优化建议

### 10.1 增加/修改关节

1. 在 `Joint_MDH_Parameter[]` 增加对应行（注意 `a/α` 使用上一行索引）；
2. 在 `Task_Init()` 里调用 `Set_RNEA_Joint_Parameter()` 写入质量、质心；
3. 在 1 kHz 回调里把该关节的实测角写入 `Set_RNEA_Joint_Angle_Rad()`；
4. 如需 `JOINT_NUM` 大于 6，修改 `alg_RNEA.h` 的宏；
5. 补全电机实例、限位、使能和心跳。

### 10.2 补偿精度标定

- **质量/质心**：重力矩对 `m_i`、`m_i·c_i` 是线性的，建议用静态多点最小二乘辨识，而不是手工试凑；
- **T_MAX**：`Joint.Init(..., torque_max)` 必须与驱动器 TMAX 完全一致，否则是全局增益误差；
- **关节零位/方向**：先用水平仪或已知姿态校准 `Bias/Gain`；
- **传动比/符号**：`scaler` 要包含关节→电机的传动比和符号（J1 的 `Joint_Angle_Gain = -17/28`）；
- **远端负载**：补 J4/J5 与末端工具参数，或在其锁死时等效合并到连杆 3。

### 10.3 性能优化（当前约 200 µs @ -O0）

- 用 `-O2` 构建（保留 `-g` 便于调试）；
- 开 I-Cache（`SCB_EnableICache()`，D-Cache 需配合 MPU 与 DMA 一致性）；
- 减少小矩阵/小向量的 CMSIS 调用与临时对象（4×4、3×3、3×1 直接展开）；
- 预计算常量（`sin/cos α`、`a_{i-1}`、`d·sinα`、`d·cosα`、`m·g`）；
- 跳过未配置的关节；
- 解算可以移出 1 kHz 中断，或降低到 200~500 Hz（重力补偿变化较慢）。

### 10.4 安全注意事项

- 首次上电/调试时保持机械臂空载、低速、限幅；
- 控制帧发送恢复前，先确认 `scaler`、T_MAX、符号和限幅；
- 保留 2 s 启动等待和心跳/使能帧机制；
- 修改 DH/质量参数后，先在 Ozone 里核对 `Homo_Trans_Matrix_Global` 和补偿力矩，再下发。

---

## 11. 参考

- CMSIS-DSP 文档（`arm_mat_*`、`arm_sin_f32/arm_cos_f32`、`arm_dot_prod_f32` 等）。
- John J. Craig, *Introduction to Robotics: Mechanics and Control*（改进 DH 约定）。
- 仓库头文件中保留的 RoboMaster 生态框架说明（JiaoLong、USTC-ROBOWALKER 等原始版权）。
# Robot_Arm_Clion
