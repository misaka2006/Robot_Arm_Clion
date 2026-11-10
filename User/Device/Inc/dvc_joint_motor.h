/**
 * @file dvc_joint_motor.h
 * @brief 机械臂关节电机抽象接口，与具体电机和通信协议无关。
 */
#ifndef DVC_JOINT_MOTOR_H
#define DVC_JOINT_MOTOR_H

/**
 * @brief 用户继承此接口，适配实际电机（可组合或同时继承具体电机类）。
 *
 * 角度统一使用关节输出轴的弧度，反馈和目标必须采用相同零点、方向。
 * 减速比、编码器零点、多圈处理和协议单位转换均由派生类实现。
 * 电机初始化、接收回调、在线超时统计和校准仍由用户设备层负责。
 */
class Class_Joint_Motor
{
public:
    virtual ~Class_Joint_Motor() = default;

    // 返回通信反馈是否有效；不能用“电机已使能”代替在线状态。
    virtual bool Get_Online() const = 0;
    virtual float Get_Now_Radian() const = 0;

    // 缓存位置目标；派生类负责选择实际的位置控制模式和参数。
    virtual void Set_Target_Radian(float radian) = 0;
    // 设置使能请求；false 必须阻止后续周期继续执行旧运动目标。
    virtual void Set_Enable(bool enable) = 0;

    // 执行本周期控制/发送，包括失能请求；总线分时调度由派生类处理。
    virtual void TIM_Calculate_PeriodElapsedCallback() = 0;
};

#endif // DVC_JOINT_MOTOR_H
