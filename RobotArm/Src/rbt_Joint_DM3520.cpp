#include "rbt_Joint_DM3520.h"

void Class_Joint_DM3520::Set_Target_Radian(float radian)
{
    Motor.Set_Target_Radian(radian);
}

void Class_Joint_DM3520::Set_Enable(bool enable)
{
    Motor.Set_Enable(enable);
}

bool Class_Joint_DM3520::Get_Online()
{
    return Motor.Get_Online();
}

float Class_Joint_DM3520::Get_Now_Radian()
{
    return Motor.Get_Now_Radian();
}

void Class_Joint_DM3520::TIM_Calculate_PeriodElapsedCallback()
{
    Motor.TIM_Calculate_PeriodElapsedCallback();
}
