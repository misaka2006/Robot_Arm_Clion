#include "alg_rnea.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

static void Require(bool condition, const char *message)
{
    if (!condition)
    {
        std::fprintf(stderr, "FAIL: %s\n", message);
        std::exit(1);
    }
}

static Struct_RNEA_Model Make_Model(int mode)
{
    Struct_RNEA_Model model;
    const float mass[4] = {0.8f, 0.6f, 0.4f, 0.2f};
    const float center[4][3] = {{-0.06f, 0.01f, 0.015f}, {-0.15f, 0.02f, -0.01f},
                               {0.03f, -0.02f, 0.04f}, {0.01f, 0.02f, 0.05f}};
    for (int i = 0; i < 4; i++)
    {
        model.Link[i].Mass = mass[i];
        for (int j = 0; j < 3; j++)
        {
            model.Link[i].First_Moment[j] = mode == 0 ? 0.0f : mass[i] * center[i][j];
        }
    }
    if (mode == 2)
    {
        model.Gravity_Base[1] = 0.0f;
    }
    if (mode == 3)
    {
        model.Gravity_Base[0] = 1.2f;
        model.Gravity_Base[1] = -8.7f;
        model.Gravity_Base[2] = -2.3f;
    }
    return model;
}

int main()
{
    Class_RNEA solver;
    float q[4] = {0.31f, -0.72f, 0.43f, 0.19f};
    Require(solver.Calculate_Gravity(q), "default model");
    for (int i = 0; i < 4; i++)
    {
        Require(solver.Get_Out(i) == 0.0f, "zero parameters must give zero torque");
    }

    //单摆静态解析解: tau = m*g*L*cos(q)。
    Struct_RNEA_Model pendulum;
    for (int i = 0; i < 4; i++)
    {
        pendulum.Link[i] = Struct_RNEA_Link();
    }
    pendulum.Link[0].A = 0.4f;
    pendulum.Link[0].Mass = 2.0f;
    Require(solver.Init(pendulum), "pendulum initialization");
    Require(solver.Calculate_Gravity(q), "pendulum evaluation");
    const float expected = 2.0f * 9.81f * 0.4f * std::cos(q[0]);
    Require(std::fabs(solver.Get_Out(0) - expected) < 3e-6f, "pendulum gravity sign and magnitude");
    Require(!solver.Calculate_Gravity(nullptr), "null q rejected");
    q[0] = std::numeric_limits<float>::quiet_NaN();
    Require(!solver.Calculate_Gravity(q), "NaN rejected");
    for (int i = 0; i < 4; i++)
    {
        Require(solver.Get_Out(i) == 0.0f, "failure clears torque");
    }
    Struct_RNEA_Model invalid = pendulum;
    invalid.Link[0].Mass = -1.0f;
    Require(!solver.Init(invalid), "negative mass rejected");
    Require(solver.Get_Model().Link[0].Mass == 2.0f, "failed Init preserves configuration");

    //运行时一阶矩更新：无需重新Init即可改变补偿，非法参数不能部分写入。
    q[0] = 0.31f;
    float moments[4][3] = {};
    moments[0][0] = 0.1f;
    Require(solver.Set_First_Moment(moments), "runtime moment update");
    Require(solver.Calculate_Gravity(q), "runtime moment evaluation");
    Require(std::fabs(solver.Get_Out(0) - (expected + 0.1f * 9.81f * std::cos(q[0]))) < 3e-6f,
            "runtime moment changes torque and preserves mass");
    moments[0][0] = 0.2f;
    moments[3][2] = std::numeric_limits<float>::infinity();
    Require(!solver.Set_First_Moment(moments), "invalid runtime moment rejected");
    Require(solver.Get_Model().Link[0].First_Moment[0] == 0.1f, "invalid update is atomic");
    for (int i = 0; i < 4; i++) Require(solver.Get_Out(i) == 0.0f, "invalid moment clears output");
    Require(!solver.Set_First_Moment(nullptr), "null moment rejected");

    //输出重力样本，供独立的双精度势能差分实现核对。
    for (int mode = 0; mode < 4; mode++)
    {
        const Struct_RNEA_Model fixture = Make_Model(mode);
        Struct_RNEA_Model zero_moment = fixture;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                moments[i][j] = fixture.Link[i].First_Moment[j];
                zero_moment.Link[i].First_Moment[j] = 0.0f;
            }
        }
        Require(solver.Init(zero_moment), "fixture initialization");
        Require(solver.Set_First_Moment(moments), "fixture runtime moment update");
        for (int sample = 0; sample < 60; sample++)
        {
            for (int j = 0; j < 4; j++)
            {
                q[j] = 1.5f * std::sin(0.37f * (sample + 1) * (j + 1));
            }
            Require(solver.Calculate_Gravity(q), "fixture evaluation");
            if (mode == 0)
            {
                Require(std::fabs(solver.Get_Out(3)) < 1e-6f, "zero first moment gives zero roll gravity");
            }
            if (mode == 2)
            {
                for (int j = 0; j < 4; j++)
                {
                    Require(solver.Get_Out(j) == 0.0f, "zero gravity gives zero torque");
                }
            }
            std::printf("%d", mode);
            for (int j = 0; j < 4; j++) std::printf(",%.9g", static_cast<double>(q[j]));
            for (int j = 0; j < 4; j++) std::printf(",%.9g", static_cast<double>(solver.Get_Out(j)));
            std::printf("\n");
        }
    }
    std::fprintf(stderr, "C++ pendulum, default, invalid-input, zero-gravity and roll checks passed.\n");
    std::fprintf(stderr, "Class_RNEA size: %u bytes.\n", static_cast<unsigned>(sizeof(Class_RNEA)));
    return 0;
}
