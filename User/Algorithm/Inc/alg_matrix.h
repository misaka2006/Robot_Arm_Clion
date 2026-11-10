//
// Created by minec on 2026/10/25.
//
// 矩阵与三维矢量运算类。
// 数值类型统一为 float，底层数学运算全部调用 CMSIS-DSP（arm_math）的 f32 接口，
// 对应工程中链接的 libarm_cortexM7lfsp_math.a，可直接使用 STM32H7 的 FPU。
//
// 典型用法：
//   Class_Matrix R = {{1.0f, 0.0f, 0.0f},
//                     {0.0f, 1.0f, 0.0f},
//                     {0.0f, 0.0f, 1.0f}};   // 花括号初始化
//   Class_Vector_3D p(1.0f, 2.0f, 3.0f);       // 矢量初始化
//   Class_Vector_3D q = R * p;                 // 矩阵与矢量的乘积
//   float norm = q.Get_Norm();                 // 模长
//   Class_Vector_3D u = q.Get_Normalized();    // 单位矢量
//   float dot = u.Dot(p);                      // 点乘
//   Class_Vector_3D cross = u.Cross(p);        // 叉乘
//   Class_Matrix Rt = R.Transpose();           // 转置
//   Class_Matrix R2 = Rt * R;                  // 矩阵乘法
//   Class_Matrix R3 = 2.0f * R2;               // 数乘
//
// 说明：矩阵的行列数在运行时指定，数据缓冲区在编译期固定，不在单片机上进行动态内存申请。
//       尺寸不匹配、访问越界等属于编程错误：Debug 下由 assert 触发定位，Release 下返回无效矩阵（0×0）。
//

#ifndef ROBOT_ARM_CLION_ALG_MATRIX_H
#define ROBOT_ARM_CLION_ALG_MATRIX_H

#include <initializer_list>
#include <stdint.h>

#include "arm_math.h"

/* ============================ 编译期配置 ============================ */
// 矩阵行列数上限。数据缓冲区按上限静态分配，因此每个矩阵对象占用
// ALG_MATRIX_MAX_ROWS * ALG_MATRIX_MAX_COLS * sizeof(float) 字节（默认 8×8=256 字节）。
// 需要更大的矩阵时，重新定义这两个宏即可（例如编译选项 -DALG_MATRIX_MAX_ROWS=10）。
#ifndef ALG_MATRIX_MAX_ROWS
#define ALG_MATRIX_MAX_ROWS 8
#endif

#ifndef ALG_MATRIX_MAX_COLS
#define ALG_MATRIX_MAX_COLS 8
#endif

class Class_Vector_3D;

/**
 * @brief 矩阵类（float，行主序），内部持有 arm_matrix_instance_f32。
 */
class Class_Matrix
{
public:
    /* ---------------------------- 构造与析构 ---------------------------- */
    // 默认构造：0×0 的空矩阵，Is_Valid() 为 false，需要 Init() 后才能参与运算。
    Class_Matrix(void);
    // 构造 rows×cols 的矩阵，所有元素填充为 value。
    Class_Matrix(uint32_t rows, uint32_t cols, float value = 0.0f);
    // 花括号初始化，例如 Class_Matrix m = {{1, 2}, {3, 4}}; 每行列数必须一致。
    Class_Matrix(std::initializer_list<std::initializer_list<float>> data);
    // 拷贝构造函数（重新绑定内部 arm 实例到自己的缓冲区）。
    Class_Matrix(const Class_Matrix &matrix);
    // 拷贝赋值。
    Class_Matrix &operator=(const Class_Matrix &matrix);

    /* ------------------------------ 初始化 ------------------------------ */
    // 重新指定尺寸并填充；尺寸非法（为 0 或超过上限）时返回 false，矩阵保持不变。
    bool Init(uint32_t rows, uint32_t cols, float value = 0.0f);
    // 用花括号数据重新初始化；每行列数不一致时返回 false。
    bool Init(std::initializer_list<std::initializer_list<float>> data);
    // 清零 / 置单位阵（行列数保持不变）。
    bool Set_Zero(void);
    bool Set_Identity(void);
    // 生成零矩阵 / 单位阵。
    static Class_Matrix Zero(uint32_t rows, uint32_t cols);
    static Class_Matrix Identity(uint32_t rows, uint32_t cols);

    /* --------------------------- 尺寸与状态 ---------------------------- */
    static uint32_t Get_Max_Rows(void);
    static uint32_t Get_Max_Cols(void);
    uint32_t Get_Rows(void) const;
    uint32_t Get_Cols(void) const;
    bool Is_Valid(void) const;
    bool Is_Square(void) const;

    /* ---------------------------- 元素访问 ---------------------------- */
    // 下标访问：matrix(row, col)，行优先，支持读写。
    float &operator()(uint32_t row, uint32_t col);
    const float &operator()(uint32_t row, uint32_t col) const;
    // 行访问：float *row_ptr = matrix[row]; 可用 matrix[row][col] 读写该行元素。
    float *operator[](uint32_t row);
    const float *operator[](uint32_t row) const;
    // 行主序连续数据缓冲区（可直接交给 arm_math 使用）。
    float *Get_Data(void);
    const float *Get_Data(void) const;
    // 内部 arm_matrix_instance_f32（可直接交给 arm_math 使用）。
    arm_matrix_instance_f32 *Get_Arm_Matrix(void);
    const arm_matrix_instance_f32 *Get_Arm_Matrix(void) const;

    /* ------------------------------ 数据拷贝 ---------------------------- */
    // 尺寸不变，从行主序数组拷贝 rows×cols 个元素。
    bool Copy_From_Array(const float *data);
    // 重新确定尺寸，并从行主序数组拷贝 rows×cols 个元素。
    bool Copy_From_Array(const float *data, uint32_t rows, uint32_t cols);

    /* ------------------------------ 算术运算 ---------------------------- */
    // 矩阵加减（要求尺寸相同）与数乘、数除。
    Class_Matrix &operator+=(const Class_Matrix &rhs);
    Class_Matrix &operator-=(const Class_Matrix &rhs);
    Class_Matrix &operator*=(float scalar);
    Class_Matrix &operator/=(float scalar);
    Class_Matrix operator+(const Class_Matrix &rhs) const;
    Class_Matrix operator-(const Class_Matrix &rhs) const;
    Class_Matrix operator*(float scalar) const;
    Class_Matrix operator/(float scalar) const;
    // 取负。
    Class_Matrix operator-(void) const;
    // 标量在左侧的数乘：result = scalar * matrix。
    friend Class_Matrix operator*(float scalar, const Class_Matrix &matrix);

    /* ------------------------------ 矩阵运算 ---------------------------- */
    // 矩阵乘法：*this 的列数必须等于 rhs 的行数。
    Class_Matrix operator*(const Class_Matrix &rhs) const;
    // 矩阵与三维矢量的乘积：*this 必须为 m×3（m 行 3 列），结果为 m×1 矩阵。
    // 若 *this 为 3×3，则可直接赋给 Class_Vector_3D（由矢量类的转换构造函数完成）。
    Class_Matrix operator*(const Class_Vector_3D &vector) const;
    // 转置。
    Class_Matrix Transpose(void) const;
    // 迹（方阵对角线元素之和）。
    float Get_Trace(void) const;
    // 逐元素比较。
    bool operator==(const Class_Matrix &rhs) const;
    bool operator!=(const Class_Matrix &rhs) const;

protected:
    // 将内部 arm 实例重新绑定到本对象的缓冲区与当前尺寸。
    void Update_Arm_Matrix(void);

private:
    // arm_math 矩阵实例，pData 始终指向 Data_。
    arm_matrix_instance_f32 Arm_Instance_;
    // 当前行列数（有效范围 1..ALG_MATRIX_MAX_ROWS / 1..ALG_MATRIX_MAX_COLS）。
    uint32_t Rows_ = 0;
    uint32_t Cols_ = 0;
    // 行主序数据缓冲区，按最大尺寸静态分配。
    float Data_[ALG_MATRIX_MAX_ROWS * ALG_MATRIX_MAX_COLS] = {0.0f};
};

/**
 * @brief 三维矢量类（float），内部同样以 3×1 的 arm_matrix_instance_f32 表示，
 *        因此可以直接参与 arm_math 的矩阵运算。
 */
class Class_Vector_3D
{
public:
    /* ---------------------------- 构造与赋值 ---------------------------- */
    // 默认构造：零矢量 (0, 0, 0)。
    Class_Vector_3D(void);
    // 由三个分量构造。
    Class_Vector_3D(float x, float y, float z);
    // 由长度为 3 的数组构造（按 x, y, z 顺序）。
    Class_Vector_3D(const float *data);
    // 由 3×1 或 1×3 矩阵构造（其他尺寸视为编程错误）。
    // 该构造函数非 explicit：可直接写 Class_Vector_3D v = matrix * vector;
    Class_Vector_3D(const Class_Matrix &matrix);
    // 拷贝构造与赋值。
    Class_Vector_3D(const Class_Vector_3D &vector);
    Class_Vector_3D &operator=(const Class_Vector_3D &vector);

    /* ------------------------------ 赋值操作 ---------------------------- */
    // 直接设置三个分量。
    bool Set(float x, float y, float z);
    // 从长度为 3 的数组赋值。
    bool Copy_From_Array(const float *data);
    // 从 3×1 或 1×3 矩阵赋值。
    bool Copy_From_Matrix(const Class_Matrix &matrix);

    /* ---------------------------- 单位矢量 ---------------------------- */
    static Class_Vector_3D Unit_X(void);
    static Class_Vector_3D Unit_Y(void);
    static Class_Vector_3D Unit_Z(void);

    /* ---------------------------- 元素访问 ---------------------------- */
    // 下标访问（0~2），支持读写。
    float &operator[](uint32_t index);
    const float &operator[](uint32_t index) const;
    float Get_X(void) const;
    float Get_Y(void) const;
    float Get_Z(void) const;
    // 长度为 3 的连续数据缓冲区。
    float *Get_Data(void);
    const float *Get_Data(void) const;
    // 内部 3×1 的 arm_matrix_instance_f32。
    arm_matrix_instance_f32 *Get_Arm_Matrix(void);
    const arm_matrix_instance_f32 *Get_Arm_Matrix(void) const;
    // 转换为 3×1 矩阵（列矢量）。
    Class_Matrix To_Matrix(void) const;
    bool To_Matrix(Class_Matrix *matrix) const;

    /* ------------------------------ 算术运算 ---------------------------- */
    Class_Vector_3D &operator+=(const Class_Vector_3D &rhs);
    Class_Vector_3D &operator-=(const Class_Vector_3D &rhs);
    Class_Vector_3D &operator*=(float scalar);
    Class_Vector_3D &operator/=(float scalar);
    Class_Vector_3D operator+(const Class_Vector_3D &rhs) const;
    Class_Vector_3D operator-(const Class_Vector_3D &rhs) const;
    Class_Vector_3D operator*(float scalar) const;
    Class_Vector_3D operator/(float scalar) const;
    // 取负。
    Class_Vector_3D operator-(void) const;
    // 标量在左侧的数乘：result = scalar * vector。
    friend Class_Vector_3D operator*(float scalar, const Class_Vector_3D &vector);

    /* ------------------------- 点乘、叉乘、模长 ------------------------- */
    // 点乘：a·b = |a||b|cosθ，结果为一个标量。
    float Dot(const Class_Vector_3D &rhs) const;
    // 叉乘：a×b，结果为一个矢量（右手定则）。
    Class_Vector_3D Cross(const Class_Vector_3D &rhs) const;
    // 模长的平方（避免开方，可用于比较大小）。
    float Get_Norm_Squared(void) const;
    // 模长：|v| = sqrt(v·v)。
    float Get_Norm(void) const;
    // 归一化（单位矢量），原地修改；零矢量返回 false 且保持不变。
    bool Normalize(void);
    // 返回单位矢量；零矢量返回零矢量。
    Class_Vector_3D Get_Normalized(void) const;

    /* ---------------------------- 比较运算 ---------------------------- */
    bool operator==(const Class_Vector_3D &rhs) const;
    bool operator!=(const Class_Vector_3D &rhs) const;

protected:
    // 将内部 arm 实例重新绑定到本对象的缓冲区。
    void Update_Arm_Matrix(void);

private:
    // arm_math 矩阵实例（3×1），pData 始终指向 Data_。
    arm_matrix_instance_f32 Arm_Instance_;
    // 分量缓冲区：Data_[0] = x, Data_[1] = y, Data_[2] = z。
    float Data_[3] = {0.0f, 0.0f, 0.0f};
};

#endif //ROBOT_ARM_CLION_ALG_MATRIX_H
