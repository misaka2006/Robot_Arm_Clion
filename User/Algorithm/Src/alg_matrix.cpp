//
// Created by minec on 2026/10/25.
//
// 矩阵与三维矢量运算类的实现。
// 所有数学运算均调用 CMSIS-DSP（arm_math）中对应的 float32 接口：
//   arm_mat_init_f32 / arm_mat_add_f32 / arm_mat_sub_f32 / arm_mat_scale_f32 /
//   arm_mat_mult_f32 / arm_mat_trans_f32 / arm_fill_f32 / arm_copy_f32 /
//   arm_add_f32 / arm_sub_f32 / arm_scale_f32 / arm_dot_prod_f32 /
//   arm_power_f32 / arm_sqrt_f32
//

#include "alg_matrix.h"

#include <assert.h>

// 归一化时判定零矢量的阈值。
static const float VECTOR_NORM_EPSILON = 1.0e-6f;

/* ======================================================================== */
/*                              Class_Matrix                                */
/* ======================================================================== */

Class_Matrix::Class_Matrix(void)
{
    Update_Arm_Matrix();
}

Class_Matrix::Class_Matrix(uint32_t rows, uint32_t cols, float value) : Class_Matrix()
{
    if (!Init(rows, cols, value))
    {
        assert(false);
    }
}

Class_Matrix::Class_Matrix(std::initializer_list<std::initializer_list<float>> data) : Class_Matrix()
{
    if (!Init(data))
    {
        assert(false);
    }
}

Class_Matrix::Class_Matrix(const Class_Matrix &matrix) : Class_Matrix()
{
    if (matrix.Is_Valid())
    {
        Rows_ = matrix.Rows_;
        Cols_ = matrix.Cols_;
        arm_copy_f32(matrix.Data_, Data_, Rows_ * Cols_);
        Update_Arm_Matrix();
    }
}

Class_Matrix &Class_Matrix::operator=(const Class_Matrix &matrix)
{
    if (this != &matrix)
    {
        Rows_ = matrix.Rows_;
        Cols_ = matrix.Cols_;
        if ((Rows_ != 0U) && (Cols_ != 0U))
        {
            arm_copy_f32(matrix.Data_, Data_, Rows_ * Cols_);
        }
        Update_Arm_Matrix();
    }
    return *this;
}

void Class_Matrix::Update_Arm_Matrix(void)
{
    // 让 arm_math 的矩阵实例始终指向本对象的缓冲区及当前行列数。
    arm_mat_init_f32(&Arm_Instance_, static_cast<uint16_t>(Rows_), static_cast<uint16_t>(Cols_), Data_);
}

/* ------------------------------ 初始化 ------------------------------ */

bool Class_Matrix::Init(uint32_t rows, uint32_t cols, float value)
{
    if ((rows == 0U) || (cols == 0U) ||
        (rows > static_cast<uint32_t>(ALG_MATRIX_MAX_ROWS)) ||
        (cols > static_cast<uint32_t>(ALG_MATRIX_MAX_COLS)))
    {
        return false;
    }

    Rows_ = rows;
    Cols_ = cols;
    arm_fill_f32(value, Data_, rows * cols);
    Update_Arm_Matrix();
    return true;
}

bool Class_Matrix::Init(std::initializer_list<std::initializer_list<float>> data)
{
    const uint32_t rows = static_cast<uint32_t>(data.size());
    if (rows == 0U)
    {
        return false;
    }

    const uint32_t cols = static_cast<uint32_t>(data.begin()->size());
    if (cols == 0U)
    {
        return false;
    }

    if (!Init(rows, cols, 0.0f))
    {
        return false;
    }

    uint32_t row = 0;
    for (const std::initializer_list<float> &row_data : data)
    {
        if (static_cast<uint32_t>(row_data.size()) != Cols_)
        {
            // 每行的列数必须一致；出错时恢复为无效矩阵（0×0）。
            Rows_ = 0;
            Cols_ = 0;
            Update_Arm_Matrix();
            assert(false);
            return false;
        }
        arm_copy_f32(row_data.begin(), &Data_[row * Cols_], Cols_);
        row++;
    }
    return true;
}

bool Class_Matrix::Set_Zero(void)
{
    if (!Is_Valid())
    {
        assert(false);
        return false;
    }
    arm_fill_f32(0.0f, Data_, Rows_ * Cols_);
    return true;
}

bool Class_Matrix::Set_Identity(void)
{
    if (!Is_Valid())
    {
        assert(false);
        return false;
    }

    arm_fill_f32(0.0f, Data_, Rows_ * Cols_);
    const uint32_t count = (Rows_ < Cols_) ? Rows_ : Cols_;
    for (uint32_t i = 0; i < count; i++)
    {
        Data_[(i * Cols_) + i] = 1.0f;
    }
    return true;
}

Class_Matrix Class_Matrix::Zero(uint32_t rows, uint32_t cols)
{
    return Class_Matrix(rows, cols, 0.0f);
}

Class_Matrix Class_Matrix::Identity(uint32_t rows, uint32_t cols)
{
    Class_Matrix result(rows, cols, 0.0f);
    result.Set_Identity();
    return result;
}

/* --------------------------- 尺寸与状态 ---------------------------- */

uint32_t Class_Matrix::Get_Max_Rows(void)
{
    return static_cast<uint32_t>(ALG_MATRIX_MAX_ROWS);
}

uint32_t Class_Matrix::Get_Max_Cols(void)
{
    return static_cast<uint32_t>(ALG_MATRIX_MAX_COLS);
}

uint32_t Class_Matrix::Get_Rows(void) const
{
    return Rows_;
}

uint32_t Class_Matrix::Get_Cols(void) const
{
    return Cols_;
}

bool Class_Matrix::Is_Valid(void) const
{
    return (Rows_ != 0U) && (Cols_ != 0U);
}

bool Class_Matrix::Is_Square(void) const
{
    return Is_Valid() && (Rows_ == Cols_);
}

/* ---------------------------- 元素访问 ---------------------------- */

float &Class_Matrix::operator()(uint32_t row, uint32_t col)
{
    assert((row < Rows_) && (col < Cols_));
    return Data_[(row * Cols_) + col];
}

const float &Class_Matrix::operator()(uint32_t row, uint32_t col) const
{
    assert((row < Rows_) && (col < Cols_));
    return Data_[(row * Cols_) + col];
}

float *Class_Matrix::operator[](uint32_t row)
{
    assert(row < Rows_);
    return &Data_[row * Cols_];
}

const float *Class_Matrix::operator[](uint32_t row) const
{
    assert(row < Rows_);
    return &Data_[row * Cols_];
}

float *Class_Matrix::Get_Data(void)
{
    return Data_;
}

const float *Class_Matrix::Get_Data(void) const
{
    return Data_;
}

arm_matrix_instance_f32 *Class_Matrix::Get_Arm_Matrix(void)
{
    return &Arm_Instance_;
}

const arm_matrix_instance_f32 *Class_Matrix::Get_Arm_Matrix(void) const
{
    return &Arm_Instance_;
}

/* ------------------------------ 数据拷贝 ---------------------------- */

bool Class_Matrix::Copy_From_Array(const float *data)
{
    if ((data == nullptr) || !Is_Valid())
    {
        assert(false);
        return false;
    }
    arm_copy_f32(data, Data_, Rows_ * Cols_);
    return true;
}

bool Class_Matrix::Copy_From_Array(const float *data, uint32_t rows, uint32_t cols)
{
    if (data == nullptr)
    {
        assert(false);
        return false;
    }

    if (!Init(rows, cols, 0.0f))
    {
        assert(false);
        return false;
    }

    arm_copy_f32(data, Data_, rows * cols);
    return true;
}

/* ------------------------------ 算术运算 ---------------------------- */

Class_Matrix &Class_Matrix::operator+=(const Class_Matrix &rhs)
{
    if (!Is_Valid() || !rhs.Is_Valid() || (Rows_ != rhs.Rows_) || (Cols_ != rhs.Cols_))
    {
        assert(false);
        return *this;
    }
    arm_mat_add_f32(&Arm_Instance_, &rhs.Arm_Instance_, &Arm_Instance_);
    return *this;
}

Class_Matrix &Class_Matrix::operator-=(const Class_Matrix &rhs)
{
    if (!Is_Valid() || !rhs.Is_Valid() || (Rows_ != rhs.Rows_) || (Cols_ != rhs.Cols_))
    {
        assert(false);
        return *this;
    }
    arm_mat_sub_f32(&Arm_Instance_, &rhs.Arm_Instance_, &Arm_Instance_);
    return *this;
}

Class_Matrix &Class_Matrix::operator*=(float scalar)
{
    if (!Is_Valid())
    {
        assert(false);
        return *this;
    }
    arm_mat_scale_f32(&Arm_Instance_, scalar, &Arm_Instance_);
    return *this;
}

Class_Matrix &Class_Matrix::operator/=(float scalar)
{
    if (!Is_Valid() || (scalar == 0.0f))
    {
        assert(false);
        return *this;
    }
    arm_mat_scale_f32(&Arm_Instance_, 1.0f / scalar, &Arm_Instance_);
    return *this;
}

Class_Matrix Class_Matrix::operator+(const Class_Matrix &rhs) const
{
    Class_Matrix result;
    if (!Is_Valid() || !rhs.Is_Valid() || (Rows_ != rhs.Rows_) || (Cols_ != rhs.Cols_))
    {
        assert(false);
        return result;
    }

    result.Init(Rows_, Cols_, 0.0f);
    arm_mat_add_f32(&Arm_Instance_, &rhs.Arm_Instance_, &result.Arm_Instance_);
    return result;
}

Class_Matrix Class_Matrix::operator-(const Class_Matrix &rhs) const
{
    Class_Matrix result;
    if (!Is_Valid() || !rhs.Is_Valid() || (Rows_ != rhs.Rows_) || (Cols_ != rhs.Cols_))
    {
        assert(false);
        return result;
    }

    result.Init(Rows_, Cols_, 0.0f);
    arm_mat_sub_f32(&Arm_Instance_, &rhs.Arm_Instance_, &result.Arm_Instance_);
    return result;
}

Class_Matrix Class_Matrix::operator*(float scalar) const
{
    Class_Matrix result;
    if (!Is_Valid())
    {
        assert(false);
        return result;
    }

    result.Init(Rows_, Cols_, 0.0f);
    arm_mat_scale_f32(&Arm_Instance_, scalar, &result.Arm_Instance_);
    return result;
}

Class_Matrix Class_Matrix::operator/(float scalar) const
{
    Class_Matrix result;
    if (!Is_Valid() || (scalar == 0.0f))
    {
        assert(false);
        return result;
    }

    result.Init(Rows_, Cols_, 0.0f);
    arm_mat_scale_f32(&Arm_Instance_, 1.0f / scalar, &result.Arm_Instance_);
    return result;
}

Class_Matrix Class_Matrix::operator-(void) const
{
    return (*this) * (-1.0f);
}

Class_Matrix operator*(float scalar, const Class_Matrix &matrix)
{
    return matrix * scalar;
}

/* ------------------------------ 矩阵运算 ---------------------------- */

Class_Matrix Class_Matrix::operator*(const Class_Matrix &rhs) const
{
    Class_Matrix result;
    if (!Is_Valid() || !rhs.Is_Valid() || (Cols_ != rhs.Rows_))
    {
        assert(false);
        return result;
    }

    if (!result.Init(Rows_, rhs.Cols_, 0.0f))
    {
        assert(false);
        return result;
    }

    arm_mat_mult_f32(&Arm_Instance_, &rhs.Arm_Instance_, &result.Arm_Instance_);
    return result;
}

Class_Matrix Class_Matrix::operator*(const Class_Vector_3D &vector) const
{
    Class_Matrix result;
    if (!Is_Valid() || (Cols_ != 3U))
    {
        assert(false);
        return result;
    }

    if (!result.Init(Rows_, 1U, 0.0f))
    {
        assert(false);
        return result;
    }

    arm_mat_mult_f32(&Arm_Instance_, vector.Get_Arm_Matrix(), &result.Arm_Instance_);
    return result;
}

Class_Matrix Class_Matrix::Transpose(void) const
{
    Class_Matrix result;
    if (!Is_Valid())
    {
        assert(false);
        return result;
    }

    if (!result.Init(Cols_, Rows_, 0.0f))
    {
        assert(false);
        return result;
    }

    arm_mat_trans_f32(&Arm_Instance_, &result.Arm_Instance_);
    return result;
}

float Class_Matrix::Get_Trace(void) const
{
    if (!Is_Valid())
    {
        assert(false);
        return 0.0f;
    }

    const uint32_t count = (Rows_ < Cols_) ? Rows_ : Cols_;
    float trace = 0.0f;
    for (uint32_t i = 0; i < count; i++)
    {
        trace += Data_[(i * Cols_) + i];
    }
    return trace;
}

bool Class_Matrix::operator==(const Class_Matrix &rhs) const
{
    if ((Rows_ != rhs.Rows_) || (Cols_ != rhs.Cols_))
    {
        return false;
    }

    const uint32_t count = Rows_ * Cols_;
    for (uint32_t i = 0; i < count; i++)
    {
        if (Data_[i] != rhs.Data_[i])
        {
            return false;
        }
    }
    return true;
}

bool Class_Matrix::operator!=(const Class_Matrix &rhs) const
{
    return !(*this == rhs);
}

/* ======================================================================== */
/*                            Class_Vector_3D                               */
/* ======================================================================== */

Class_Vector_3D::Class_Vector_3D(void)
{
    Update_Arm_Matrix();
}

Class_Vector_3D::Class_Vector_3D(float x, float y, float z) : Class_Vector_3D()
{
    Set(x, y, z);
}

Class_Vector_3D::Class_Vector_3D(const float *data) : Class_Vector_3D()
{
    if (!Copy_From_Array(data))
    {
        assert(false);
    }
}

Class_Vector_3D::Class_Vector_3D(const Class_Matrix &matrix) : Class_Vector_3D()
{
    if (!Copy_From_Matrix(matrix))
    {
        assert(false);
    }
}

Class_Vector_3D::Class_Vector_3D(const Class_Vector_3D &vector) : Class_Vector_3D()
{
    arm_copy_f32(vector.Data_, Data_, 3);
}

Class_Vector_3D &Class_Vector_3D::operator=(const Class_Vector_3D &vector)
{
    if (this != &vector)
    {
        arm_copy_f32(vector.Data_, Data_, 3);
    }
    return *this;
}

void Class_Vector_3D::Update_Arm_Matrix(void)
{
    // 内部始终是一个 3×1 的列矢量。
    arm_mat_init_f32(&Arm_Instance_, 3U, 1U, Data_);
}

/* ------------------------------ 赋值操作 ---------------------------- */

bool Class_Vector_3D::Set(float x, float y, float z)
{
    Data_[0] = x;
    Data_[1] = y;
    Data_[2] = z;
    return true;
}

bool Class_Vector_3D::Copy_From_Array(const float *data)
{
    if (data == nullptr)
    {
        assert(false);
        return false;
    }
    arm_copy_f32(data, Data_, 3);
    return true;
}

bool Class_Vector_3D::Copy_From_Matrix(const Class_Matrix &matrix)
{
    if (!matrix.Is_Valid())
    {
        assert(false);
        return false;
    }

    const uint32_t rows = matrix.Get_Rows();
    const uint32_t cols = matrix.Get_Cols();
    if (!(((rows == 3U) && (cols == 1U)) || ((rows == 1U) && (cols == 3U))))
    {
        // 只有 3×1 的列矢量或 1×3 的行矢量才能转换为三维矢量。
        assert(false);
        return false;
    }

    arm_copy_f32(matrix.Get_Data(), Data_, 3);
    return true;
}

Class_Vector_3D Class_Vector_3D::Unit_X(void)
{
    return Class_Vector_3D(1.0f, 0.0f, 0.0f);
}

Class_Vector_3D Class_Vector_3D::Unit_Y(void)
{
    return Class_Vector_3D(0.0f, 1.0f, 0.0f);
}

Class_Vector_3D Class_Vector_3D::Unit_Z(void)
{
    return Class_Vector_3D(0.0f, 0.0f, 1.0f);
}

/* ---------------------------- 元素访问 ---------------------------- */

float &Class_Vector_3D::operator[](uint32_t index)
{
    assert(index < 3U);
    return Data_[index];
}

const float &Class_Vector_3D::operator[](uint32_t index) const
{
    assert(index < 3U);
    return Data_[index];
}

float Class_Vector_3D::Get_X(void) const
{
    return Data_[0];
}

float Class_Vector_3D::Get_Y(void) const
{
    return Data_[1];
}

float Class_Vector_3D::Get_Z(void) const
{
    return Data_[2];
}

float *Class_Vector_3D::Get_Data(void)
{
    return Data_;
}

const float *Class_Vector_3D::Get_Data(void) const
{
    return Data_;
}

arm_matrix_instance_f32 *Class_Vector_3D::Get_Arm_Matrix(void)
{
    return &Arm_Instance_;
}

const arm_matrix_instance_f32 *Class_Vector_3D::Get_Arm_Matrix(void) const
{
    return &Arm_Instance_;
}

Class_Matrix Class_Vector_3D::To_Matrix(void) const
{
    Class_Matrix result(3U, 1U, 0.0f);
    arm_copy_f32(Data_, result.Get_Data(), 3);
    return result;
}

bool Class_Vector_3D::To_Matrix(Class_Matrix *matrix) const
{
    if (matrix == nullptr)
    {
        assert(false);
        return false;
    }

    if (!matrix->Init(3U, 1U, 0.0f))
    {
        assert(false);
        return false;
    }

    arm_copy_f32(Data_, matrix->Get_Data(), 3);
    return true;
}

/* ------------------------------ 算术运算 ---------------------------- */

Class_Vector_3D &Class_Vector_3D::operator+=(const Class_Vector_3D &rhs)
{
    arm_add_f32(Data_, rhs.Data_, Data_, 3);
    return *this;
}

Class_Vector_3D &Class_Vector_3D::operator-=(const Class_Vector_3D &rhs)
{
    arm_sub_f32(Data_, rhs.Data_, Data_, 3);
    return *this;
}

Class_Vector_3D &Class_Vector_3D::operator*=(float scalar)
{
    arm_scale_f32(Data_, scalar, Data_, 3);
    return *this;
}

Class_Vector_3D &Class_Vector_3D::operator/=(float scalar)
{
    if (scalar == 0.0f)
    {
        assert(false);
        return *this;
    }
    arm_scale_f32(Data_, 1.0f / scalar, Data_, 3);
    return *this;
}

Class_Vector_3D Class_Vector_3D::operator+(const Class_Vector_3D &rhs) const
{
    Class_Vector_3D result;
    arm_add_f32(Data_, rhs.Data_, result.Data_, 3);
    return result;
}

Class_Vector_3D Class_Vector_3D::operator-(const Class_Vector_3D &rhs) const
{
    Class_Vector_3D result;
    arm_sub_f32(Data_, rhs.Data_, result.Data_, 3);
    return result;
}

Class_Vector_3D Class_Vector_3D::operator*(float scalar) const
{
    Class_Vector_3D result;
    arm_scale_f32(Data_, scalar, result.Data_, 3);
    return result;
}

Class_Vector_3D Class_Vector_3D::operator/(float scalar) const
{
    Class_Vector_3D result;
    if (scalar == 0.0f)
    {
        assert(false);
        return result;
    }
    arm_scale_f32(Data_, 1.0f / scalar, result.Data_, 3);
    return result;
}

Class_Vector_3D Class_Vector_3D::operator-(void) const
{
    Class_Vector_3D result;
    arm_scale_f32(Data_, -1.0f, result.Data_, 3);
    return result;
}

Class_Vector_3D operator*(float scalar, const Class_Vector_3D &vector)
{
    return vector * scalar;
}

/* ------------------------- 点乘、叉乘、模长 ------------------------- */

float Class_Vector_3D::Dot(const Class_Vector_3D &rhs) const
{
    float result = 0.0f;
    arm_dot_prod_f32(Data_, rhs.Data_, 3, &result);
    return result;
}

Class_Vector_3D Class_Vector_3D::Cross(const Class_Vector_3D &rhs) const
{
    // a×b = [a]×·b，其中 [a]× 为 a 的反对称矩阵：
    //   [  0  -az   ay ]
    //   [ az    0  -ax ]
    //   [-ay   ax    0 ]
    // 反对称矩阵与矢量的乘积直接调用 arm_mat_mult_f32 完成。
    float skew_data[9] =
    {
         0.0f, -Data_[2],  Data_[1],
         Data_[2],  0.0f, -Data_[0],
        -Data_[1],  Data_[0], 0.0f
    };

    arm_matrix_instance_f32 skew_matrix;
    arm_mat_init_f32(&skew_matrix, 3U, 3U, skew_data);

    Class_Vector_3D result;
    arm_mat_mult_f32(&skew_matrix, &rhs.Arm_Instance_, &result.Arm_Instance_);
    return result;
}

float Class_Vector_3D::Get_Norm_Squared(void) const
{
    float result = 0.0f;
    arm_power_f32(Data_, 3, &result);
    return result;
}

float Class_Vector_3D::Get_Norm(void) const
{
    float result = 0.0f;
    if (arm_sqrt_f32(Get_Norm_Squared(), &result) != ARM_MATH_SUCCESS)
    {
        result = 0.0f;
    }
    return result;
}

bool Class_Vector_3D::Normalize(void)
{
    const float norm = Get_Norm();
    if (norm < VECTOR_NORM_EPSILON)
    {
        // 零矢量没有确定的方向，保持原值不变。
        return false;
    }

    arm_scale_f32(Data_, 1.0f / norm, Data_, 3);
    return true;
}

Class_Vector_3D Class_Vector_3D::Get_Normalized(void) const
{
    Class_Vector_3D result;
    const float norm = Get_Norm();
    if (norm < VECTOR_NORM_EPSILON)
    {
        return result;
    }

    arm_scale_f32(Data_, 1.0f / norm, result.Data_, 3);
    return result;
}

/* ---------------------------- 比较运算 ---------------------------- */

bool Class_Vector_3D::operator==(const Class_Vector_3D &rhs) const
{
    return (Data_[0] == rhs.Data_[0]) &&
           (Data_[1] == rhs.Data_[1]) &&
           (Data_[2] == rhs.Data_[2]);
}

bool Class_Vector_3D::operator!=(const Class_Vector_3D &rhs) const
{
    return !(*this == rhs);
}
