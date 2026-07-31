#pragma once

#include "KPhysics/Utils.hpp"

#include <glm/glm.hpp>
#include <glm/mat3x3.hpp>

#include <glm/ext/matrix_transform.hpp>

BEGIN_NAMESPACE_KAYOU

using Mat3 = glm::mat3;

inline Mat3 Mat3Identity()
{
    return glm::identity<Mat3>();
}

inline Mat3 Transpose(const Mat3& matrix)
{
    return glm::transpose(matrix);
}

inline Mat3 Inverse(const Mat3& matrix)
{
    return glm::inverse(matrix);
}

inline float Determinant(const Mat3& matrix)
{
    return glm::determinant(matrix);
}

END_NAMESPACE_KAYOU