#pragma once

#include "KPhysics/KPhysics.hpp"
#include "Vec3.hpp"

#include <glm/glm.hpp>
#include <glm/mat4x4.hpp>

#include <glm/ext/matrix_transform.hpp>

BEGIN_NAMESPACE_KAYOU

using Mat4 = glm::mat4;

inline Mat4 Mat4Identity()
{
    return glm::identity<Mat4>();
}

inline Mat4 Transpose(const Mat4& matrix)
{
    return glm::transpose(matrix);
}

inline Mat4 Inverse(const Mat4& matrix)
{
    return glm::inverse(matrix);
}

inline Mat4 Translate(const Mat4& matrix, const Vec3& translation)
{
    return glm::translate(matrix, translation);
}

inline Mat4 Rotate(const Mat4& matrix, const float angle, const Vec3& axis)
{
    return glm::rotate(matrix, angle, axis);
}

inline Mat4 Scale(const Mat4& matrix, const Vec3& scale)
{
    return glm::scale(matrix, scale);
}

END_NAMESPACE_KAYOU