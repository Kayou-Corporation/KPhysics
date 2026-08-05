#pragma once

#include "KPhysics/Utils.hpp"

#include <glm/glm.hpp>
#include <glm/mat3x3.hpp>

#include <glm/ext/matrix_transform.hpp>

#include "Vec3.hpp"

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

inline Mat3 ComputeSphereInverseInertia(const float invMass, const float radius)
{
    const float invI = (5.f * invMass) / (2.f * radius * radius);
    return {invI};
}

inline Mat3 ComputeBoxInverseInertia(const float invMass, const Vec3& halfExtents)
{
    const float width = halfExtents.x * 2.0f;
    const float height = halfExtents.y * 2.0f;
    const float depth = halfExtents.z * 2.0f;

    const float invIx = 12.0f * invMass / (height * height + depth * depth);
    const float invIy = 12.0f * invMass / (width * width + depth * depth);
    const float invIz = 12.0f * invMass / (width * width + height * height);

    Mat3 invInertia = Mat3Identity();
    invInertia[0][0] = invIx;
    invInertia[1][1] = invIy;
    invInertia[2][2] = invIz;
    return invInertia;
}

inline Mat3 ComputeCapsuleInverseInertia(const float invMass, const float radius, const float halfHeight)
{
    const float height = halfHeight * 2.0f;

    const float invIxz = 12.0f * invMass / (3.0f * radius * radius + height * height);
    const float invIy = 2.0f * invMass / (radius * radius);

    Mat3 invInertia = Mat3Identity();
    invInertia[0][0] = invIxz;
    invInertia[1][1] = invIy;
    invInertia[2][2] = invIxz;
    return invInertia;
}

END_NAMESPACE_KAYOU