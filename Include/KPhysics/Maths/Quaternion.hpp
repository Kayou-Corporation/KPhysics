#pragma once

#include "KPhysics/Utils.hpp"
#include "Vec3.hpp"
#include "Mat3.hpp"
#include "Mat4.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>

BEGIN_NAMESPACE_KAYOU

using Quat = glm::quat;

KINLINE Quat QuatIdentity()
{
    return glm::identity<Quat>();
}

KINLINE Quat AngleAxis(const float angle, const Vec3& axis)
{
    return glm::angleAxis(angle, axis);
}

KINLINE Quat FromEuler(const Vec3& euler)
{
    return { euler };
}

KINLINE float Length(const Quat& q)
{
    return glm::length(q);
}

KINLINE float LengthSquared(const Quat& q)
{
    return glm::length2(q);
}

KINLINE float Dot(const Quat& a, const Quat& b)
{
    return glm::dot(a, b);
}

KINLINE Quat Normalize(const Quat& q)
{
    return glm::normalize(q);
}

KINLINE Quat Conjugate(const Quat& q)
{
    return glm::conjugate(q);
}

KINLINE Quat Inverse(const Quat& q)
{
    return glm::inverse(q);
}

KINLINE Quat Lerp(const Quat& a, const Quat& b, const float t)
{
    return glm::lerp(a, b, t);
}

KINLINE Quat Slerp(const Quat& a, const Quat& b, const float t)
{
    return glm::slerp(a, b, t);
}

KINLINE Vec3 ToEuler(const Quat& q)
{
    return glm::eulerAngles(q);
}

KINLINE Mat3 ToMat3(const Quat& q)
{
    return glm::mat3_cast(q);
}

KINLINE Mat4 ToMat4(const Quat& q)
{
    return glm::mat4_cast(q);
}

KINLINE Vec3 Rotate(const Quat& q, const Vec3& v)
{
    return q * v;
}

END_NAMESPACE_KAYOU