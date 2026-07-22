#pragma once

#include "KPhysics/KPhysics.hpp"
#include "Vec3.hpp"
#include "Mat3.hpp"
#include "Mat4.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>

BEGIN_NAMESPACE_KAYOU

using Quat = glm::quat;

inline Quat QuatIdentity()
{
    return glm::identity<Quat>();
}

inline Quat AngleAxis(const float angle, const Vec3& axis)
{
    return glm::angleAxis(angle, axis);
}

inline Quat FromEuler(const Vec3& euler)
{
    return { euler };
}

inline float Length(const Quat& q)
{
    return glm::length(q);
}

inline float LengthSquared(const Quat& q)
{
    return glm::length2(q);
}

inline float Dot(const Quat& a, const Quat& b)
{
    return glm::dot(a, b);
}

inline Quat Normalize(const Quat& q)
{
    return glm::normalize(q);
}

inline Quat Conjugate(const Quat& q)
{
    return glm::conjugate(q);
}

inline Quat Inverse(const Quat& q)
{
    return glm::inverse(q);
}

inline Quat Lerp(const Quat& a, const Quat& b, const float t)
{
    return glm::lerp(a, b, t);
}

inline Quat Slerp(const Quat& a, const Quat& b, const float t)
{
    return glm::slerp(a, b, t);
}

inline Vec3 ToEuler(const Quat& q)
{
    return glm::eulerAngles(q);
}

inline Mat3 ToMat3(const Quat& q)
{
    return glm::mat3_cast(q);
}

inline Mat4 ToMat4(const Quat& q)
{
    return glm::mat4_cast(q);
}

inline Vec3 Rotate(const Quat& q, const Vec3& v)
{
    return q * v;
}

END_NAMESPACE_KAYOU