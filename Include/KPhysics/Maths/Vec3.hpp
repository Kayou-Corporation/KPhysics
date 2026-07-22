#pragma once

#include "KPhysics/KPhysics.hpp"

#include <glm/glm.hpp>
#include <glm/vec3.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>

BEGIN_NAMESPACE_KAYOU

using Vec3 = glm::vec3;

inline float Dot(const Vec3& a, const Vec3& b)
{
    return glm::dot(a, b);
}

inline Vec3 Cross(const Vec3& a, const Vec3& b)
{
    return glm::cross(a, b);
}

inline float Length(const Vec3& v)
{
    return glm::length(v);
}

inline float LengthSquared(const Vec3& v)
{
    return glm::length2(v);
}

inline Vec3 Normalize(const Vec3& v)
{
    return glm::normalize(v);
}

inline float Distance(const Vec3& a, const Vec3& b)
{
    return glm::distance(a, b);
}

inline float DistanceSquared(const Vec3& a, const Vec3& b)
{
    return glm::distance2(a, b);
}

inline Vec3 Min(const Vec3& a, const Vec3& b)
{
    return glm::min(a, b);
}

inline Vec3 Max(const Vec3& a, const Vec3& b)
{
    return glm::max(a, b);
}

END_NAMESPACE_KAYOU