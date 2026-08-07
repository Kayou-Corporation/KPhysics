#pragma once

#include "KPhysics/Utils.hpp"

#include <glm/glm.hpp>
#include <glm/vec3.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>

BEGIN_NAMESPACE_KAYOU

using Vec3 = glm::vec3;

KINLINE float Dot(const Vec3& a, const Vec3& b)
{
    return glm::dot(a, b);
}

KINLINE Vec3 Cross(const Vec3& a, const Vec3& b)
{
    return glm::cross(a, b);
}

KINLINE float Length(const Vec3& v)
{
    return glm::length(v);
}

KINLINE float LengthSquared(const Vec3& v)
{
    return glm::length2(v);
}

KINLINE Vec3 Normalize(const Vec3& v)
{
    return glm::normalize(v);
}

KINLINE float Distance(const Vec3& a, const Vec3& b)
{
    return glm::distance(a, b);
}

KINLINE float DistanceSquared(const Vec3& a, const Vec3& b)
{
    return glm::distance2(a, b);
}

KINLINE Vec3 Min(const Vec3& a, const Vec3& b)
{
    return glm::min(a, b);
}

KINLINE Vec3 Max(const Vec3& a, const Vec3& b)
{
    return glm::max(a, b);
}

END_NAMESPACE_KAYOU