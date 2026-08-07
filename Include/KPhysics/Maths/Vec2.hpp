#pragma once

#include "KPhysics/Utils.hpp"

#include <glm/glm.hpp>
#include <glm/vec2.hpp>
#include <glm/gtx/norm.hpp>

BEGIN_NAMESPACE_KAYOU

using Vec2 = glm::vec2;

KINLINE float Dot(const Vec2& a, const Vec2& b)
{
    return glm::dot(a, b);
}

KINLINE float Length(const Vec2& v)
{
    return glm::length(v);
}

KINLINE float LengthSquared(const Vec2& v)
{
    return glm::length2(v);
}

KINLINE Vec2 Normalize(const Vec2& v)
{
    return glm::normalize(v);
}

KINLINE float Distance(const Vec2& a, const Vec2& b)
{
    return glm::distance(a, b);
}

KINLINE float DistanceSquared(const Vec2& a, const Vec2& b)
{
    return glm::distance2(a, b);
}

KINLINE Vec2 Min(const Vec2& a, const Vec2& b)
{
    return glm::min(a, b);
}

KINLINE Vec2 Max(const Vec2& a, const Vec2& b)
{
    return glm::max(a, b);
}

END_NAMESPACE_KAYOU