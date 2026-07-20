#pragma once

#include "KPhysics/KPhysics.hpp"

#include <glm/glm.hpp>
#include <glm/vec2.hpp>
#include <glm/gtx/norm.hpp>

BEGIN_NAMESPACE_KAYOU

using Vec2 = glm::vec2;

inline float Dot(const Vec2& a, const Vec2& b)
{
    return glm::dot(a, b);
}

inline float Length(const Vec2& v)
{
    return glm::length(v);
}

inline float LengthSquared(const Vec2& v)
{
    return glm::length2(v);
}

inline Vec2 Normalize(const Vec2& v)
{
    return glm::normalize(v);
}

inline float Distance(const Vec2& a, const Vec2& b)
{
    return glm::distance(a, b);
}

inline float DistanceSquared(const Vec2& a, const Vec2& b)
{
    return glm::distance2(a, b);
}

inline Vec2 Min(const Vec2& a, const Vec2& b)
{
    return glm::min(a, b);
}

inline Vec2 Max(const Vec2& a, const Vec2& b)
{
    return glm::max(a, b);
}

END_NAMESPACE_KAYOU