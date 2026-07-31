#pragma once

#include "KPhysics/Utils.hpp"

#include <glm/glm.hpp>
#include <glm/vec4.hpp>
#include <glm/gtx/norm.hpp>

BEGIN_NAMESPACE_KAYOU

using Vec4 = glm::vec4;

inline float Dot(const Vec4& a, const Vec4& b)
{
    return glm::dot(a, b);
}

inline float Length(const Vec4& v)
{
    return glm::length(v);
}

inline float LengthSquared(const Vec4& v)
{
    return glm::length2(v);
}

inline Vec4 Normalize(const Vec4& v)
{
    return glm::normalize(v);
}

inline Vec4 Min(const Vec4& a, const Vec4& b)
{
    return glm::min(a, b);
}

inline Vec4 Max(const Vec4& a, const Vec4& b)
{
    return glm::max(a, b);
}

inline Vec4 Clamp(const Vec4& v, const Vec4& min, const Vec4& max)
{
    return glm::clamp(v, min, max);
}

inline Vec4 Lerp(const Vec4& a, const Vec4& b, const float t)
{
    return glm::mix(a, b, t);
}

END_NAMESPACE_KAYOU