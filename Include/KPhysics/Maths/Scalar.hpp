#pragma once

#include "KPhysics/KPhysics.hpp"

#include <glm/glm.hpp>

BEGIN_NAMESPACE_KAYOU

constexpr float Epsilon = 1e-6f;

inline float Radians(const float degrees)
{
    return glm::radians(degrees);
}

inline float Degrees(const float radians)
{
    return glm::degrees(radians);
}

inline float Clamp(const float value, const float min, const float max)
{
    return glm::clamp(value, min, max);
}

inline float Min(const float a, const float b)
{
    return glm::min(a, b);
}

inline float Max(const float a, const float b)
{
    return glm::max(a, b);
}

inline float Lerp(const float a, const float b, const float t)
{
    return glm::mix(a, b, t);
}

inline float Abs(const float value)
{
    return glm::abs(value);
}

inline float Sqrt(const float value)
{
    return glm::sqrt(value);
}

inline float Sin(const float angle)
{
    return glm::sin(angle);
}

inline float Cos(const float angle)
{
    return glm::cos(angle);
}

inline float Tan(const float angle)
{
    return glm::tan(angle);
}

inline float Sign(const float value)
{
    return glm::sign(value);
}

inline bool IsNearlyEqual(const float a, const float b, const float epsilon = Epsilon)
{
    return glm::abs(a - b) <= epsilon;
}

inline bool IsNearlyZero(const float value, const float epsilon = Epsilon)
{
    return glm::abs(value) <= epsilon;
}

END_NAMESPACE_KAYOU