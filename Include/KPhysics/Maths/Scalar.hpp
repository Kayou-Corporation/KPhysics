#pragma once

#include "KPhysics/Utils.hpp"

#include <glm/glm.hpp>

BEGIN_NAMESPACE_KAYOU

constexpr float Epsilon = 1e-6f;

KINLINE float Radians(const float degrees)
{
    return glm::radians(degrees);
}

KINLINE float Degrees(const float radians)
{
    return glm::degrees(radians);
}

KINLINE float Clamp(const float value, const float min, const float max)
{
    return glm::clamp(value, min, max);
}

KINLINE float Min(const float a, const float b)
{
    return glm::min(a, b);
}

KINLINE float Max(const float a, const float b)
{
    return glm::max(a, b);
}

KINLINE float Lerp(const float a, const float b, const float t)
{
    return glm::mix(a, b, t);
}

KINLINE float Abs(const float value)
{
    return glm::abs(value);
}

KINLINE float Sqrt(const float value)
{
    return glm::sqrt(value);
}

KINLINE float Sin(const float angle)
{
    return glm::sin(angle);
}

KINLINE float Cos(const float angle)
{
    return glm::cos(angle);
}

KINLINE float Tan(const float angle)
{
    return glm::tan(angle);
}

KINLINE float Sign(const float value)
{
    return glm::sign(value);
}

KINLINE bool IsNearlyEqual(const float a, const float b, const float epsilon = Epsilon)
{
    return glm::abs(a - b) <= epsilon;
}

KINLINE bool IsNearlyZero(const float value, const float epsilon = Epsilon)
{
    return glm::abs(value) <= epsilon;
}

END_NAMESPACE_KAYOU