#pragma once

#include "Preprocessor.hpp"
#include "Utils.hpp"

BEGIN_NAMESPACE_KAYOU

template <typename Tag>
class ID
{
public:
    static constexpr uint32_t GENERATION_NUMBER_SHIFT = 24;
    static constexpr uint32_t INVALID = 0xFFFFFFFF;
    static constexpr uint32_t MAX_BODY_ID = (1u << GENERATION_NUMBER_SHIFT) - 2;
    static constexpr uint8_t MAX_GENERATION = 0xFF;

    ID() = default;

    explicit ID(const uint32_t value) : m_value(value)
    {
    }

    explicit ID(const uint32_t index, const uint8_t generation)
    {
        m_value = (generation << GENERATION_NUMBER_SHIFT) | index;
    }

    KNO_DISCARD uint32_t Index() const
    {
        return m_value & MAX_BODY_ID;
    }

    KNO_DISCARD uint8_t Generation() const
    {
        return static_cast<uint8_t>(m_value >> GENERATION_NUMBER_SHIFT);
    }

    KNO_DISCARD bool IsValid() const
    {
        return m_value != INVALID;
    }

    explicit operator bool() const
    {
        return IsValid();
    }

    bool operator==(const ID& other) const
    {
        return m_value == other.m_value;
    }

    bool operator!=(const ID& other) const
    {
        return m_value != other.m_value;
    }

private:
    uint32_t m_value = INVALID;
};

// Tag dispatching

struct BodyIDTag {};
struct ColliderIDTag {};
struct ShapeIDTag {};

using BodyID = ID<BodyIDTag>;
using ColliderID = ID<ColliderIDTag>;
using ShapeID = ID<ShapeIDTag>;

END_NAMESPACE_KAYOU