#pragma once

#include "Preprocessor.hpp"
#include "Utils.hpp"

BEGIN_NAMESPACE_KAYOU

class BodyID
{
public:
    static constexpr uint32_t INVALID = 0xFFFFFFFF;
    static constexpr uint32_t MAX_BODY_ID = 0x00FFFFFF;
    static constexpr uint8_t MAX_GENERATION = 0xFF;
    static constexpr uint32_t GENERATION_NUMBER_SHIFT = 24;

    BodyID() = default;

    explicit BodyID(const uint32_t value) : m_value(value)
    {
    }

    explicit BodyID(const uint32_t index, const uint8_t generation)
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

private:
    uint32_t m_value = INVALID;
};

END_NAMESPACE_KAYOU