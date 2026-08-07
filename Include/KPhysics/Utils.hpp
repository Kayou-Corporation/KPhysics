#pragma once

#define KAYOU_GLOBAL Kayou

#define USING_KAYOU using namespace KAYOU_GLOBAL;

#define BEGIN_NAMESPACE_KAYOU namespace KAYOU_GLOBAL {
#define END_NAMESPACE_KAYOU } /* namespace KAYOU_GLOBAL */
#include <vector>
#include <cstdint>

#include "Preprocessor.hpp"

template<typename T>
using Array = std::vector<T>;

using ColliderID = uint32_t;
using ShapeID = uint32_t;
