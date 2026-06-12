/**
 * @file lite3_highlevel.hpp
 * @brief Aggregate header — include this for the full C++ API
 */
#pragma once

#include "types.hpp"
#include "bridge.hpp"
#include "observation.hpp"
#ifdef HAS_ONNX
#include "onnx_runner.hpp"
#endif
