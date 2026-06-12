cmake_minimum_required(VERSION 3.14)
project(Lite3Highlevel VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ========== ONNX Runtime ==========
# Set this to your ONNX Runtime installation path, e.g.:
#   cmake .. -DONNXRUNTIME_ROOT=/usr
#   cmake .. -DONNXRUNTIME_ROOT=~/onnxruntime-linux-aarch64-1.22.0
set(ONNXRUNTIME_ROOT "" CACHE PATH "Path to ONNX Runtime installation")
set(HAS_ONNX OFF)

# ========== Options ==========
option(BUILD_EXAMPLES "Build example programs" ON)

# ========== Include paths ==========
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp/include
    ${CMAKE_CURRENT_SOURCE_DIR}/bridge
)

# ========== Library sources ==========

# ONNX runner — only compiled when ONNXRUNTIME_ROOT is set
if(ONNXRUNTIME_ROOT)
    include_directories(${ONNXRUNTIME_ROOT}/include)
    link_directories(${ONNXRUNTIME_ROOT}/lib)
    add_definitions(-DHAS_ONNX)
    set(HAS_ONNX ON)
    message(STATUS "ONNX Runtime: ${ONNXRUNTIME_ROOT}")
else()
    message(STATUS "ONNX Runtime: not configured (Bridge + ObsBuilder + CmdProcessor only)")
endif()

# ========== Library ==========
if(HAS_ONNX)
    add_library(lite3_highlevel STATIC cpp/src/onnx_runner.cpp)
else()
    # Header-only library when ONNX is not configured
    add_library(lite3_highlevel INTERFACE)
endif()

if(HAS_ONNX)
    target_include_directories(lite3_highlevel PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/cpp/include
        ${CMAKE_CURRENT_SOURCE_DIR}/bridge
    )
else()
    target_include_directories(lite3_highlevel INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/cpp/include
        ${CMAKE_CURRENT_SOURCE_DIR}/bridge
    )
endif()

if(HAS_ONNX)
    target_link_libraries(lite3_highlevel PUBLIC onnxruntime pthread)
endif()

# ========== Examples ==========
if(BUILD_EXAMPLES AND HAS_ONNX)
    add_executable(simple_controller examples/simple_controller.cpp)
    target_link_libraries(simple_controller lite3_highlevel)
endif()
