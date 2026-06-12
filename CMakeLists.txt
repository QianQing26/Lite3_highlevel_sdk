cmake_minimum_required(VERSION 3.14)
project(Lite3Highlevel VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ========== ONNX Runtime ==========
# Auto-detect in third_party/onnxruntime, or set manually:
#   cmake .. -DONNXRUNTIME_ROOT=/path/to/onnxruntime
set(ONNXRUNTIME_ROOT "" CACHE PATH "Path to ONNX Runtime installation")

if(NOT ONNXRUNTIME_ROOT)
    set(_BUNDLED "${CMAKE_CURRENT_SOURCE_DIR}/third_party/onnxruntime")
    if(EXISTS "${_BUNDLED}/include/onnxruntime_cxx_api.h")
        set(ONNXRUNTIME_ROOT "${_BUNDLED}")
    endif()
endif()

set(HAS_ONNX OFF)
if(ONNXRUNTIME_ROOT)
    include_directories(${ONNXRUNTIME_ROOT}/include)
    link_directories(${ONNXRUNTIME_ROOT}/lib)
    add_definitions(-DHAS_ONNX)
    set(HAS_ONNX ON)
    message(STATUS "ONNX Runtime: ${ONNXRUNTIME_ROOT}")
else()
    message(STATUS "ONNX Runtime: not found — Bridge + ObsBuilder + CmdProcessor only")
endif()

# ========== Options ==========
option(BUILD_EXAMPLES "Build example programs" ON)

# ========== Include paths ==========
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp/include
    ${CMAKE_CURRENT_SOURCE_DIR}/bridge
)

# ========== Library ==========
if(HAS_ONNX)
    add_library(lite3_highlevel STATIC cpp/src/onnx_runner.cpp)
    target_include_directories(lite3_highlevel PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/cpp/include
        ${CMAKE_CURRENT_SOURCE_DIR}/bridge
    )
    target_link_libraries(lite3_highlevel PUBLIC onnxruntime pthread)
else()
    add_library(lite3_highlevel INTERFACE)
    target_include_directories(lite3_highlevel INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/cpp/include
        ${CMAKE_CURRENT_SOURCE_DIR}/bridge
    )
endif()

# ========== Examples ==========
if(BUILD_EXAMPLES AND HAS_ONNX)
    add_executable(simple_controller examples/simple_controller.cpp)
    target_link_libraries(simple_controller lite3_highlevel)
endif()
