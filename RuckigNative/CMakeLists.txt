cmake_minimum_required(VERSION 3.22)
project(ruckig_jni)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Path to the ruckig submodule.  Can be overridden at configure time:
#   cmake -DRUCKIG_ROOT=/path/to/ruckig ...
if(NOT DEFINED RUCKIG_ROOT)
    set(RUCKIG_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../ruckig")
endif()

add_library(ruckig_jni SHARED
    src/main/cpp/ruckig_jni.cpp
    ${RUCKIG_ROOT}/src/ruckig/brake.cpp
    ${RUCKIG_ROOT}/src/ruckig/position_first_step1.cpp
    ${RUCKIG_ROOT}/src/ruckig/position_first_step2.cpp
    ${RUCKIG_ROOT}/src/ruckig/position_second_step1.cpp
    ${RUCKIG_ROOT}/src/ruckig/position_second_step2.cpp
    ${RUCKIG_ROOT}/src/ruckig/position_third_step1.cpp
    ${RUCKIG_ROOT}/src/ruckig/position_third_step2.cpp
    ${RUCKIG_ROOT}/src/ruckig/velocity_second_step1.cpp
    ${RUCKIG_ROOT}/src/ruckig/velocity_second_step2.cpp
    ${RUCKIG_ROOT}/src/ruckig/velocity_third_step1.cpp
    ${RUCKIG_ROOT}/src/ruckig/velocity_third_step2.cpp
)

target_include_directories(ruckig_jni PRIVATE
    ${RUCKIG_ROOT}/include
)

if(ANDROID)
    target_link_libraries(ruckig_jni android log)
else()
    find_package(JNI REQUIRED)
    target_include_directories(ruckig_jni PRIVATE ${JNI_INCLUDE_DIRS})
endif()

# MSVC does not define M_PI unless _USE_MATH_DEFINES is set
target_compile_definitions(ruckig_jni PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:_USE_MATH_DEFINES>
)
