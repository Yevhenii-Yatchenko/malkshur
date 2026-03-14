cmake_minimum_required(VERSION 3.10)
project(dd_shahed CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Detect platform
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|ARM64")
    set(PLATFORM_ARM64 ON)
    message(STATUS "Building for ARM64 (Jetson Nano)")
else()
    set(PLATFORM_X86_64 ON)
    message(STATUS "Building for x86_64")
endif()

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Source files
set(SOURCES
    src/main.cpp
    src/detection.cpp
    src/direction_vector.cpp
    src/socket_client.cpp
    src/trt_engine.cpp
    src/tcp_frame_client.cpp
    src/utils.cpp
)

# Find OpenCV
find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
    message(STATUS "OpenCV version: ${OpenCV_VERSION}")
    message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
    message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")
else()
    message(FATAL_ERROR "OpenCV not found")
endif()

# Find CUDA
find_package(CUDA REQUIRED)
if(CUDA_FOUND)
    message(STATUS "CUDA version: ${CUDA_VERSION}")
    message(STATUS "CUDA include dirs: ${CUDA_INCLUDE_DIRS}")
else()
    message(FATAL_ERROR "CUDA not found")
endif()

# TensorRT paths - try common locations
set(TENSORRT_PATHS
    /usr/src/tensorrt
    /usr/local/tensorrt
    /opt/tensorrt
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/tensorrt
)

find_path(TENSORRT_INCLUDE_DIR
    NvInfer.h
    PATHS ${TENSORRT_PATHS}
    PATH_SUFFIXES include
    DOC "TensorRT include directory"
)

if(NOT TENSORRT_INCLUDE_DIR)
    message(FATAL_ERROR "TensorRT include directory not found. Please set TENSORRT_ROOT or install TensorRT.")
endif()

message(STATUS "TensorRT include dir: ${TENSORRT_INCLUDE_DIR}")

# Find TensorRT libraries
if(PLATFORM_ARM64)
    set(TENSORRT_LIB_PATHS
        /usr/lib/aarch64-linux-gnu
        /usr/local/lib
    )
    set(OPENCV_LIB_PATHS
        /usr/lib/aarch64-linux-gnu
    )
    set(CUDA_LIB_PATHS
        /usr/local/cuda/lib64
        /usr/lib/aarch64-linux-gnu
    )
else()
    set(TENSORRT_LIB_PATHS
        /usr/lib/x86_64-linux-gnu
        /usr/local/lib
    )
    set(OPENCV_LIB_PATHS
        /usr/lib/x86_64-linux-gnu
    )
    set(CUDA_LIB_PATHS
        /usr/local/cuda/lib64
        /usr/lib/x86_64-linux-gnu
    )
endif()

find_library(TENSORRT_LIBRARY
    NAMES nvinfer
    PATHS ${TENSORRT_LIB_PATHS}
    DOC "TensorRT library"
)

if(PLATFORM_X86_64)
    find_library(TENSORRT_ONNX_LIBRARY
        NAMES nvonnxparser
        PATHS ${TENSORRT_LIB_PATHS}
        DOC "TensorRT ONNX parser library"
    )
endif()

if(NOT TENSORRT_LIBRARY)
    message(FATAL_ERROR "TensorRT library (nvinfer) not found")
endif()

message(STATUS "TensorRT library: ${TENSORRT_LIBRARY}")

# Include directories
include_directories(
    ${TENSORRT_INCLUDE_DIR}
    ${CUDA_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# Link directories
if(PLATFORM_ARM64)
    link_directories(
        /usr/lib/aarch64-linux-gnu
        /usr/local/cuda/lib64
    )
else()
    link_directories(
        /usr/lib/x86_64-linux-gnu
        /usr/local/cuda/lib64
    )
endif()

# Create executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Link libraries
target_link_libraries(${PROJECT_NAME}
    ${TENSORRT_LIBRARY}
    ${OpenCV_LIBS}
    ${CUDA_LIBRARIES}
    cudart
)

if(PLATFORM_X86_64 AND TENSORRT_ONNX_LIBRARY)
    target_link_libraries(${PROJECT_NAME} ${TENSORRT_ONNX_LIBRARY})
endif()

# Set default build type to Release if not specified
# (Only for single-configuration generators like Unix Makefiles)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
    message(STATUS "CMAKE_BUILD_TYPE not set, defaulting to Release")
endif()

# Set output name with platform suffix
if(PLATFORM_ARM64)
    set(PLATFORM_SUFFIX "arm64")
else()
    set(PLATFORM_SUFFIX "x86_64")
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    OUTPUT_NAME "${PROJECT_NAME}_${PLATFORM_SUFFIX}"
)

# Option to copy binary to source bin/ directory (for version control)
option(COPY_BINARY_TO_BIN "Copy built binary to bin/ directory after build" OFF)

if(COPY_BINARY_TO_BIN)
    # Create bin directory in source tree
    file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

    # Copy binary to source bin/ directory after build
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
        $<TARGET_FILE:${PROJECT_NAME}>
        ${CMAKE_SOURCE_DIR}/bin/${PROJECT_NAME}_${PLATFORM_SUFFIX}
        COMMENT "Copying binary to ${CMAKE_SOURCE_DIR}/bin/${PROJECT_NAME}_${PLATFORM_SUFFIX}"
    )
    message(STATUS "Binary will be copied to bin/ after build")
else()
    message(STATUS "Binary will NOT be copied to bin/ (use -DCOPY_BINARY_TO_BIN=ON to enable)")
endif()

# Print build info
message(STATUS "")
message(STATUS "=== Build Configuration ===")
message(STATUS "Platform: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "Platform suffix: ${PLATFORM_SUFFIX}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Build output: ${CMAKE_BINARY_DIR}/bin/${PROJECT_NAME}_${PLATFORM_SUFFIX}")
if(COPY_BINARY_TO_BIN)
    message(STATUS "Binary copy: ${CMAKE_SOURCE_DIR}/bin/${PROJECT_NAME}_${PLATFORM_SUFFIX}")
endif()
message(STATUS "==========================")
message(STATUS "")

