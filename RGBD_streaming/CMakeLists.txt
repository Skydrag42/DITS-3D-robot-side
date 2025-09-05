cmake_minimum_required(VERSION 3.12)
project(gst_app_tutorial)

# Set C++20 standard globally
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Platform-specific compiler flags
if(WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /O2 /W3")
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wall")
endif()

# Find required packages
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)

# GStreamer configuration
pkg_check_modules(GST REQUIRED 
    gstreamer-1.0 
    gstreamer-audio-1.0 
    gstreamer-video-1.0  
    gstreamer-app-1.0
)

message(STATUS "GST_INCLUDE_DIRS = ${GST_INCLUDE_DIRS}")
message(STATUS "GST_LIBRARY_DIRS = ${GST_LIBRARY_DIRS}")
message(STATUS "GST_LIBRARIES = ${GST_LIBRARIES}")

# Intel RealSense SDK configuration
set(REALSENSE_ROOT "C:/Program Files (x86)/Intel RealSense SDK 2.0")
set(REALSENSE_INCLUDE_DIR "${REALSENSE_ROOT}/include")
set(REALSENSE_LIB_DIR "${REALSENSE_ROOT}/lib/x64")
set(REALSENSE_BIN_DIR "${REALSENSE_ROOT}/bin/x64")

# Check if RealSense is available
if(EXISTS "${REALSENSE_INCLUDE_DIR}" AND EXISTS "${REALSENSE_LIB_DIR}/realsense2.lib")
    set(REALSENSE_FOUND TRUE)
    message(STATUS "Found Intel RealSense SDK")
else()
    set(REALSENSE_FOUND FALSE)
    message(WARNING "Intel RealSense SDK not found at ${REALSENSE_ROOT}")
endif()

# Search for libyuv headers and library in local paths
find_path(LIBYUV_INCLUDE_DIR 
    NAMES libyuv.h 
    PATHS 
        ${CMAKE_CURRENT_SOURCE_DIR}/include 
        ${CMAKE_CURRENT_SOURCE_DIR}
    PATH_SUFFIXES libyuv
)
find_library(LIBYUV_LIBRARY 
    NAMES yuv libyuv 
    PATHS 
        ${CMAKE_CURRENT_SOURCE_DIR}/lib 
        ${CMAKE_CURRENT_SOURCE_DIR}
)

# If libyuv is not found locally, download and build from source
if(NOT LIBYUV_INCLUDE_DIR OR NOT LIBYUV_LIBRARY)
    include(FetchContent)
    FetchContent_Declare(
        libyuv 
        GIT_REPOSITORY https://chromium.googlesource.com/libyuv/libyuv 
        GIT_TAG main 
        GIT_SHALLOW TRUE
    )
    set(LIBYUV_UNIT_TEST OFF CACHE BOOL "Disable libyuv unit tests")
    FetchContent_MakeAvailable(libyuv)
    set(LIBYUV_INCLUDE_DIR ${libyuv_SOURCE_DIR}/include)
    set(LIBYUV_LIBRARY yuv)
endif()

# Create executable targets
add_executable(send 
    send.cpp 
    "encode_z16.cpp" 
    "encode_z16.h"
    "encodeRGBtoI420.h" 
    "encodeRGBtoI420.cpp"
)

add_executable(Tests_z16_yuv 
    "Tests_z16_yuv.cpp" 
    "encode_z16.cpp" 
    "encode_z16.h"
)

add_executable(receive 
    receive.cpp 
    "encode_z16.cpp" 
    "encode_z16.h" 
    "encodeRGBtoI420.cpp" 
    "encodeRGBtoI420.h"
)

add_executable(rs_tests 
    rs_tests.cpp 
    "rs_tests.h" 
    "encode_z16.cpp" 
    "encode_z16.h" 
    "encodeRGBtoI420.cpp" 
    "encodeRGBtoI420.h"
)

# Configure all targets
set(ALL_TARGETS send Tests_z16_yuv receive rs_tests)

foreach(target ${ALL_TARGETS})
    # Include directories
    target_include_directories(${target} PRIVATE 
        ${GST_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${LIBYUV_INCLUDE_DIR}
    )
    
    # Link directories
    target_link_directories(${target} PRIVATE ${GST_LIBRARY_DIRS})
    
    # Basic libraries
    target_link_libraries(${target} 
        ${GST_LIBRARIES} 
        ${OpenCV_LIBS}
    )
    
    # Add RealSense for specific targets
    if(REALSENSE_FOUND AND (${target} STREQUAL "send" OR ${target} STREQUAL "rs_tests"))
        target_include_directories(${target} PRIVATE ${REALSENSE_INCLUDE_DIR})
        target_link_libraries(${target} "${REALSENSE_LIB_DIR}/realsense2.lib")
    endif()
    
    # Add libyuv for targets that use encodeRGBtoI420.cpp
    if(${target} STREQUAL "send" OR ${target} STREQUAL "receive" OR ${target} STREQUAL "rs_tests")
        target_link_libraries(${target} ${LIBYUV_LIBRARY})
    endif()
endforeach()

# Copy RealSense DLL for rs_tests executable
if(WIN32 AND REALSENSE_FOUND)
    message(STATUS "Adding post-build script to copy realsense2.dll")
    add_custom_command(TARGET rs_tests POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${REALSENSE_BIN_DIR}/realsense2.dll"
            $<TARGET_FILE_DIR:rs_tests>
        COMMENT "Copying realsense2.dll to output directory"
    )
endif()