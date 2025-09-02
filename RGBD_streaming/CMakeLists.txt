cmake_minimum_required(VERSION 3.1)
project(gst_app_tutorial)
set(CMAKE_CXX_STANDARD 20)

find_package(PkgConfig REQUIRED)

# GStreamer
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-audio-1.0 gstreamer-video-1.0  gstreamer-app-1.0)
message("GST_INCLUDE_DIRS = ${GST_INCLUDE_DIRS}")
message("GST_LIBRARY_DIRS = ${GST_LIBRARY_DIRS}")
message("GST_LIBRARIES = ${GST_LIBRARIES}")

link_directories( ${GST_LIBRARY_DIRS}  )
include_directories( ${GST_INCLUDE_DIRS}  )


# OpenCV ${OpenCV_LIBS}, used for some examples
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

link_libraries("C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\lib\\x64\\realsense2.lib")
include_directories("C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\include")

add_executable(send send.cpp "encode_z16.cpp" "encode_z16.h"  "encodeRGBtoI420.h" "encodeRGBtoI420.cpp")
target_link_libraries(send ${GST_LIBRARIES} ${OpenCV_LIBS})

add_executable(Testage "Tests_z16_yuv.cpp" "encode_z16.cpp" "encode_z16.h"  )
target_link_libraries(Testage ${GST_LIBRARIES} ${OpenCV_LIBS})

add_executable(receive receive.cpp "encode_z16.cpp" "encode_z16.h" "encodeRGBtoI420.cpp" "encodeRGBtoI420.h")
target_link_libraries(receive ${GST_LIBRARIES} ${OpenCV_LIBS})

add_executable (rs_tests rs_tests.cpp "rs_tests.h" "encode_z16.cpp" "encode_z16.h" "encodeRGBtoI420.cpp" "encodeRGBtoI420.h" )
target_link_libraries(rs_tests ${GST_LIBRARIES} ${OpenCV_LIBS})

set(PROJECT_BINARY_OUTPUT_PATH "realsense2.dll") 
set(DLL_DIR "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\bin\\x64") 
if(WIN32)
    message(STATUS "Adding Post build script to copy realsense2.dll to project's binary folder")
    message(STATUS "Will try to copy from ${DLL_DIR} to ${PROJECT_BINARY_OUTPUT_PATH}")
    add_custom_command(TARGET rs_tests POST_BUILD        # Adds a post-build event to rs-getting-started
    COMMAND ${CMAKE_COMMAND} -E copy_if_different              # which executes "cmake - E copy_if_different..."
        "${DLL_DIR}\\realsense2.dll"                            # <--this is in-file
        ${PROJECT_BINARY_OUTPUT_PATH})                 # <--this is out-file path
endif()

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET rs_tests PROPERTY CXX_STANDARD 20)
endif()
