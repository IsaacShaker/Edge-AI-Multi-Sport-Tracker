cmake_minimum_required(VERSION 3.16)
project(EdgeAISportTracker VERSION 1.0.0 LANGUAGES CXX)

# C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Options
option(BUILD_TESTS "Build unit tests" ON)
option(BUILD_EXAMPLES "Build example applications" ON)
option(WITH_HAILO "Enable Hailo NPU support (requires HailoRT SDK)" OFF)

# Find packages
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)

# ── ONNX Runtime (required for YOLODetector) ─────────────────────────────────
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime/onnxruntime_cxx_api.h
    PATHS /usr/include /usr/local/include)
find_library(ONNXRUNTIME_LIB onnxruntime
    PATHS /usr/lib/aarch64-linux-gnu /usr/local/lib /usr/lib)

if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIB)
    message(FATAL_ERROR "ONNX Runtime not found. On the Pi: sudo apt install libonnxruntime-dev")
endif()
message(STATUS "ONNX Runtime: ${ONNXRUNTIME_LIB}")

# ── Optional: Hailo NPU (HailoRT) ─────────────────────────────────────────────
if(WITH_HAILO)
    find_package(HailoRT QUIET)
    if(HailoRT_FOUND)
        message(STATUS "HailoRT found: ${HailoRT_VERSION}")
        add_compile_definitions(WITH_HAILO)
        # HailoRT cmake config exposes an imported target; derive include/lib from it.
        get_target_property(HailoRT_INCLUDE_DIRS HailoRT::libhailort INTERFACE_INCLUDE_DIRECTORIES)
        set(HailoRT_LIBRARIES HailoRT::libhailort)
    else()
        # Fall back to searching header/lib manually under /usr/local
        find_path(HAILO_INCLUDE_DIR hailo/hailort.hpp PATHS /usr/local/include /usr/include)
        find_library(HAILO_LIB hailort PATHS /usr/local/lib /usr/lib)
        if(HAILO_INCLUDE_DIR AND HAILO_LIB)
            message(STATUS "HailoRT found manually: ${HAILO_LIB}")
            add_compile_definitions(WITH_HAILO)
            set(HailoRT_INCLUDE_DIRS ${HAILO_INCLUDE_DIR})
            set(HailoRT_LIBRARIES    ${HAILO_LIB})
        else()
            message(WARNING "WITH_HAILO=ON but HailoRT not found — building without Hailo support")
            set(WITH_HAILO OFF)
        endif()
    endif()
endif()

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${ONNXRUNTIME_INCLUDE_DIR}
)

# Source files
set(VISION_SOURCES
    src/vision/vision_detector_base.cpp
    src/vision/color_based_detector.cpp
    src/vision/yolo_detector.cpp
)

if(WITH_HAILO)
    list(APPEND VISION_SOURCES src/vision/hailo_detector.cpp)
endif()

set(ESTIMATION_SOURCES
    src/estimation/state_estimator_base.cpp
    src/estimation/kalman_filter_cv.cpp
    src/estimation/kalman_filter_ca.cpp
    src/estimation/imm_estimator.cpp
    src/estimation/distance_estimator.cpp
)

set(MOTOR_SOURCES
    src/motor/motor_controller_base.cpp
    src/motor/simplefoc_controller.cpp
    src/motor/mock_controller.cpp
)

set(STREAMING_SOURCES
    src/streaming/websocket_server.cpp
    src/streaming/mjpeg_streamer.cpp
)

# ── Embed web/index.html into a generated C++ header ─────────────────────────
set(WEB_DASHBOARD_HTML  "${CMAKE_CURRENT_SOURCE_DIR}/web/index.html")
set(WEB_DASHBOARD_HDR   "${CMAKE_CURRENT_BINARY_DIR}/include/web_dashboard.h")

add_custom_command(
    OUTPUT  "${WEB_DASHBOARD_HDR}"
    COMMAND ${CMAKE_COMMAND}
            -DINPUT="${WEB_DASHBOARD_HTML}"
            -DOUTPUT="${WEB_DASHBOARD_HDR}"
            -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed_html.cmake"
    DEPENDS "${WEB_DASHBOARD_HTML}"
            "${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed_html.cmake"
    COMMENT "Embedding web/index.html → ${WEB_DASHBOARD_HDR}"
)

add_custom_target(web_dashboard DEPENDS "${WEB_DASHBOARD_HDR}")

set(FACTORY_SOURCES
    src/factories/vision_factory.cpp
    src/factories/estimator_factory.cpp
    src/factories/motor_factory.cpp
)

set(CORE_SOURCES
    src/core/tracker_server.cpp
    src/core/config_manager.cpp
)

# Create static libraries for each subsystem
add_library(vision STATIC ${VISION_SOURCES})
target_include_directories(vision PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
target_link_libraries(vision ${OpenCV_LIBS} ${ONNXRUNTIME_LIB})
if(WITH_HAILO)
    target_include_directories(vision PRIVATE ${HailoRT_INCLUDE_DIRS})
    target_link_libraries(vision ${HailoRT_LIBRARIES})
endif()

add_library(estimation STATIC ${ESTIMATION_SOURCES})
target_link_libraries(estimation ${OpenCV_LIBS})

add_library(motor STATIC ${MOTOR_SOURCES})
target_link_libraries(motor Threads::Threads)

add_library(streaming STATIC ${STREAMING_SOURCES})
target_include_directories(streaming PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/include)
target_link_libraries(streaming Threads::Threads ${OpenCV_LIBS})
add_dependencies(streaming web_dashboard)

add_library(factories STATIC ${FACTORY_SOURCES})
target_link_libraries(factories vision estimation motor)

add_library(core STATIC ${CORE_SOURCES})
target_link_libraries(core factories streaming)

# Main executable
add_executable(tracker_server src/main.cpp)
target_link_libraries(tracker_server
    core
    factories
    vision
    estimation
    motor
    streaming
    ${OpenCV_LIBS}
    Threads::Threads
)
if(WITH_HAILO)
    target_link_libraries(tracker_server ${HailoRT_LIBRARIES})
endif()

# Tests
if(BUILD_TESTS)
    enable_testing()
    # TODO: Create test directory
    # add_subdirectory(test)
endif()

# Examples
if(BUILD_EXAMPLES)
    # TODO: Create examples directory
    # add_subdirectory(examples)
endif()

# Installation
install(TARGETS tracker_server
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
