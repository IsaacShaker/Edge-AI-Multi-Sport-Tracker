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
option(ENABLE_OPENCV "Enable OpenCV support" ON)

# Find packages
find_package(Threads REQUIRED)

if(ENABLE_OPENCV)
    find_package(OpenCV REQUIRED)
    add_compile_definitions(ENABLE_OPENCV)
endif()

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

# Source files
set(VISION_SOURCES
    src/vision/vision_detector_base.cpp
    src/vision/color_based_detector.cpp
    src/vision/yolo_detector.cpp
)

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
target_link_libraries(vision ${OpenCV_LIBS})

add_library(estimation STATIC ${ESTIMATION_SOURCES})
target_link_libraries(estimation ${OpenCV_LIBS})

add_library(motor STATIC ${MOTOR_SOURCES})
target_link_libraries(motor Threads::Threads)

add_library(streaming STATIC ${STREAMING_SOURCES})
target_link_libraries(streaming Threads::Threads ${OpenCV_LIBS})

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
