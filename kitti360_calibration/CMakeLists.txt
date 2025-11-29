cmake_minimum_required(VERSION 3.12)
project(kitti360_calibration)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS})

# Create library
add_library(kitti360_calibration STATIC
    load_calibration.cpp
    load_calibration.h
)

# Link OpenCV libraries
target_link_libraries(kitti360_calibration ${OpenCV_LIBS})

# Create executable for testing
add_executable(test_calibration test_calibration_loading.cc)
target_link_libraries(test_calibration kitti360_calibration ${OpenCV_LIBS})

# Set output directories
set_target_properties(kitti360_calibration PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)

set_target_properties(test_calibration PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

# Copy binary to main directory after build
add_custom_command(TARGET test_calibration POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
        "${CMAKE_BINARY_DIR}/bin/test_calibration"
        "${CMAKE_SOURCE_DIR}/test_calibration"
    COMMENT "Copying test_calibration to main directory"
)

# Installation rules
install(TARGETS kitti360_calibration 
    ARCHIVE DESTINATION lib
)

install(FILES load_calibration.h 
    DESTINATION include/kitti360
)

install(TARGETS test_calibration 
    RUNTIME DESTINATION bin
)