CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O3
LIBS = -lSDL2 -lSDL2_image
TARGET = fisheye_viewer
DUAL_TARGET = dual_fisheye_viewer
SINGLE_UNDISTORT_TARGET = single_undistort
SOURCE = main.cpp
DUAL_SOURCE = dual_main.cpp
SINGLE_UNDISTORT_SOURCE = single_undistort.cpp

# Check if we're on Ubuntu/Debian and need additional include paths
UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Linux)
    # Try to find SDL2 headers in common locations
    SDL2_INCLUDE := $(shell pkg-config --cflags sdl2 2>/dev/null || echo "-I/usr/include/SDL2")
    SDL2_LIBS := $(shell pkg-config --libs sdl2 SDL2_image 2>/dev/null || echo "-lSDL2 -lSDL2_image")
    
    # OpenCV configuration for dual viewer
    OPENCV_INCLUDE := $(shell pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv 2>/dev/null || echo "-I/usr/include/opencv4")
    OPENCV_LIBS := $(shell pkg-config --libs opencv4 2>/dev/null || pkg-config --libs opencv 2>/dev/null || echo "-lopencv_core -lopencv_imgproc -lopencv_calib3d -lopencv_imgcodecs")
    
    CXXFLAGS += $(SDL2_INCLUDE)
    LIBS = $(SDL2_LIBS)
    
    # Dual viewer specific flags and libs (includes OpenCV and calibration library)
    DUAL_CXXFLAGS = $(CXXFLAGS) $(OPENCV_INCLUDE)
    DUAL_LIBS = $(SDL2_LIBS) $(OPENCV_LIBS) -Lkitti360_calibration/build/lib -lkitti360_calibration
else
    # Fallback for non-Linux systems
    DUAL_CXXFLAGS = $(CXXFLAGS)
    DUAL_LIBS = $(LIBS) -lopencv_core -lopencv_imgproc -lopencv_calib3d -lopencv_imgcodecs -Lkitti360_calibration/build/lib -lkitti360_calibration
endif

all: $(TARGET) $(DUAL_TARGET) $(SINGLE_UNDISTORT_TARGET) calibration

$(TARGET): $(SOURCE)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SOURCE) $(LIBS)

$(DUAL_TARGET): $(DUAL_SOURCE) calibration
	$(CXX) $(DUAL_CXXFLAGS) -o $(DUAL_TARGET) $(DUAL_SOURCE) $(DUAL_LIBS)

$(SINGLE_UNDISTORT_TARGET): $(SINGLE_UNDISTORT_SOURCE) calibration
	$(CXX) $(DUAL_CXXFLAGS) -o $(SINGLE_UNDISTORT_TARGET) $(SINGLE_UNDISTORT_SOURCE) $(OPENCV_LIBS) -Lkitti360_calibration/build/lib -lkitti360_calibration

# Calibration library targets
calibration:
	@echo "Building calibration library..."
	@mkdir -p kitti360_calibration/build
	@cd kitti360_calibration/build && cmake .. && make

calibration-clean:
	@echo "Cleaning calibration build..."
	@rm -rf kitti360_calibration/build
	@rm -f kitti360_calibration/test_calibration

calibration-test: calibration
	@echo "Running calibration tests..."
	@cd kitti360_calibration && ./test_calibration

calibration-install-deps:
	@echo "Installing OpenCV dependencies for calibration and dual fisheye viewer..."
	@if command -v apt-get >/dev/null 2>&1; then \
		echo "Using apt-get (Ubuntu/Debian)"; \
		sudo apt-get update && sudo apt-get install -y libopencv-dev; \
	elif command -v yum >/dev/null 2>&1; then \
		echo "Using yum (RHEL/CentOS)"; \
		sudo yum install -y opencv-devel; \
	elif command -v dnf >/dev/null 2>&1; then \
		echo "Using dnf (Fedora)"; \
		sudo dnf install -y opencv-devel; \
	elif command -v pacman >/dev/null 2>&1; then \
		echo "Using pacman (Arch Linux)"; \
		sudo pacman -S opencv; \
	else \
		echo "Package manager not recognized. Please install OpenCV development libraries manually."; \
	fi

clean: calibration-clean
	rm -f $(TARGET) $(DUAL_TARGET) $(SINGLE_UNDISTORT_TARGET)

install-deps:
	@echo "Installing SDL2 dependencies..."
	@if command -v apt-get >/dev/null 2>&1; then \
		echo "Using apt-get (Ubuntu/Debian)"; \
		sudo apt-get update && sudo apt-get install -y libsdl2-dev libsdl2-image-dev; \
	elif command -v yum >/dev/null 2>&1; then \
		echo "Using yum (RHEL/CentOS)"; \
		sudo yum install -y SDL2-devel SDL2_image-devel; \
	elif command -v dnf >/dev/null 2>&1; then \
		echo "Using dnf (Fedora)"; \
		sudo dnf install -y SDL2-devel SDL2_image-devel; \
	elif command -v pacman >/dev/null 2>&1; then \
		echo "Using pacman (Arch Linux)"; \
		sudo pacman -S sdl2 sdl2_image; \
	else \
		echo "Package manager not recognized. Please install SDL2 and SDL2_image development libraries manually."; \
	fi

run: $(TARGET)
	@echo "Usage: ./$(TARGET) <image_directory>"
	@echo "Example: ./$(TARGET) /path/to/your/fisheye/images"

run-dual: $(DUAL_TARGET)
	@echo "Usage: ./$(DUAL_TARGET) <left_directory> <right_directory>"
	@echo "Example: ./$(DUAL_TARGET) /path/to/left/images /path/to/right/images"
	@echo "Note: Requires OpenCV and calibration data for fisheye undistortion"

run-single-undistort: $(SINGLE_UNDISTORT_TARGET)
	@echo "Usage: ./$(SINGLE_UNDISTORT_TARGET) <image_path>"
	@echo "Example: ./$(SINGLE_UNDISTORT_TARGET) /path/to/fisheye/image.png"
	@echo "Shows original (left) vs undistorted (right) side-by-side"

.PHONY: all clean install-deps run run-dual run-single-undistort calibration calibration-clean calibration-test calibration-install-deps