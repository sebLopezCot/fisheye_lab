CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O3
LIBS = -lSDL2 -lSDL2_image
TARGET = fisheye_viewer
DUAL_TARGET = dual_fisheye_viewer
SOURCE = main.cpp
DUAL_SOURCE = dual_main.cpp

# Check if we're on Ubuntu/Debian and need additional include paths
UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Linux)
    # Try to find SDL2 headers in common locations
    SDL2_INCLUDE := $(shell pkg-config --cflags sdl2 2>/dev/null || echo "-I/usr/include/SDL2")
    SDL2_LIBS := $(shell pkg-config --libs sdl2 SDL2_image 2>/dev/null || echo "-lSDL2 -lSDL2_image")
    CXXFLAGS += $(SDL2_INCLUDE)
    LIBS = $(SDL2_LIBS)
endif

all: $(TARGET) $(DUAL_TARGET)

$(TARGET): $(SOURCE)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SOURCE) $(LIBS)

$(DUAL_TARGET): $(DUAL_SOURCE)
	$(CXX) $(CXXFLAGS) -o $(DUAL_TARGET) $(DUAL_SOURCE) $(LIBS)

clean:
	rm -f $(TARGET) $(DUAL_TARGET)

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

.PHONY: all clean install-deps run run-dual