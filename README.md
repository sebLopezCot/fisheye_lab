# Fisheye Camera Image Viewer

A high-performance C++ application using SDL2 to visualize fisheye camera images with smooth navigation and intelligent prefetching.

## Features

- **Fast Image Navigation**: Use left/right arrow keys to flip through images
- **Intelligent Prefetching**: Automatically loads nearby images into memory for smooth navigation
- **Multi-format Support**: Handles JPG, JPEG, and PNG image formats
- **Automatic Scaling**: Images are scaled to fit the window while maintaining aspect ratio
- **Sorted Loading**: Images are loaded in ascending filename order

## Dependencies

- SDL2 development libraries
- SDL2_image development libraries
- C++17 compatible compiler

## Installation

1. Install SDL2 dependencies:
   ```bash
   make install-deps
   ```

2. Build the application:
   ```bash
   make
   ```

## Usage

```bash
./fisheye_viewer <path_to_image_directory>
```

### Example:
```bash
./fisheye_viewer /home/user/fisheye_photos
```

## Controls

- **Left Arrow**: Previous image
- **Right Arrow**: Next image
- **ESC**: Quit application
- **Window Resize**: Supported - images will scale automatically

## Performance Features

- **GPU Acceleration**: Uses hardware-accelerated SDL2 renderer
- **Memory Prefetching**: Loads up to 20 images ahead and behind current position
- **Multithreaded Loading**: Background thread handles image loading without blocking UI
- **Efficient Scaling**: Real-time image scaling with aspect ratio preservation

## Build Options

- `make`: Build the application
- `make clean`: Remove built files
- `make install-deps`: Install system dependencies
- `make run`: Show usage instructions

## System Requirements

- Linux (tested on Ubuntu/Debian, supports other distributions)
- Sufficient RAM for image prefetching (program will use several GB for large image sets)
- OpenGL-capable graphics hardware (recommended for best performance)