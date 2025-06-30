# CloudStream - Real-Time Point Cloud Streaming & Visualization

A high-performance C++ application for real-time point cloud streaming, processing, and visualization using modern C++ techniques and industry-standard libraries.

## Features

- **Real-time Point Cloud Streaming**: Stream point cloud data over network using Boost.Asio
- **3D Visualization**: Interactive 3D rendering using Qt and OpenGL
- **Multi-threaded Architecture**: Efficient processing using C++17 threading features
- **Point Cloud Processing**: Basic filtering and transformation operations
- **Network Protocol**: Custom TCP/UDP protocol for low-latency streaming
- **Performance Monitoring**: Real-time FPS and bandwidth statistics

## Technologies Used

- **C++17**: Modern C++ features for clean, efficient code
- **Qt 6**: Cross-platform GUI framework
- **OpenGL 4.5**: Hardware-accelerated 3D graphics
- **Boost Libraries**: Networking (Asio), threading, and utilities
- **PCL (Point Cloud Library)**: Point cloud processing
- **CMake**: Cross-platform build system
- **OpenCV**: Image processing for depth camera integration (optional)

## Architecture

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│   Data Source   │────▶│    Network      │────▶│   Visualizer    │
│  (Server/File)  │     │    Module       │     │   (Qt/OpenGL)   │
└─────────────────┘     └─────────────────┘     └─────────────────┘
         │                       │                        │
         ▼                       ▼                        ▼
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│ Point Cloud Gen │     │ Protocol Handler│     │   GPU Renderer  │
└─────────────────┘     └─────────────────┘     └─────────────────┘
```

## Building

### Prerequisites

- C++17 compatible compiler (GCC 8+, Clang 7+, MSVC 2019+)
- CMake 3.16+
- Qt 6.2+
- Boost 1.75+
- PCL 1.12+
- OpenGL 4.5 capable GPU

### Build Instructions

```bash
mkdir build && cd build
cmake ..
cmake --build . --config Release
```

## Usage

### Server Mode (Streaming)
```bash
./CloudStream --server --port 8080 --source sample.pcd
```

### Client Mode (Visualization)
```bash
./CloudStream --client --host localhost --port 8080
```

### Standalone Mode
```bash
./CloudStream --file sample.pcd
```

## Performance

- Supports streaming of 1M+ points at 30 FPS
- Multi-threaded point cloud processing pipeline
- GPU-accelerated rendering with instanced drawing
- Efficient memory management with custom allocators

## Future Enhancements

- CUDA acceleration for point cloud processing
- Machine learning integration for object detection
- Real-time compression algorithms
- Support for multiple point cloud formats
- VTK integration for advanced visualization

## License

MIT License - See LICENSE file for details

## Author

[Your Name] - Senior C++ Developer 