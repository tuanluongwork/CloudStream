# CloudStream Architecture Documentation

## Overview

CloudStream is a high-performance, real-time point cloud streaming and visualization application built with modern C++17. The architecture follows a modular design with clear separation of concerns between networking, data processing, and visualization components.

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                           CloudStream Application                    │
├─────────────────────────────────────────────────────────────────────┤
│                         Visualization Layer                          │
│  ┌─────────────┐  ┌──────────────────┐  ┌────────────────────┐    │
│  │ MainWindow  │  │ PointCloudRenderer│  │     GLWidget       │    │
│  │   (Qt UI)   │  │  (OpenGL 4.5)     │  │ (3D Interaction)   │    │
│  └─────────────┘  └──────────────────┘  └────────────────────┘    │
├─────────────────────────────────────────────────────────────────────┤
│                           Core Layer                                 │
│  ┌─────────────┐  ┌──────────────────┐  ┌────────────────────┐    │
│  │ PointCloud  │  │PointCloudProcessor│  │   ThreadPool       │    │
│  │(Data Model) │  │  (Algorithms)     │  │ (Parallelization)  │    │
│  └─────────────┘  └──────────────────┘  └────────────────────┘    │
├─────────────────────────────────────────────────────────────────────┤
│                         Network Layer                                │
│  ┌─────────────┐  ┌──────────────────┐  ┌────────────────────┐    │
│  │   Server    │  │     Client       │  │  StreamProtocol    │    │
│  │(Boost.Asio) │  │  (Boost.Asio)    │  │ (Binary Protocol)  │    │
│  └─────────────┘  └──────────────────┘  └────────────────────┘    │
├─────────────────────────────────────────────────────────────────────┤
│                         Utility Layer                                │
│  ┌─────────────┐  ┌──────────────────┐  ┌────────────────────┐    │
│  │   Logger    │  │      Timer       │  │    MathUtils       │    │
│  │ (Singleton) │  │  (Performance)   │  │  (GLM Helpers)     │    │
│  └─────────────┘  └──────────────────┘  └────────────────────┘    │
└─────────────────────────────────────────────────────────────────────┘
```

## Component Details

### 1. Core Components

#### PointCloud
- **Purpose**: Central data structure for point cloud storage
- **Key Features**:
  - Efficient memory layout with structure-of-arrays design
  - Thread-safe operations with mutex protection
  - Bounding box computation and caching
  - Serialization/deserialization support
  - Statistical outlier removal and downsampling

#### PointCloudProcessor
- **Purpose**: Algorithms for point cloud processing
- **Key Features**:
  - Parallel processing using ThreadPool
  - Normal estimation using PCA
  - Euclidean clustering for segmentation
  - Feature extraction (planarity, sphericity, linearity)
  - Various filtering operations

#### ThreadPool
- **Purpose**: Efficient task parallelization
- **Key Features**:
  - Work-stealing queue implementation
  - Priority-based task scheduling
  - Performance monitoring
  - Automatic load balancing

### 2. Network Components

#### StreamProtocol
- **Purpose**: Binary protocol for efficient point cloud streaming
- **Message Types**:
  - Handshake/Authentication
  - Point Cloud Header (metadata)
  - Point Cloud Data (chunked transfer)
  - Control Commands (start/stop/pause)
  - Status Updates (bandwidth, FPS, etc.)

#### Server
- **Purpose**: Multi-client point cloud streaming server
- **Key Features**:
  - Asynchronous I/O with Boost.Asio
  - Multiple client support
  - Chunked data transfer
  - Bandwidth management
  - Real-time status broadcasting

#### Client
- **Purpose**: Network client for receiving point cloud streams
- **Key Features**:
  - Automatic reconnection
  - Chunk assembly
  - Callback-based architecture
  - Performance monitoring

### 3. Visualization Components

#### MainWindow
- **Purpose**: Main application window and UI management
- **Key Features**:
  - Qt-based UI with docking windows
  - Menu system and toolbars
  - Settings management
  - Statistics display
  - File I/O operations

#### GLWidget
- **Purpose**: OpenGL rendering widget with camera controls
- **Key Features**:
  - Mouse-based camera navigation
  - Orthographic/perspective projection
  - Grid and axes display
  - FPS monitoring

#### PointCloudRenderer
- **Purpose**: High-performance GPU rendering
- **Key Features**:
  - Modern OpenGL 4.5 with VAO/VBO
  - Custom GLSL shaders
  - Point sprites with distance attenuation
  - Phong lighting model
  - Frustum culling
  - Level-of-detail (LOD) support

## Data Flow

### Streaming Mode
```
1. Server loads point cloud from file
2. Server chunks data into manageable packets
3. Protocol encodes chunks with headers
4. Network layer transmits via TCP
5. Client receives and assembles chunks
6. Client triggers callback with complete point cloud
7. MainWindow updates GLWidget
8. Renderer uploads to GPU and displays
```

### Standalone Mode
```
1. User selects file via MainWindow
2. File is loaded into PointCloud object
3. Optional processing via PointCloudProcessor
4. Direct update to GLWidget
5. Renderer handles GPU upload and display
```

## Threading Model

- **Main Thread**: Qt UI and OpenGL rendering
- **Network I/O Threads**: Boost.Asio event loops (2 threads)
- **Worker Thread Pool**: Point cloud processing (hardware_concurrency threads)
- **Broadcast Thread**: Server status updates

## Performance Optimizations

1. **Memory Management**
   - Custom memory pool for frequent allocations
   - Structure-of-arrays layout for cache efficiency
   - Move semantics throughout

2. **Rendering**
   - Instanced rendering for large point counts
   - Frustum culling to skip invisible points
   - Level-of-detail based on distance
   - GPU-based point size calculation

3. **Networking**
   - Zero-copy buffer management
   - Chunked transfer to avoid memory spikes
   - Compression support (prepared but not implemented)

4. **Processing**
   - Parallel algorithms using std::execution
   - SIMD-friendly data layout
   - Lazy evaluation for expensive computations

## Security Considerations

1. **Network Security**
   - Input validation on all network messages
   - Maximum packet size limits
   - Checksum verification
   - Connection rate limiting (TODO)

2. **File Security**
   - Path traversal prevention
   - File size limits
   - Format validation

## Future Enhancements

1. **Compression**: Implement LZ4/ZLIB compression for network transfer
2. **CUDA Support**: GPU-accelerated point cloud processing
3. **Machine Learning**: Integration for object detection/segmentation
4. **VR Support**: Stereoscopic rendering for VR headsets
5. **Cloud Storage**: S3/Azure blob storage integration 