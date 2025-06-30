cmake_minimum_required(VERSION 3.16)
project(CloudStream VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set default build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Find required packages
find_package(Qt6 REQUIRED COMPONENTS Core Widgets OpenGL OpenGLWidgets Network)
find_package(OpenGL REQUIRED)
find_package(Boost 1.75 REQUIRED COMPONENTS system thread filesystem program_options)
find_package(Threads REQUIRED)

# PCL is optional but recommended
find_package(PCL 1.12 QUIET COMPONENTS common io filters visualization)

# OpenCV is optional
find_package(OpenCV QUIET)

# Set Qt auto MOC and UIC
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${Boost_INCLUDE_DIRS}
)

# Source files
set(SOURCES
    src/main.cpp
    src/core/PointCloud.cpp
    src/core/PointCloudProcessor.cpp
    src/network/NetworkManager.cpp
    src/network/StreamProtocol.cpp
    src/network/Server.cpp
    src/network/Client.cpp
    src/visualization/MainWindow.cpp
    src/visualization/PointCloudRenderer.cpp
    src/visualization/GLWidget.cpp
    src/utils/ThreadPool.cpp
    src/utils/Timer.cpp
    src/utils/Logger.cpp
)

# Header files
set(HEADERS
    include/core/PointCloud.h
    include/core/PointCloudProcessor.h
    include/network/NetworkManager.h
    include/network/StreamProtocol.h
    include/network/Server.h
    include/network/Client.h
    include/visualization/MainWindow.h
    include/visualization/PointCloudRenderer.h
    include/visualization/GLWidget.h
    include/utils/ThreadPool.h
    include/utils/Timer.h
    include/utils/Logger.h
    include/utils/MathUtils.h
)

# Shader files
set(SHADERS
    resources/shaders/point_cloud.vert
    resources/shaders/point_cloud.frag
)

# Create executable
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# Link libraries
target_link_libraries(${PROJECT_NAME}
    Qt6::Core
    Qt6::Widgets
    Qt6::OpenGL
    Qt6::OpenGLWidgets
    Qt6::Network
    OpenGL::GL
    ${Boost_LIBRARIES}
    Threads::Threads
)

# Link PCL if found
if(PCL_FOUND)
    include_directories(${PCL_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
    target_compile_definitions(${PROJECT_NAME} PRIVATE USE_PCL)
endif()

# Link OpenCV if found
if(OpenCV_FOUND)
    target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
    target_compile_definitions(${PROJECT_NAME} PRIVATE USE_OPENCV)
endif()

# Set compiler flags
if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /W4 /WX)
else()
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Copy shaders to build directory
foreach(SHADER ${SHADERS})
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${SHADER} 
                   ${CMAKE_CURRENT_BINARY_DIR}/${SHADER} COPYONLY)
endforeach()

# Installation
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
install(DIRECTORY resources/shaders DESTINATION share/${PROJECT_NAME}) 