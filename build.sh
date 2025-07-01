#!/bin/bash
# Build script for CloudStream on Linux/macOS

echo "Building CloudStream..."
echo

# Create build directory
mkdir -p build
cd build

# Configure with CMake
echo "Configuring project..."
cmake -DCMAKE_BUILD_TYPE=Release ..
if [ $? -ne 0 ]; then
    echo "CMake configuration failed!"
    exit 1
fi

echo
echo "Building..."
cmake --build . --parallel $(nproc)
if [ $? -ne 0 ]; then
    echo "Build failed!"
    exit 1
fi

echo
echo "Build completed successfully!"
echo "Executable: build/CloudStream"
echo

cd .. 