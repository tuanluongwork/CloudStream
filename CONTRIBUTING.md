# Contributing to CloudStream

Thank you for your interest in contributing to CloudStream! This document provides guidelines and instructions for contributing to the project.

## Code of Conduct

Please note that this project adheres to a code of conduct. By participating, you are expected to uphold professional standards and treat all contributors with respect.

## Getting Started

1. Fork the repository
2. Clone your fork: `git clone https://github.com/your-username/CloudStream.git`
3. Create a feature branch: `git checkout -b feature/your-feature-name`
4. Make your changes
5. Commit with descriptive messages: `git commit -m "Add: Feature description"`
6. Push to your fork: `git push origin feature/your-feature-name`
7. Create a Pull Request

## Development Setup

### Prerequisites

- C++17 compatible compiler
- CMake 3.16+
- Qt 6.2+
- Boost 1.75+
- OpenGL 4.5 capable GPU

### Building

```bash
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
cmake --build .
```

## Coding Standards

### C++ Style Guide

- Use modern C++ features (C++17)
- Follow RAII principles
- Prefer smart pointers over raw pointers
- Use `const` wherever possible
- Follow the rule of zero/three/five

### Naming Conventions

- Classes: `PascalCase`
- Functions/Methods: `camelCase`
- Variables: `snake_case`
- Constants: `UPPER_CASE`
- Private members: `member_name_`

### Code Organization

```cpp
// Header files (.h)
#pragma once

namespace CloudStream {

class ExampleClass {
public:
    // Public methods
    
private:
    // Private members
};

} // namespace CloudStream
```

### Documentation

- Use Doxygen-style comments for public APIs
- Include brief descriptions for all public methods
- Document complex algorithms and non-obvious code

```cpp
/**
 * @brief Processes the point cloud using the specified algorithm
 * @param cloud Input point cloud
 * @param params Algorithm parameters
 * @return Processed point cloud
 */
PointCloud::Ptr processPointCloud(PointCloud::Ptr cloud, const Parameters& params);
```

## Testing

- Write unit tests for new features
- Ensure all tests pass before submitting PR
- Test on multiple platforms if possible

## Performance Considerations

- Profile code for performance bottlenecks
- Use parallel algorithms where appropriate
- Minimize memory allocations in hot paths
- Consider cache locality

## Pull Request Process

1. Update documentation for any API changes
2. Add tests for new functionality
3. Ensure CI/CD passes
4. Request review from maintainers
5. Address review feedback promptly

## Commit Message Format

Use conventional commit format:

- `feat:` New feature
- `fix:` Bug fix
- `docs:` Documentation changes
- `style:` Code style changes
- `refactor:` Code refactoring
- `perf:` Performance improvements
- `test:` Test additions/changes
- `chore:` Build process/auxiliary tool changes

Example: `feat: Add real-time compression for network streaming`

## Areas for Contribution

- **Performance Optimization**: GPU acceleration, SIMD optimizations
- **File Format Support**: Add support for more point cloud formats
- **Algorithms**: Implement advanced point cloud processing algorithms
- **UI/UX**: Improve the user interface and experience
- **Documentation**: Improve documentation and examples
- **Testing**: Increase test coverage

## Questions?

Feel free to open an issue for any questions about contributing! 