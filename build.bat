@echo off
REM Build script for CloudStream on Windows

echo Building CloudStream...
echo.

REM Create build directory
if not exist build mkdir build
cd build

REM Configure with CMake
echo Configuring project...
cmake -G "Visual Studio 17 2022" -A x64 ..
if %errorlevel% neq 0 (
    echo CMake configuration failed!
    exit /b %errorlevel%
)

echo.
echo Building in Release mode...
cmake --build . --config Release --parallel
if %errorlevel% neq 0 (
    echo Build failed!
    exit /b %errorlevel%
)

echo.
echo Build completed successfully!
echo Executable: build\Release\CloudStream.exe
echo.

cd ..
pause 