@echo off
setlocal enabledelayedexpansion

echo ========================================
echo CloudStream Build Script for Windows
echo ========================================
echo.

:: Check for Visual Studio
where cl >nul 2>nul
if %errorlevel% neq 0 (
    echo ERROR: Visual Studio compiler not found in PATH.
    echo Please run this script from a Visual Studio Developer Command Prompt.
    exit /b 1
)

:: Parse command line arguments
set BUILD_TYPE=Release
set GENERATOR=Ninja
set BUILD_TESTS=ON

:parse_args
if "%~1"=="" goto :args_done
if /i "%~1"=="debug" set BUILD_TYPE=Debug
if /i "%~1"=="release" set BUILD_TYPE=Release
if /i "%~1"=="vs" set GENERATOR=Visual Studio 17 2022
if /i "%~1"=="ninja" set GENERATOR=Ninja
if /i "%~1"=="notests" set BUILD_TESTS=OFF
shift
goto :parse_args
:args_done

echo Build Configuration:
echo   Build Type: %BUILD_TYPE%
echo   Generator: %GENERATOR%
echo   Build Tests: %BUILD_TESTS%
echo.

:: Create build directory
set BUILD_DIR=build-windows-%BUILD_TYPE%
if not exist %BUILD_DIR% mkdir %BUILD_DIR%
cd %BUILD_DIR%

:: Configure with CMake
echo Configuring project...
if "%GENERATOR%"=="Ninja" (
    cmake .. -G Ninja -DCMAKE_BUILD_TYPE=%BUILD_TYPE% -DBUILD_TESTS=%BUILD_TESTS%
) else (
    cmake .. -G "%GENERATOR%" -A x64 -DCMAKE_BUILD_TYPE=%BUILD_TYPE% -DBUILD_TESTS=%BUILD_TESTS%
)

if %errorlevel% neq 0 (
    echo ERROR: CMake configuration failed!
    cd ..
    exit /b 1
)

:: Build
echo.
echo Building project...
cmake --build . --config %BUILD_TYPE% --parallel

if %errorlevel% neq 0 (
    echo ERROR: Build failed!
    cd ..
    exit /b 1
)

:: Run tests if enabled
if "%BUILD_TESTS%"=="ON" (
    echo.
    echo Running tests...
    ctest -C %BUILD_TYPE% --output-on-failure
)

:: Copy resources
echo.
echo Copying resources...
if not exist %BUILD_TYPE%\resources\shaders mkdir %BUILD_TYPE%\resources\shaders
xcopy /Y /E ..\resources\shaders %BUILD_TYPE%\resources\shaders >nul

:: Success
echo.
echo ========================================
echo Build completed successfully!
echo Executable: %BUILD_DIR%\%BUILD_TYPE%\CloudStream.exe
echo ========================================

cd ..
endlocal 