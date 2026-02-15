@echo off
setlocal

:: --- CONFIGURATION ---
:: Path to your vcpkg toolchain (Based on your previous logs)
set "VCPKG_CMAKE=D:\Work\vcpkg\scripts\buildsystems\vcpkg.cmake"

:: --- STEP 1: CLEANUP ---
if exist build (
    echo [INFO] Found existing build folder. Deleting it...
    rmdir /s /q build
    
    :: Double check if delete was successful (sometimes files are locked)
    if exist build (
        echo [ERROR] Could not delete 'build' folder. Is a file open?
        pause
        exit /b 1
    )
)

:: --- STEP 2: PREPARE ---
echo [INFO] Creating new build directory...
mkdir build
cd build

:: --- STEP 3: CONFIGURE ---
echo [INFO] Configuring with CMake...
:: We use the vcpkg toolchain file we defined above
cmake .. -DCMAKE_TOOLCHAIN_FILE="%VCPKG_CMAKE%" -DVCPKG_TARGET_TRIPLET=x64-windows
if %errorlevel% neq 0 (
    echo [ERROR] CMake configuration failed.
    pause
    exit /b %errorlevel%
)

:: --- STEP 4: COMPILE ---
echo [INFO] Building project (Release mode)...
cmake --build . --config Release
if %errorlevel% neq 0 (
    echo [ERROR] Compilation failed.
    pause
    exit /b %errorlevel%
)

echo.
echo ==========================================
echo [SUCCESS] Build finished successfully.
echo Executable should be in: build\Release\
echo ==========================================
pause