@echo off
setlocal enabledelayedexpansion

:: --- CONFIGURATION ---
:: Paths to your compiled executables (Adjust if your build folder structure differs)
set DECIMATOR_EXE=adaptive_decimation\build\Release\curvature_decimator.exe
set SEGMENTER_EXE=segmentation\build\Release\region_detector.exe

:: Input and Output directories
set INPUT_DIR=NASA_obj
set OUT_DIR=pipeline_output
set DIR_SIMP=%OUT_DIR%\1_simplified_ply
set DIR_LABELS=%OUT_DIR%\2_segmented_labels
set DIR_VISUAL=%OUT_DIR%\3_segmented_visual

:: --- SETUP ---
echo [INFO] Setting up directories...
if not exist "%DIR_SIMP%" mkdir "%DIR_SIMP%"
if not exist "%DIR_LABELS%" mkdir "%DIR_LABELS%"
if not exist "%DIR_VISUAL%" mkdir "%DIR_VISUAL%"

:: Check if executables exist
if not exist "%DECIMATOR_EXE%" (
    echo [ERROR] Decimator executable not found at: %DECIMATOR_EXE%
    echo Did you compile it in Release mode?
    pause
    exit /b
)
if not exist "%SEGMENTER_EXE%" (
    echo [ERROR] Segmenter executable not found at: %SEGMENTER_EXE%
    pause
    exit /b
)

:: --- PROCESSING LOOP ---
echo [INFO] Starting pipeline...

for %%f in ("%INPUT_DIR%\*.obj") do (
    set "filename=%%~nf"
    echo.
    echo ----------------------------------------------------------------
    echo Processing: !filename!
    echo ----------------------------------------------------------------

    :: 1. RUN DECIMATION
    :: Assuming the decimator takes: [Input] [Output]
    echo [STEP 1] Decimating...
    "%DECIMATOR_EXE%" "%%f" "%DIR_SIMP%\!filename!.ply"

    :: 2. RUN SEGMENTATION
    :: Assuming the segmenter takes: [Input] [OutputLabels] [OutputVisual]
    :: Note: The segmenter likely takes the SIMPLIFIED ply from step 1, not the original obj
    echo [STEP 2] Segmenting...
    if exist "%DIR_SIMP%\!filename!.ply" (
        "%SEGMENTER_EXE%" "%DIR_SIMP%\!filename!.ply" "%DIR_LABELS%\!filename!.txt" "%DIR_VISUAL%\!filename!.ply"
    ) else (
        echo [ERROR] Decimation failed for !filename!, skipping segmentation.
    )
)

echo.
echo ----------------------------------------------------------------
echo [SUCCESS] Pipeline Finished.
pause