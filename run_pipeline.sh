#!/bin/bash

# --- CONFIGURATION ---
# Script to exit immediately if a command exits with a non-zero status.
set -e

# 1. Input directory
INPUT_DIR="NASA_obj"

# 2. Output directories
OUTPUT_BASE="pipeline_output"
SIMPLIFIED_PLY_DIR="$OUTPUT_BASE/1_simplified_ply"
SEGMENTED_LABELS_DIR="$OUTPUT_BASE/2_segmented_labels"
SEGMENTED_VISUAL_DIR="$OUTPUT_BASE/3_segmented_visual"
ANALYSIS_DIR="$OUTPUT_BASE/4_region_analysis"

# 3. Executable paths
DECIMATOR_EXEC="adaptive_decimation/build/curvature_decimator"
SEGMENTATOR_EXEC="segmentation/build/region_detector"
ANALYZER_EXEC="region_analysis/build/analyzer"

# --- PIPELINE EXECUTION ---

echo "--- Starting 3D Model Processing Pipeline ---"

# Create all output directories
mkdir -p "$SIMPLIFIED_PLY_DIR"
mkdir -p "$SEGMENTED_LABELS_DIR"
mkdir -p "$SEGMENTED_VISUAL_DIR"
mkdir -p "$ANALYSIS_DIR"

echo "Input folder: $INPUT_DIR"
echo "Output folder: $OUTPUT_BASE"

# Check if input directory is empty
if [ -z "$(ls -A $INPUT_DIR)" ]; then
   echo "Error: Input directory '$INPUT_DIR' is empty. Nothing to process."
   exit 1
fi

# Loop over all .obj files in the input directory
for input_file in "$INPUT_DIR"/*.obj; do
    
    filename=$(basename "$input_file" .obj)
    echo "-------------------------------------------------"
    echo "Processing model: $filename"
    echo "-------------------------------------------------"

    # Define paths for intermediate and final files
    simplified_ply_path="$SIMPLIFIED_PLY_DIR/${filename}_simplified.ply"
    labels_path="$SEGMENTED_LABELS_DIR/${filename}_labels.txt"
    visual_path="$SEGMENTED_VISUAL_DIR/${filename}_segmented.ply"
    analysis_path="$ANALYSIS_DIR/${filename}_analysis.csv"
    
    # Create a temporary directory for the analysis step
    analysis_temp_dir=$(mktemp -d)

    # Step 1: Adaptive Decimation (writes PLY directly)
    echo "  [1/3] Simplifying model and saving to PLY..."
    $DECIMATOR_EXEC "$input_file" "$simplified_ply_path"

    # Step 2: Segmentation
    echo "  [2/3] Performing segmentation..."
    $SEGMENTATOR_EXEC "$simplified_ply_path" "$visual_path" "$labels_path"

    # Step 3: Region Analysis
    echo "  [3/3] Analyzing segmented regions..."
    # The analyzer expects specific filenames, so we copy and rename
    cp "$simplified_ply_path" "$analysis_temp_dir/model.ply"
    cp "$labels_path" "$analysis_temp_dir/face_labels.txt"
    
    # Run the analyzer on the temporary directory
    $ANALYZER_EXEC "$analysis_temp_dir"
    
    # Move the result to the final analysis directory
    mv "$analysis_temp_dir/meteorite_analysis.csv" "$analysis_path"
    
    # Clean up the temporary directory
    rm -r "$analysis_temp_dir"

    echo "Finished processing $filename."
done

echo "--- Pipeline execution finished successfully! ---"
echo "All outputs are available in the '$OUTPUT_BASE' directory."