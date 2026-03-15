import json
import os

def extract_descriptors_from_full_data(input_json_path, output_json_path):
    """
    Reads the full meteorite data JSON and extracts only the 'spectrum' 
    (the descriptor) for each model, maintaining the subfolder structure.
    """
    if not os.path.exists(input_json_path):
        print(f"Error: Could not find '{input_json_path}'")
        return

    print(f"Loading full data from '{input_json_path}'...")
    with open(input_json_path, 'r') as f:
        full_data = json.load(f)

    descriptors_only = {}
    total_models = 0

    # Iterate through the subfolders (e.g., 'APOLLO', 'ANTARCTIC')
    for subfolder, models in full_data.items():
        descriptors_only[subfolder] = {}
        
        # Iterate through each 3D model in the subfolder
        for model_name, model_data in models.items():
            # Extract just the spectrum list
            if "spectrum" in model_data:
                descriptors_only[subfolder][model_name] = model_data["spectrum"]
                total_models += 1
            else:
                print(f"Warning: No spectrum found for {subfolder}/{model_name}")

    # Save the cleaned data to the new JSON file
    print(f"Extracted {total_models} descriptors across {len(descriptors_only)} subfolders.")
    print(f"Saving to '{output_json_path}'...")
    
    with open(output_json_path, 'w') as out_file:
        json.dump(descriptors_only, out_file, indent=4)
        
    print("Done!")

if __name__ == "__main__":
    # Define your file paths here
    input_file = "meteorite_data.json"
    output_file = "descriptors2.json"
    
    extract_descriptors_from_full_data(input_file, output_file)