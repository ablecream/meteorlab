import json
import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
from sklearn.preprocessing import StandardScaler
from sklearn.metrics.pairwise import cosine_similarity

# --- CONFIGURATION ---
DESCRIPTORS_JSON = "descriptors.json"
METADATA_JSON = "../NASA_metadata2.json"

def load_data():
    """Loads metadata and descriptors, extracting the new macro-categories."""
    # 1. Load Metadata to get Macro-Categories
    with open(METADATA_JSON, 'r') as f:
        meta = json.load(f)
        
    name_to_macro = {}
    for key, value in meta.items():
        if key == "metadata": continue
        for item in value:
            # Fallback to "Unknown" just in case a field is missing
            name_to_macro[item["name"]] = item.get("macro_category", "Unknown")

    # 2. Load Descriptors
    with open(DESCRIPTORS_JSON, 'r') as f:
        desc_data = json.load(f)
        
    names = []
    features = []
    
    # Handle nested JSON
    for subfolder, models in desc_data.items():
        if isinstance(models, dict):
            for name, feat in models.items():
                names.append(name)
                features.append(feat)
        else:
            names.append(subfolder)
            features.append(models)
            
    return names, np.array(features), name_to_macro

def generate_validation_heatmap(names, features, name_to_macro, sort_by="macro"):
    """Calculates pairwise similarity and generates a heatmap sorted by macro-category."""
    print("Cleaning features and calculating similarity...")
    
    # 1. Drop the l=0 component (the base sphere)
    #features_no_l0 = features[:, 1:]
    features_no_l0 = features
    
    # 2. Standardize the features to balance low and high frequencies
    scaler = StandardScaler()
    features_scaled = scaler.fit_transform(features_no_l0)
    
    # 3. Calculate Cosine Similarity
    similarity_matrix = cosine_similarity(features_scaled)
    
    # Build the Pandas DataFrame
    df = pd.DataFrame(similarity_matrix, index=names, columns=names)
    
    # Map the macro-categories to the dataframe
    macros = [name_to_macro.get(n, "Unknown") for n in names]
    df['Macro_Category'] = macros
    
    # Sort the DataFrame
    if sort_by == "macro":
        print("Sorting matrix by Structural Macro-Category...")
        df = df.sort_values(by='Macro_Category')
        
        # Symmetrically sort the columns
        sorted_names = df.index.tolist()
        df = df[sorted_names] 
        sorted_macros = [name_to_macro.get(n, "Unknown") for n in sorted_names]
    else:
        sorted_names = names
        sorted_macros = macros
        df = df.drop(columns=['Macro_Category'])

    # Plotting the Heatmap
    print("Generating Heatmap Visualization...")
    plt.figure(figsize=(14, 12))
    
    # Update labels to show the macro-category
    display_labels = [f"{n[:8]}... ({m})" for n, m in zip(sorted_names, sorted_macros)]
    
    ax = sns.heatmap(
        df, 
        cmap="magma",       
        xticklabels=display_labels, 
        yticklabels=display_labels,
        linewidths=0.5,     
        linecolor='black'
    )
    
    plt.title("Meteorite Geometric Similarity Matrix (Sorted by Macro-Category)", fontsize=16, pad=20)
    plt.xticks(rotation=90, fontsize=8)
    plt.yticks(rotation=0, fontsize=8)
    
    plt.tight_layout()
    output_filename = "similarity_heatmap_macro.png"
    plt.savefig(output_filename, dpi=300)
    plt.show()
    
    print(f"Heatmap successfully saved as {output_filename}")

if __name__ == "__main__":
    names, features, name_to_macro = load_data()
    
    if len(names) > 0:
        generate_validation_heatmap(names, features, name_to_macro, sort_by="macro")
    else:
        print("Error: No data loaded.")