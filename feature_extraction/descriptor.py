import os
import json
import numpy as np
import trimesh
import pyshtools as pysh
import matplotlib.pyplot as plt
from pathlib import Path
from tqdm import tqdm

def get_sh_data(mesh_path, lmax=15):
    """Extracts both the power spectrum (fingerprint) and full coefficients."""
    mesh = trimesh.load(mesh_path, force='mesh')
    mesh.apply_translation(-mesh.centroid)
    mesh.apply_scale(1.0 / mesh.scale)

    nlat = 2 * lmax + 2
    nlon = 2 * nlat
    
    lats = np.linspace(90, -90, nlat)
    lons = np.linspace(0, 360, nlon, endpoint=False)
    lon_grid, lat_grid = np.meshgrid(lons, lats)
    
    theta = np.radians(90 - lat_grid)
    phi = np.radians(lon_grid)
    
    x = np.sin(theta) * np.cos(phi)
    y = np.sin(theta) * np.sin(phi)
    z = np.cos(theta)
    
    ray_directions = np.stack((x, y, z), axis=-1).reshape(-1, 3)
    ray_origins = np.zeros_like(ray_directions)
    
    locations, index_ray, _ = mesh.ray.intersects_location(
        ray_origins=ray_origins,
        ray_directions=ray_directions,
        multiple_hits=False
    )
    
    radii_flat = np.zeros(len(ray_directions))
    distances = np.linalg.norm(locations, axis=1)
    radii_flat[index_ray] = distances
    
    radius_grid = radii_flat.reshape(nlat, nlon)
    
    grid = pysh.SHGrid.from_array(radius_grid)
    clm = grid.expand()
    
    return {
        "spectrum": clm.spectrum().tolist(),
        "coefficients": clm.to_array().tolist()
    }

def batch_process_to_json(models_by_subfolder, output_data_path, output_desc_path, lmax=15):
    """Processes organized lists of 3D models and saves data to nested JSON files."""
    results_full = {}
    results_desc = {}
    failed_models = []
    
    for subfolder, paths in models_by_subfolder.items():
        results_full[subfolder] = {}
        results_desc[subfolder] = {}
        
        for path in tqdm(paths, desc=f"Processing {subfolder}"):
            model_name = os.path.splitext(os.path.basename(path))[0]
            try:
                sh_data = get_sh_data(path, lmax)
                
                # Assign full data (spectrum + coefficients) to the main JSON dict
                results_full[subfolder][model_name] = sh_data
                
                # Assign ONLY the spectrum descriptor to the descriptors JSON dict
                results_desc[subfolder][model_name] = sh_data["spectrum"]
                
            except Exception as e:
                failed_models.append((f"{subfolder}/{model_name}", str(e)))
                
    # Save the full reconstruction data
    with open(output_data_path, 'w') as json_file:
        json.dump(results_full, json_file, indent=4)
        
    # Save the standalone descriptors
    with open(output_desc_path, 'w') as desc_file:
        json.dump(results_desc, desc_file, indent=4)
        
    print(f"\nBatch processing complete.")
    print(f" -> Full Reconstruction Data saved to: {output_data_path}")
    print(f" -> Clean Descriptors saved to: {output_desc_path}")
    
    if failed_models:
        print(f"\nFailed to process {len(failed_models)} models:")
        for name, err in failed_models:
            print(f"  - {name}: {err}")

def create_trimesh_from_sh_grid(grid_recon):
    """Converts a PySHTools expanded grid back into a proper trimesh.Trimesh object."""
    radii = grid_recon.data
    lats = grid_recon.lats()
    lons = grid_recon.lons()
    nlat, nlon = radii.shape
    
    lon_grid, lat_grid = np.meshgrid(lons, lats)
    theta = np.radians(90 - lat_grid)
    phi = np.radians(lon_grid)
    
    x = radii * np.sin(theta) * np.cos(phi)
    y = radii * np.sin(theta) * np.sin(phi)
    z = radii * np.cos(theta)
    
    vertices = np.stack([x.flatten(), y.flatten(), z.flatten()], axis=1)
    
    faces = []
    for i in range(nlat - 1):
        for j in range(nlon):
            next_j = (j + 1) % nlon
            p1 = i * nlon + j
            p2 = i * nlon + next_j
            p3 = (i + 1) * nlon + j
            p4 = (i + 1) * nlon + next_j
            faces.append([p1, p2, p3])
            faces.append([p2, p4, p3])
            
    recon_mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=True)
    recon_mesh.fix_normals()
    return recon_mesh

def export_reconstructions(json_path, output_folder):
    """
    Reads the nested JSON file, reconstructs all models, and exports them to matched subfolders.
    """
    os.makedirs(output_folder, exist_ok=True)
    
    with open(json_path, 'r') as f:
        data = json.load(f)
        
    print(f"\nExporting reconstructed models to '{output_folder}'...")
    
    for subfolder, models in data.items():
        # Create matching subfolder in the output directory
        sub_out_dir = os.path.join(output_folder, subfolder)
        os.makedirs(sub_out_dir, exist_ok=True)
        
        for model_name, model_data in tqdm(models.items(), desc=f"Exporting {subfolder}"):
            coeffs_array = np.array(model_data["coefficients"])
            clm = pysh.SHCoeffs.from_array(coeffs_array)
            grid_recon = clm.expand()
            
            recon_mesh = create_trimesh_from_sh_grid(grid_recon)
            
            export_path = os.path.join(sub_out_dir, f"{model_name}_recon.obj")
            recon_mesh.export(export_path)

def visualize_reconstructions(json_path, original_models_paths, num_to_visualize=3):
    """Visualizes models by locating their data inside the nested JSON."""
    with open(json_path, 'r') as f:
        data = json.load(f)
        
    models_to_plot = original_models_paths[:num_to_visualize]
    num_models = len(models_to_plot)
    
    if num_models == 0:
        return

    fig = plt.figure(figsize=(5 * num_models, 10))
    
    for i, path in enumerate(models_to_plot):
        p = Path(path)
        model_name = p.stem
        subfolder = p.parent.name # Extracts 'APOLLO' or 'ANTARCTIC' from the path
        
        orig_mesh = trimesh.load(path, force='mesh')
        orig_mesh.apply_translation(-orig_mesh.centroid)
        orig_mesh.apply_scale(1.0 / orig_mesh.scale)
        
        # Access the specific subfolder in the JSON data
        model_data = data.get(subfolder, {}).get(model_name)
        
        if not model_data:
            print(f"Skipping visualization for {subfolder}/{model_name} - no data found.")
            continue
            
        coeffs_array = np.array(model_data["coefficients"])
        clm = pysh.SHCoeffs.from_array(coeffs_array)
        grid_recon = clm.expand()
        
        recon_mesh = create_trimesh_from_sh_grid(grid_recon)
        
        ax_orig = fig.add_subplot(2, num_models, i + 1, projection='3d')
        ax_orig.plot_trisurf(orig_mesh.vertices[:, 0], orig_mesh.vertices[:, 1], 
                             orig_mesh.faces, orig_mesh.vertices[:, 2], 
                             color='lightblue', edgecolor='none', alpha=0.9)
        ax_orig.set_title(f"Original:\n{model_name}")
        ax_orig.axis('off')
        
        ax_recon = fig.add_subplot(2, num_models, i + 1 + num_models, projection='3d')
        ax_recon.plot_trisurf(recon_mesh.vertices[:, 0], recon_mesh.vertices[:, 1], 
                               recon_mesh.faces, recon_mesh.vertices[:, 2], 
                               color='lightgreen', edgecolor='none', alpha=0.9)
        ax_recon.set_title(f"Reconstructed\n(lmax={clm.lmax})")
        ax_recon.axis('off')

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    folder_path = Path("../Nasa_obj") 
    output_mesh_folder = Path("../Nasa_reconstructed") 
    valid_extensions = {'.obj', '.stl', '.ply', '.off'}
    
    models_by_subfolder = {}
    
    # Identify all subfolders and group valid 3D files
    if folder_path.exists():
        for subfolder in [d for d in folder_path.iterdir() if d.is_dir()]:
            files = [str(f) for f in subfolder.iterdir() if f.is_file() and f.suffix.lower() in valid_extensions]
            if files:
                models_by_subfolder[subfolder.name] = files

    # Flatten the list just for the visualizer
    flat_model_paths = [f for files in models_by_subfolder.values() for f in files]

    if not models_by_subfolder:
        print(f"No 3D models or subfolders found in '{folder_path}'. Please ensure folders like 'ANTARCTIC' or 'APOLLO' exist.")
    else:
        total_models = sum(len(v) for v in models_by_subfolder.values())
        print(f"Found {total_models} models across {len(models_by_subfolder)} subfolders. Starting extraction...")
        
        output_data_file = "meteorite_data.json"
        output_desc_file = "descriptors.json"
        
        # 1. Extract data and build BOTH nested JSONs
        batch_process_to_json(models_by_subfolder, output_data_file, output_desc_file, lmax=20)
        
        # 2. Reconstruct models into mapped subdirectories using the full data JSON
        export_reconstructions(output_data_file, output_mesh_folder)
        
        # 3. Visualize a sample model
        # print("\nGenerating visualization...")
        # visualize_reconstructions(output_data_file, flat_model_paths, num_to_visualize=1)