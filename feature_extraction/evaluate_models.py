import numpy as np
import trimesh
from pathlib import Path
from scipy.spatial import cKDTree

def compute_mesh_errors(mesh_A, mesh_B, num_samples=100000):
    """
    Computes fast approximations of both the Hausdorff distance 
    and the Symmetric Mean distance using KD-Trees.
    """
    points_A, _ = trimesh.sample.sample_surface(mesh_A, num_samples)
    points_B, _ = trimesh.sample.sample_surface(mesh_B, num_samples)
    
    tree_A = cKDTree(points_A)
    tree_B = cKDTree(points_B)
    
    distances_A_to_B, _ = tree_B.query(points_A, k=1)
    max_A_to_B = np.max(distances_A_to_B)
    mean_A_to_B = np.mean(distances_A_to_B)
    
    distances_B_to_A, _ = tree_A.query(points_B, k=1)
    max_B_to_A = np.max(distances_B_to_A)
    mean_B_to_A = np.mean(distances_B_to_A)
    
    hausdorff_dist = float(max(max_A_to_B, max_B_to_A))
    mean_dist = float((mean_A_to_B + mean_B_to_A) / 2.0)
    
    return mean_dist, hausdorff_dist

def evaluate_folder_errors(orig_folder, recon_folder, num_samples=50000):
    """
    Reads original and reconstructed models from folders, matches them, 
    and calculates both Mean and Hausdorff distances.
    """
    orig_path = Path(orig_folder)
    recon_path = Path(recon_folder)
    
    valid_extensions = {'.obj', '.stl', '.ply', '.off'}
    
    orig_files = [f for f in orig_path.iterdir() if f.is_file() and f.suffix.lower() in valid_extensions]
    
    if not orig_files:
        print(f"No original models found in {orig_folder}")
        return

    mean_distances = []
    hausdorff_distances = []
    failed_models = []

    print(f"{'Model Name':<25} | {'Mean Distance':<17} | {'Hausdorff Distance'}")
    print("-" * 67)

    for orig_file in orig_files:
        model_name = orig_file.stem
        
        matching_recon_files = [f for f in recon_path.iterdir() if f.stem == model_name and f.suffix.lower() in valid_extensions]
        
        if not matching_recon_files:
            failed_models.append((model_name, "No matching reconstructed file found."))
            continue
            
        recon_file = matching_recon_files[0]

        try:
            orig_mesh = trimesh.load(str(orig_file), force='mesh')
            recon_mesh = trimesh.load(str(recon_file), force='mesh')
            
            orig_mesh.apply_translation(-orig_mesh.centroid)
            orig_mesh.apply_scale(1.0 / orig_mesh.scale)

            recon_mesh.apply_translation(-recon_mesh.centroid)
            recon_mesh.apply_scale(1.0 / recon_mesh.scale)
            
            m_dist, h_dist = compute_mesh_errors(orig_mesh, recon_mesh, num_samples)
            
            mean_distances.append(m_dist)
            hausdorff_distances.append(h_dist)
            
            print(f"{model_name:<25} | {m_dist:<17.6f} | {h_dist:.6f}")
            
        except Exception as e:
            failed_models.append((model_name, str(e)))

    print("-" * 67)
    if hausdorff_distances and mean_distances:
        global_mean_dist = np.mean(mean_distances)
        global_hausdorff_dist = np.mean(hausdorff_distances)
        max_hausdorff_overall = np.max(hausdorff_distances)
        
        print(f"{'GLOBAL AVG MEAN ERROR:':<25} | {global_mean_dist:<17.6f} |")
        print(f"{'GLOBAL AVG HAUSDORFF:':<25} | {'':<17} | {global_hausdorff_dist:.6f}")
        print(f"{'PEAK HAUSDORFF (WORST):':<25} | {'':<17} | {max_hausdorff_overall:.6f}")
    
    if failed_models:
        print(f"\nFailed to process {len(failed_models)} models:")
        for name, err in failed_models:
            print(f"  - {name}: {err}")

if __name__ == "__main__":
    original_models_folder = "../Nasa_obj"
    first_reconstructed_models_folder = "../reconstructed_debug" 
    second_reconstructed_models_folder = "../Nasa_reconstructed" 
    print("Evaluating first set of reconstructions (our method)...")
    evaluate_folder_errors(original_models_folder, first_reconstructed_models_folder, num_samples=100000)
    
    print("Evaluating second set of reconstructions (library)...")
    evaluate_folder_errors(original_models_folder, second_reconstructed_models_folder, num_samples=100000)