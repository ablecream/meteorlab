import os
import glob
import json
import math
import numpy as np
import trimesh
from scipy.special import lpmv
from scipy.special import gamma

# --- CONFIGURATION ---
INPUT_FOLDER = "../Nasa_obj"
OUTPUT_JSON = "descriptors.json"
RECON_FOLDER = "../reconstructed_debug"
SH_DEGREE_MAX = 20  
N_SAMPLES = 5000    

def K_factor(l, m):
    """Normalization constant for Spherical Harmonics."""
    num = (2.0 * l + 1.0)
    den = 4.0 * np.pi
    fact = gamma(l - abs(m) + 1) / gamma(l + abs(m) + 1)
    return np.sqrt((num / den) * fact)

def eval_Y(l, m, theta, phi):
    """
    Evaluates Real Spherical Harmonics. 
    Vectorized to accept numpy arrays for theta and phi.
    """
    p = lpmv(abs(m), l, np.cos(theta))
    
    if m > 0:
        return np.sqrt(2.0) * K_factor(l, m) * np.cos(m * phi) * p
    elif m < 0:
        return np.sqrt(2.0) * K_factor(l, -m) * np.sin(-m * phi) * p
    else:
        return K_factor(l, 0) * p

def save_reconstruction(coeffs, filename):
    """Reconstructs the mesh from SH coefficients and saves it as an OBJ."""
    n_lat = 100
    n_lon = 200

    theta_indices = np.arange(n_lat + 1)
    phi_indices = np.arange(n_lon + 1)
    
    theta_mesh = np.pi * theta_indices / n_lat
    phi_mesh = 2 * np.pi * phi_indices / n_lon
    
    theta_grid, phi_grid = np.meshgrid(theta_mesh, phi_mesh, indexing='ij')
    
    r_grid = np.zeros_like(theta_grid)
    for l in range(len(coeffs)):
        for m_idx, m in enumerate(range(-l, l + 1)):
            if coeffs[l][m_idx] != 0:
                r_grid += coeffs[l][m_idx] * eval_Y(l, m, theta_grid, phi_grid)

    x = r_grid * np.sin(theta_grid) * np.cos(phi_grid)
    y = r_grid * np.sin(theta_grid) * np.sin(phi_grid)
    z = r_grid * np.cos(theta_grid)

    verts = np.stack([x.ravel(), y.ravel(), z.ravel()], axis=1)

    with open(filename, 'w') as f:
        for v in verts:
            f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")

        for i in range(n_lat):
            for j in range(n_lon):
                p1 = i * (n_lon + 1) + j + 1
                p2 = p1 + 1
                p3 = (i + 1) * (n_lon + 1) + j + 2
                p4 = p3 - 1
                f.write(f"f {p1} {p2} {p3} {p4}\n")

def process_mesh_and_reconstruct(filepath, name, out_recon_folder):
    """Loads mesh, normalizes, raycasts, computes SH, and returns descriptor."""
    try:
        mesh = trimesh.load(filepath, force='mesh')
    except Exception as e:
        print(f"Failed to load {filepath}: {e}")
        return []

    if len(mesh.vertices) == 0:
        return []

    # Normalization 
    centroid = np.mean(mesh.vertices, axis=0)
    mesh.vertices -= centroid
    
    max_r = np.max(np.linalg.norm(mesh.vertices, axis=1))
    if max_r > 0:
        mesh.vertices /= max_r

    # Fibonacci Sphere
    indices = np.arange(0, N_SAMPLES, dtype=float)
    golden_ratio = (1.0 + np.sqrt(5.0)) / 2.0
    
    phi = 2 * np.pi * (indices / golden_ratio)
    phi = phi % (2 * np.pi)
    z = 1 - (2.0 * (indices + 0.5) / N_SAMPLES)
    theta = np.arccos(z)
    rad = np.sqrt(1 - z * z)

    ray_dirs = np.stack([rad * np.cos(phi), rad * np.sin(phi), z], axis=1)
    ray_origins = np.zeros_like(ray_dirs)

    # Perform Raycasting
    locs, index_ray, _ = mesh.ray.intersects_location(
        ray_origins=ray_origins,
        ray_directions=ray_dirs,
        multiple_hits=False
    )

    r_vals = np.zeros(N_SAMPLES)
    if len(locs) > 0:
        distances = np.linalg.norm(locs, axis=1)
        r_vals[index_ray] = distances

    coeffs = []
    weight = 4.0 * np.pi / N_SAMPLES

    for l in range(SH_DEGREE_MAX + 1):
        coeffs_l = []
        for m in range(-l, l + 1):
            y_vals = eval_Y(l, m, theta, phi)
            c = np.sum(r_vals * y_vals) * weight
            coeffs_l.append(c)
        coeffs.append(coeffs_l)

    # Save to the specific subfolder passed into the function
    recon_path = os.path.join(out_recon_folder, f"{name}.obj")
    save_reconstruction(coeffs, recon_path)

    # Rotation-Invariant Descriptor 
    descriptor = []
    for l in range(SH_DEGREE_MAX + 1):
        energy = np.sum(np.array(coeffs[l])**2)
        descriptor.append(float(np.sqrt(energy)))
        
    return descriptor

def main():
    if not os.path.exists(INPUT_FOLDER):
        print(f"Error: Input folder '{INPUT_FOLDER}' not found.")
        return

    results = {}
    search_patterns = ["*.obj", "*.off", "*.ply", "*.stl"]

    # Identify all subfolders inside INPUT_FOLDER
    subfolders = [f for f in os.listdir(INPUT_FOLDER) if os.path.isdir(os.path.join(INPUT_FOLDER, f))]

    if not subfolders:
        print(f"No subfolders found inside '{INPUT_FOLDER}'. Please ensure 'ANTARCTIC', 'APOLLO', etc., exist.")
        return

    for subfolder in subfolders:
        print(f"\n--- Processing Subfolder: {subfolder} ---")
        
        # Define paths for this specific subfolder
        sub_input_dir = os.path.join(INPUT_FOLDER, subfolder)
        sub_recon_dir = os.path.join(RECON_FOLDER, subfolder)
        
        # Create matching output directory for reconstructions
        os.makedirs(sub_recon_dir, exist_ok=True)
        
        # Initialize the dictionary key for this subfolder
        results[subfolder] = {}

        # Grab all matching files
        files_to_process = []
        for pattern in search_patterns:
            files_to_process.extend(glob.glob(os.path.join(sub_input_dir, pattern)))

        if not files_to_process:
            print(f"No 3D models found in {subfolder}.")
            continue

        for filepath in files_to_process:
            name = os.path.splitext(os.path.basename(filepath))[0]
            print(f" > Extracting {name}... ", end="", flush=True)

            # Pass the subfolder path to the function so it knows where to save
            desc = process_mesh_and_reconstruct(filepath, name, sub_recon_dir)
            
            if desc:
                # Store the descriptor under the correct subfolder key
                results[subfolder][name] = desc
                print("Done.")

    # Export the final nested JSON
    with open(OUTPUT_JSON, 'w') as json_file:
        json.dump(results, json_file, indent=2)
        
    print(f"\nProcessing complete. Data saved to '{OUTPUT_JSON}'.")

if __name__ == "__main__":
    main()