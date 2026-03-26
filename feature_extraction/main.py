import os
import json
import glob
import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import trimesh
import pyshtools as pysh
from fpdf import FPDF
from pathlib import Path
from tqdm import tqdm
from collections import Counter
from scipy.special import lpmv, gamma
from scipy.spatial import cKDTree
from scipy.cluster.hierarchy import dendrogram, linkage
from sklearn.preprocessing import StandardScaler
from sklearn.decomposition import PCA
from sklearn.cluster import KMeans
from sklearn.metrics.pairwise import cosine_similarity

# =====================================================================
# CONFIGURATION GLOBALE DES CHEMINS ET PARAMÈTRES
# =====================================================================

# --- Inputs ---
METADATA_JSON = "../NASA_metadata3.json" 
BASE_MESH_DIR = "../Nasa_obj/ANTARCTIC"
IMAGE_DIR     = "../Nasa_obj/ANTARCTIC/pics"

# --- Outputs ---
OUT_DIR = "./Pipeline_SH_Output"
RECON_FIBO_DIR = os.path.join(OUT_DIR, "Reconstructed_Fibonacci_ANTARCTIC")
RECON_PYSH_DIR = os.path.join(OUT_DIR, "Reconstructed_PySHTools_ANTARCTIC")

OUT_PDF_DATA      = os.path.join(OUT_DIR, "01_Rapport_Data_Analysis.pdf")
OUT_PDF_CATALOG   = os.path.join(OUT_DIR, "02_Catalogue_Visuel.pdf")
OUT_JSON_FIBO     = os.path.join(OUT_DIR, "03_Descriptors_Fibonacci.json")
OUT_JSON_PYSH     = os.path.join(OUT_DIR, "04_Descriptors_PySHTools.json")
OUT_JSON_PYSH_FULL= os.path.join(OUT_DIR, "04_FullData_PySHTools.json")
OUT_PLOT_HEATMAP  = os.path.join(OUT_DIR, "05_Heatmap_Similarite.png")
OUT_PLOT_DENDRO   = os.path.join(OUT_DIR, "06_Dendrogramme_Clustering.png")
OUT_PLOT_PCA      = os.path.join(OUT_DIR, "07_PCA_Clusters.png")

# --- Paramètres ---
SH_DEGREE_MAX = 20  
N_SAMPLES = 5000 
EVAL_SAMPLES = 50000

# =====================================================================
# FONCTIONS
# =====================================================================

def load_antarctic_metadata(filepath):
    with open(filepath, 'r', encoding='utf-8') as f:
        data = json.load(f)
    meta_list = data.get("ANTARCTIC", [])
    name_to_macro = {item["name"]: item.get("macro_category", "Unknown") for item in meta_list}
    return meta_list, name_to_macro

# =====================================================================
# MODULE 1 : Analyse des Métadonnées
# =====================================================================
class PDFDataReport(FPDF):
    def header(self):
        self.set_font("helvetica", "B", 18)
        self.set_text_color(11, 61, 145) 
        self.cell(0, 10, "Rapport d'Analyse des Métadonnées - Météorites", align="C", ln=True)
        self.ln(10)
    def section_title(self, title):
        self.set_font("helvetica", "B", 14)
        self.set_text_color(40, 40, 40)
        self.cell(0, 10, title, ln=True)
        self.set_draw_color(200, 200, 200)
        self.line(self.get_x(), self.get_y(), self.get_x() + 190, self.get_y())
        self.ln(5)
    def print_stat_row(self, label, value, is_bold_value=True):
        self.set_font("helvetica", "", 11)
        self.set_text_color(0, 0, 0)
        self.cell(140, 7, label, ln=0)
        if is_bold_value:
            self.set_font("helvetica", "B", 11)
            self.set_text_color(11, 61, 145)
        self.cell(0, 7, str(value), ln=True)

def run_data_analysis():
    with open(METADATA_JSON, 'r', encoding='utf-8') as f:
        data = json.load(f)
    apollo = data.get("APOLLO", [])
    antarctic = data.get("ANTARCTIC", [])

    origins = Counter([s.get("origin", "Inconnue") for s in apollo + antarctic])
    classifs = Counter([s.get("macro_category", "Inconnue") for s in apollo + antarctic])

    pdf = PDFDataReport()
    pdf.add_page()
    pdf.section_title("1. Vue d'Ensemble des Échantillons")
    pdf.print_stat_row("Échantillons Apollo :", f"{len(apollo)} modèles")
    pdf.print_stat_row("Échantillons Antarctique :", f"{len(antarctic)} modèles")
    
    pdf.ln(5)
    pdf.section_title(f"2. Répartition par Origine")
    for o, c in origins.most_common():
        pdf.print_stat_row(f"    - {o}", f"{c} modèles", False)
        
    pdf.ln(5)
    pdf.section_title(f"3. Répartition par Macro-Catégorie")
    for cls, c in classifs.most_common():
        pdf.print_stat_row(f"    - {cls}", f"{c} modèles", False)

    pdf.output(OUT_PDF_DATA)

# =====================================================================
# MODULE 2 : Catalague de données
# =====================================================================
class VisualReportPDF(FPDF):
    def header(self):
        self.set_font("helvetica", "B", 14)
        self.set_text_color(11, 61, 145)
        self.cell(0, 10, "Catalogue Visuel et Structurel (Antarctique)", align="C", ln=True)
        self.set_draw_color(200, 200, 200)
        self.line(10, 20, 200, 200)
        self.ln(5)
    def print_metadata(self, label, value):
        self.set_font("helvetica", "B", 11)
        self.set_text_color(40, 40, 40)
        self.cell(45, 8, label, ln=0)
        self.set_font("helvetica", "", 11)
        self.set_text_color(0, 0, 0)
        self.multi_cell(0, 8, value)

def run_visual_catalog():
    meta_list, _ = load_antarctic_metadata(METADATA_JSON)
    meta_map = {item["name"]: item for item in meta_list}
    
    image_paths = []
    for ext in ('*.png', '*.jpg', '*.jpeg'):
        image_paths.extend(glob.glob(os.path.join(IMAGE_DIR, ext)))

    if not image_paths:
        print("  -> Aucune image trouvée, passage de l'étape.")
        return

    pdf = VisualReportPDF()
    for img_path in image_paths:
        base_name = os.path.splitext(os.path.basename(img_path))[0]
        info = meta_map.get(base_name)
        pdf.add_page()
        
        if info:
            pdf.print_metadata("Nom du Modèle :", info.get("name", "Inconnu"))
            pdf.print_metadata("Origine :", info.get("origin", "Inconnue"))
            pdf.print_metadata("Macro-Catégorie :", info.get("macro_category", "Inconnue"))
        else:
            pdf.print_metadata("Fichier :", base_name)
            
        pdf.ln(10)
        try:
            pdf.image(img_path, x=(210 - 150) / 2, y=pdf.get_y(), w=150)
        except:
            pass
    pdf.output(OUT_PDF_CATALOG)

# =====================================================================
# MODULE 3 : NOTRE METHODE
# =====================================================================
def K_factor(l, m):
    num = (2.0 * l + 1.0)
    den = 4.0 * np.pi
    fact = gamma(l - abs(m) + 1) / gamma(l + abs(m) + 1)
    return np.sqrt((num / den) * fact)

def eval_Y(l, m, theta, phi):
    p = lpmv(abs(m), l, np.cos(theta))
    if m > 0: return np.sqrt(2.0) * K_factor(l, m) * np.cos(m * phi) * p
    elif m < 0: return np.sqrt(2.0) * K_factor(l, -m) * np.sin(-m * phi) * p
    else: return K_factor(l, 0) * p

def save_recon_fibonacci(coeffs, filename):
    n_lat, n_lon = 100, 200
    theta_grid, phi_grid = np.meshgrid(np.pi * np.arange(n_lat + 1) / n_lat, 
                                       2 * np.pi * np.arange(n_lon + 1) / n_lon, indexing='ij')
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
        for v in verts: f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
        for i in range(n_lat):
            for j in range(n_lon):
                p1 = i * (n_lon + 1) + j + 1
                p2, p3 = p1 + 1, (i + 1) * (n_lon + 1) + j + 2
                f.write(f"f {p1} {p2} {p3} {p3-1}\n")

def run_fibonacci_sh():
    files = glob.glob(os.path.join(BASE_MESH_DIR, "*.obj"))
    results = {"ANTARCTIC": {}}
    
    for filepath in tqdm(files, desc="Fibonacci SH"):
        name = os.path.splitext(os.path.basename(filepath))[0]
        mesh = trimesh.load(filepath, force='mesh')
        
        if len(mesh.vertices) == 0: continue
        mesh.vertices -= np.mean(mesh.vertices, axis=0)
        max_r = np.max(np.linalg.norm(mesh.vertices, axis=1))
        if max_r > 0: mesh.vertices /= max_r

        indices = np.arange(0, N_SAMPLES, dtype=float)
        phi = (2 * np.pi * (indices / ((1.0 + np.sqrt(5.0)) / 2.0))) % (2 * np.pi)
        z = 1 - (2.0 * (indices + 0.5) / N_SAMPLES)
        theta, rad = np.arccos(z), np.sqrt(1 - z * z)

        ray_dirs = np.stack([rad * np.cos(phi), rad * np.sin(phi), z], axis=1)
        locs, index_ray, _ = mesh.ray.intersects_location(np.zeros_like(ray_dirs), ray_dirs, multiple_hits=False)

        r_vals = np.zeros(N_SAMPLES)
        if len(locs) > 0: r_vals[index_ray] = np.linalg.norm(locs, axis=1)

        coeffs, descriptor = [], []
        weight = 4.0 * np.pi / N_SAMPLES

        for l in range(SH_DEGREE_MAX + 1):
            coeffs_l = [np.sum(r_vals * eval_Y(l, m, theta, phi)) * weight for m in range(-l, l + 1)]
            coeffs.append(coeffs_l)
            descriptor.append(float(np.sqrt(np.sum(np.array(coeffs_l)**2))))
            
        save_recon_fibonacci(coeffs, os.path.join(RECON_FIBO_DIR, f"{name}.obj"))
        results["ANTARCTIC"][name] = descriptor

    with open(OUT_JSON_FIBO, 'w') as f: json.dump(results, f, indent=2)

# =====================================================================
# MODULE 4 : PYSHTOOLS
# =====================================================================
def run_pyshtools_sh():
    files = glob.glob(os.path.join(BASE_MESH_DIR, "*.obj"))
    results_desc, results_full = {"ANTARCTIC": {}}, {"ANTARCTIC": {}}
    
    for path in tqdm(files, desc="PySHTools"):
        name = os.path.splitext(os.path.basename(path))[0]
        mesh = trimesh.load(path, force='mesh')
        mesh.apply_translation(-mesh.centroid)
        mesh.apply_scale(1.0 / mesh.scale)

        nlat, nlon = 2 * SH_DEGREE_MAX + 2, 4 * SH_DEGREE_MAX + 4
        lats, lons = np.linspace(90, -90, nlat), np.linspace(0, 360, nlon, endpoint=False)
        lon_grid, lat_grid = np.meshgrid(lons, lats)
        
        theta, phi = np.radians(90 - lat_grid), np.radians(lon_grid)
        ray_dirs = np.stack((np.sin(theta)*np.cos(phi), np.sin(theta)*np.sin(phi), np.cos(theta)), axis=-1).reshape(-1, 3)
        
        locs, index_ray, _ = mesh.ray.intersects_location(np.zeros_like(ray_dirs), ray_dirs, multiple_hits=False)
        radii_flat = np.zeros(len(ray_dirs))
        if len(locs) > 0:
            radii_flat[index_ray] = np.linalg.norm(locs, axis=1)
        
        grid = pysh.SHGrid.from_array(radii_flat.reshape(nlat, nlon))
        clm = grid.expand()
        
        results_desc["ANTARCTIC"][name] = clm.spectrum().tolist()
        results_full["ANTARCTIC"][name] = {"spectrum": clm.spectrum().tolist(), "coefficients": clm.to_array().tolist()}
        
        grid_recon = clm.expand()
        radii_recon = grid_recon.data
        
        lats_recon = grid_recon.lats()
        lons_recon = grid_recon.lons()
        nlat_recon, nlon_recon = radii_recon.shape
        
        lon_grid_recon, lat_grid_recon = np.meshgrid(lons_recon, lats_recon)
        theta_recon = np.radians(90 - lat_grid_recon)
        phi_recon = np.radians(lon_grid_recon)
        
        x = radii_recon * np.sin(theta_recon) * np.cos(phi_recon)
        y = radii_recon * np.sin(theta_recon) * np.sin(phi_recon)
        z = radii_recon * np.cos(theta_recon)
        vertices = np.stack([x.flatten(), y.flatten(), z.flatten()], axis=1)
        
        faces = []
        for i in range(nlat_recon - 1):
            for j in range(nlon_recon):
                p1 = i * nlon_recon + j
                p2 = i * nlon_recon + ((j + 1) % nlon_recon)
                p3 = (i + 1) * nlon_recon + j
                p4 = (i + 1) * nlon_recon + ((j + 1) % nlon_recon)
                faces.extend([[p1, p2, p3], [p2, p4, p3]])
                
        recon_mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=True)
        recon_mesh.export(os.path.join(RECON_PYSH_DIR, f"{name}.obj"))

    with open(OUT_JSON_PYSH, 'w') as f: json.dump(results_desc, f, indent=2)
    with open(OUT_JSON_PYSH_FULL, 'w') as f: json.dump(results_full, f, indent=2)


# =====================================================================
# MODULE 5 : EVALUATION
# =====================================================================
def run_evaluation():
    def compute_errors(m_orig, m_recon):
        pA, _ = trimesh.sample.sample_surface(m_orig, EVAL_SAMPLES)
        pB, _ = trimesh.sample.sample_surface(m_recon, EVAL_SAMPLES)
        tA, tB = cKDTree(pA), cKDTree(pB)
        dA, _ = tB.query(pA, k=1)
        dB, _ = tA.query(pB, k=1)
        return float((np.mean(dA) + np.mean(dB)) / 2.0), float(max(np.max(dA), np.max(dB)))

    orig_files = glob.glob(os.path.join(BASE_MESH_DIR, "*.obj"))
    for method_name, recon_dir in [("FIBONACCI", RECON_FIBO_DIR), ("PYSHTOOLS", RECON_PYSH_DIR)]:
        print(f"\n--- Évaluation de {method_name} ---")
        m_dists, h_dists = [], []
        
        for orig in orig_files:
            name = os.path.basename(orig)
            recon_path = os.path.join(recon_dir, name)
            if not os.path.exists(recon_path): continue
            
            orig_m = trimesh.load(orig, force='mesh')
            recon_m = trimesh.load(recon_path, force='mesh')
            
            orig_m.apply_translation(-orig_m.centroid)
            orig_m.apply_scale(1.0 / orig_m.scale)
            recon_m.apply_translation(-recon_m.centroid)
            recon_m.apply_scale(1.0 / recon_m.scale)
            
            m_err, h_err = compute_errors(orig_m, recon_m)
            m_dists.append(m_err); h_dists.append(h_err)
            
        if m_dists:
            print(f" > Erreur Moyenne Globale   : {np.mean(m_dists):.6f}")
            print(f" > Erreur Hausdorff Globale : {np.mean(h_dists):.6f}")

# =====================================================================
# MODULE 6 : HEATMAP
# =====================================================================
def run_heatmap():
    _, name_to_macro = load_antarctic_metadata(METADATA_JSON)
    with open(OUT_JSON_FIBO, 'r') as f:
        data = json.load(f).get("ANTARCTIC", {})
        
    if not data: return
    names, features = list(data.keys()), np.array(list(data.values()))
    
    features_scaled = StandardScaler().fit_transform(features[:, 1:])
    df = pd.DataFrame(cosine_similarity(features_scaled), index=names, columns=names)
    
    macros = [name_to_macro.get(n, "Unknown") for n in names]
    df['Macro_Category'] = macros
    df = df.sort_values(by='Macro_Category')
    df = df[df.index.tolist()] 
    
    display_labels = [f"{n[:8]}... ({name_to_macro.get(n, '?')})" for n in df.index]
    
    plt.figure(figsize=(14, 12))
    sns.heatmap(df, cmap="magma", xticklabels=display_labels, yticklabels=display_labels, linewidths=0.5, linecolor='black')
    plt.title("Matrice de Similarité Cosinus (Méthode Fibonacci)", fontsize=16, pad=20)
    plt.tight_layout()
    plt.savefig(OUT_PLOT_HEATMAP, dpi=300)
    plt.close()

# =====================================================================
# MODULE 7 : CLUSTERING
# =====================================================================
def run_clustering():
    _, name_to_macro = load_antarctic_metadata(METADATA_JSON)
    with open(OUT_JSON_FIBO, 'r') as f:
        data = json.load(f).get("ANTARCTIC", {})
        
    if not data: return
    names, features = list(data.keys()), np.array(list(data.values()))
    features_scaled = StandardScaler().fit_transform(features[:, 1:]) # Ignore L=0
    n_clusters = len(set(name_to_macro.values()))
    display_names = [f"{n[:8]} ({name_to_macro.get(n, '?')})" for n in names]

    plt.figure(figsize=(14, 7))
    dendrogram(linkage(features_scaled, method='ward'), labels=display_names, leaf_rotation=90., leaf_font_size=8.)
    plt.title("Arbre Hiérarchique des Formes Antarctiques")
    plt.tight_layout()
    plt.savefig(OUT_PLOT_DENDRO)
    plt.close()

    kmeans = KMeans(n_clusters=n_clusters, random_state=42, n_init='auto')
    labels = kmeans.fit_predict(features_scaled)
    pca = PCA(n_components=2)
    f2d = pca.fit_transform(features_scaled)

    plt.figure(figsize=(12, 9))
    scatter = plt.scatter(f2d[:, 0], f2d[:, 1], c=labels, cmap='tab10', s=100, alpha=0.8, edgecolors='w')
    for i, name in enumerate(names):
        plt.annotate(display_names[i], (f2d[i, 0], f2d[i, 1]), xytext=(5, 5), textcoords='offset points', fontsize=8)
        
    plt.title(f"Clustering K-Means (K={n_clusters})")
    plt.xlabel(f"Composante Principale 1 ({pca.explained_variance_ratio_[0]*100:.1f}%)")
    plt.ylabel(f"Composante Principale 2 ({pca.explained_variance_ratio_[1]*100:.1f}%)")
    plt.legend(*scatter.legend_elements(), title="Clusters")
    plt.tight_layout()
    plt.savefig(OUT_PLOT_PCA)
    plt.close()


if __name__ == "__main__":
    print("=====================================================================")
    print("      PIPELINE GLOBAL - ANALYSE GÉOMÉTRIQUE DE MÉTÉORITES")
    print("=====================================================================\n")

    os.makedirs(OUT_DIR, exist_ok=True)
    os.makedirs(RECON_FIBO_DIR, exist_ok=True)
    os.makedirs(RECON_PYSH_DIR, exist_ok=True)

    print("[ÉTAPE 1/7] Analyse des Métadonnées PDF...")
    run_data_analysis()

    print("\n[ÉTAPE 2/7] Création du Catalogue Visuel PDF...")
    run_visual_catalog()

    print("\n[ÉTAPE 3/7] Extraction Harmoniques Sphériques (Méthode Fibonacci)...")
    run_fibonacci_sh()

    print("\n[ÉTAPE 4/7] Extraction Harmoniques Sphériques (Baseline PySHTools)...")
    run_pyshtools_sh()

    print("\n[ÉTAPE 5/7] Évaluation des Erreurs de Reconstruction...")
    run_evaluation()

    print("\n[ÉTAPE 6/7] Génération de la Heatmap de Similarité...")
    run_heatmap()

    print("\n[ÉTAPE 7/7] Apprentissage Non-Supervisé (Clustering PCA & K-Means)...")
    run_clustering()

    print("\n=====================================================================")
    print(" PIPELINE TERMINÉ AVEC SUCCÈS ! ")
    print(f" Tous les résultats sont proprement sauvegardés dans : {OUT_DIR}/")
    print("=====================================================================")