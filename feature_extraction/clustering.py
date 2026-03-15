import json
import numpy as np
import trimesh
import os
import matplotlib.pyplot as plt
from pathlib import Path
from collections import Counter
from sklearn.preprocessing import StandardScaler
from sklearn.decomposition import PCA
from sklearn.cluster import KMeans
from scipy.cluster.hierarchy import dendrogram, linkage

# --- CONFIGURATION ---
DESCRIPTORS_JSON = "descriptors.json"
METADATA_JSON = "../NASA_metadata.json"
MESH_FOLDER = "../Nasa_obj"

def load_metadata(filepath):
    """Charge les métadonnées pour extraire le nombre de clusters et les origines."""
    with open(filepath, 'r') as f:
        meta = json.load(f)
        
    n_clusters = meta["metadata"]["number_of_origins"]
    
    # Création d'un dictionnaire { "nom_du_modele" : "Origine" }
    name_to_origin = {}
    for key, value in meta.items():
        if key == "metadata":
            continue
        for item in value:
            name_to_origin[item["name"]] = item["origin"]
            
    return n_clusters, name_to_origin

def load_descriptors(filepath):
    """Charge les descripteurs, compatible avec une structure JSON plate ou imbriquée."""
    with open(filepath, 'r') as f:
        data = json.load(f)
    
    names = []
    features = []
    
    for key, value in data.items():
        if isinstance(value, dict):
            # Structure imbriquée (ex: "APOLLO": {"nom": [...]})
            for sub_name, sub_feat in value.items():
                names.append(sub_name)
                features.append(sub_feat)
        else:
            # Structure plate
            names.append(key)
            features.append(value)
            
    return names, np.array(features)

def plot_dendrogram(features, display_names):
    """Crée un dendrogramme en utilisant les noms avec leurs origines."""
    plt.figure(figsize=(14, 7))
    plt.title("Meteorite Shape Family Tree (Hierarchical Clustering)")
    
    Z = linkage(features, method='ward')
    
    dendrogram(
        Z,
        labels=display_names,
        leaf_rotation=90.,
        leaf_font_size=8.,
        color_threshold=np.max(Z[:, 2]) * 0.7 
    )
    
    plt.ylabel("Shape Distance (Dissimilarity)")
    plt.tight_layout()
    plt.savefig("meteorite_dendrogram.png")
    plt.show()

def cluster_and_plot_pca(features, names, name_to_origin, n_clusters):
    """Clusterise les données et affiche les résultats de la PCA avec les origines."""
    # K-Means Clustering
    kmeans = KMeans(n_clusters=n_clusters, random_state=42, n_init='auto')
    labels = kmeans.fit_predict(features)
    
    # PCA pour la réduction à 2D
    pca = PCA(n_components=2)
    features_2d = pca.fit_transform(features)
    
    variance_explained = pca.explained_variance_ratio_
    print(f"PCA a capturé {variance_explained[0]*100:.1f}% de la variance sur l'axe X")
    print(f"PCA a capturé {variance_explained[1]*100:.1f}% de la variance sur l'axe Y")

    # Affichage graphique
    plt.figure(figsize=(12, 9))
    scatter = plt.scatter(
        features_2d[:, 0], features_2d[:, 1], 
        c=labels, cmap='tab10', s=100, alpha=0.8, edgecolors='w'
    )
    
    # Ajout des annotations (Nom + Origine)
    for i, name in enumerate(names):
        origin = name_to_origin.get(name, "Unknown")
        label_text = f"{name[:10]}... ({origin})" # On tronque le nom pour la lisibilité
        plt.annotate(
            label_text, 
            (features_2d[i, 0], features_2d[i, 1]),
            xytext=(5, 5), textcoords='offset points', fontsize=8, alpha=0.8
        )
        
    plt.title(f"Meteorite Shape Clusters (K-Means, K={n_clusters})")
    plt.xlabel(f"Principal Component 1 ({variance_explained[0]*100:.1f}%)")
    plt.ylabel(f"Principal Component 2 ({variance_explained[1]*100:.1f}%)")
    
    legend1 = plt.legend(*scatter.legend_elements(), title="Clusters")
    plt.gca().add_artist(legend1)
    
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.tight_layout()
    plt.savefig("meteorite_pca_clusters.png")
    plt.show()

    # --- ÉVALUATION DES CLUSTERS DANS LE TERMINAL ---
    print("\n--- Analyse des Clusters par Origine ---")
    print("Vérification de la séparation géométrique naturelle :")
    for i in range(n_clusters):
        # Trouver toutes les météorites dans ce cluster
        cluster_members = [names[j] for j in range(len(names)) if labels[j] == i]
        # Trouver leurs origines réelles
        origins_in_cluster = [name_to_origin.get(n, 'Unknown') for n in cluster_members]
        
        # Compter les occurrences de chaque origine
        origin_counts = Counter(origins_in_cluster)
        
        print(f"\nCluster {i} (Total: {len(cluster_members)} objets):")
        for origin, count in origin_counts.items():
            percentage = (count / len(cluster_members)) * 100
            print(f"  - {origin}: {count} ({percentage:.1f}%)")

    return labels

def visualize_clusters_in_3d(mesh_folder, names, labels):
    print("\nBuilding 3D Scene. This might take a moment depending on mesh sizes...")
    scene = trimesh.Scene()
    
    # Génération dynamique de couleurs si on a beaucoup de clusters
    cluster_colors = plt.cm.get_cmap('tab10', max(labels) + 1)(np.arange(max(labels) + 1))
    cluster_colors = (cluster_colors[:, :3] * 255).astype(np.uint8) 
    
    cluster_counts = {}
    SPACING = 1.0 

    for name, cluster_id in zip(names, labels):
        # Recherche récursive du fichier .obj (utile s'ils sont dans des sous-dossiers)
        matches = list(Path(mesh_folder).rglob(f"{name}.obj"))
        
        if not matches:
            print(f"Warning: Could not find mesh for {name} in {mesh_folder} or its subfolders.")
            continue
            
        filepath = str(matches[0])
        mesh = trimesh.load(filepath, force='mesh')

        if hasattr(mesh.visual, 'to_color'):
            mesh.visual = mesh.visual.to_color()

        mesh.apply_scale(6.0)
        
        # Application de la couleur du cluster (Format RGBA)
        color = np.append(cluster_colors[cluster_id], 255)
        mesh.visual.face_colors = color

        count = cluster_counts.get(cluster_id, 0)
        
        x_offset = count * SPACING
        y_offset = cluster_id * SPACING
        z_offset = 0.0
        
        translation = trimesh.transformations.translation_matrix([x_offset, y_offset, z_offset])
        mesh.apply_transform(translation)
        
        scene.add_geometry(mesh)
        
        cluster_counts[cluster_id] = count + 1
        
    scene.show()

def main():
    if not os.path.exists(METADATA_JSON):
        print(f"Error: Fichier {METADATA_JSON} introuvable.")
        return
    if not os.path.exists(DESCRIPTORS_JSON):
        print(f"Error: Fichier {DESCRIPTORS_JSON} introuvable.")
        return

    print(f"Loading metadata from {METADATA_JSON}...")
    n_clusters, name_to_origin = load_metadata(METADATA_JSON)
    print(f" -> Nombre d'origines détecté : {n_clusters}")
    
    print(f"Loading descriptors from {DESCRIPTORS_JSON}...")
    names, features = load_descriptors(DESCRIPTORS_JSON)
    
    if len(names) == 0:
        print("Error: No data found in descriptors JSON.")
        return

    # Normalisation des données
    scaler = StandardScaler()
    features_scaled = scaler.fit_transform(features)

    # Création des noms d'affichage (Nom + Origine)
    display_names = [f"{n} ({name_to_origin.get(n, 'Unknown')})" for n in names]

    print("\nGenerating Dendrogram...")
    plot_dendrogram(features_scaled, display_names)
    
    print(f"\nRunning K-Means (K={n_clusters}) and PCA...")
    labels = cluster_and_plot_pca(features_scaled, names, name_to_origin, n_clusters)
    
    visualize_clusters_in_3d(MESH_FOLDER, names, labels)

if __name__ == "__main__":
    main()