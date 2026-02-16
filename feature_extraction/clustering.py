import json
import numpy as np
import matplotlib.pyplot as plt
from sklearn.preprocessing import StandardScaler
from sklearn.decomposition import PCA
from sklearn.cluster import KMeans
from scipy.cluster.hierarchy import dendrogram, linkage

# --- CONFIGURATION ---
INPUT_JSON = "descriptors.json"
N_CLUSTERS = 3  

def load_data(filepath):
    """Loads the SH descriptors from JSON."""
    with open(filepath, 'r') as f:
        data = json.load(f)
    
    names = list(data.keys())
    features = np.array(list(data.values()))
    return names, features

def plot_dendrogram(features, names):
    """Creates a family tree (dendrogram) of meteorite shapes."""
    plt.figure(figsize=(12, 6))
    plt.title("Meteorite Shape Family Tree (Hierarchical Clustering)")
    
    Z = linkage(features, method='ward')
    
    dendrogram(
        Z,
        labels=names,
        leaf_rotation=90.,
        leaf_font_size=10.,
        color_threshold=np.max(Z[:, 2]) * 0.7 
    )
    
    plt.ylabel("Shape Distance (Dissimilarity)")
    plt.tight_layout()
    plt.savefig("meteorite_dendrogram.png")
    plt.show()

def cluster_and_plot_pca(features, names, n_clusters):
    """Clusters data using K-Means and visualizes it in 2D using PCA."""
    # K-Means Clustering
    kmeans = KMeans(n_clusters=n_clusters, random_state=42, n_init='auto')
    labels = kmeans.fit_predict(features)
    
    # PCA
    pca = PCA(n_components=2)
    features_2d = pca.fit_transform(features)
    
    variance_explained = pca.explained_variance_ratio_
    print(f"PCA captured {variance_explained[0]*100:.1f}% of variance on X-axis")
    print(f"PCA captured {variance_explained[1]*100:.1f}% of variance on Y-axis")

    # 3. Plotting
    plt.figure(figsize=(10, 8))
    
    # Scatter plot, colored by K-Means cluster
    scatter = plt.scatter(
        features_2d[:, 0], features_2d[:, 1], 
        c=labels, cmap='viridis', s=100, alpha=0.8, edgecolors='w'
    )
    
    for i, name in enumerate(names):
        plt.annotate(
            name, 
            (features_2d[i, 0], features_2d[i, 1]),
            xytext=(5, 5), textcoords='offset points', fontsize=8, alpha=0.7
        )
        
    plt.title(f"Meteorite Shape Clusters (PCA reduced to 2D)")
    plt.xlabel(f"Principal Component 1 ({variance_explained[0]*100:.1f}%) - e.g. Elongation")
    plt.ylabel(f"Principal Component 2 ({variance_explained[1]*100:.1f}%) - e.g. Surface Roughness")
    
    legend1 = plt.legend(*scatter.legend_elements(), title="Clusters")
    plt.gca().add_artist(legend1)
    
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.tight_layout()
    plt.savefig("meteorite_pca_clusters.png")
    plt.show()

    print("\n--- Final Cluster Groupings ---")
    for i in range(n_clusters):
        cluster_members = [names[j] for j in range(len(names)) if labels[j] == i]
        print(f"\nCluster {i} (Count: {len(cluster_members)}):")
        print(", ".join(cluster_members))

def main():
    print(f"Loading data from {INPUT_JSON}...")
    names, features = load_data(INPUT_JSON)
    
    if len(names) == 0:
        print("Error: No data found in JSON.")
        return

    scaler = StandardScaler()
    features_scaled = scaler.fit_transform(features)

    print("\nGenerating Dendrogram...")
    plot_dendrogram(features_scaled, names)
    
    print(f"\nRunning K-Means (K={N_CLUSTERS}) and PCA...")
    cluster_and_plot_pca(features_scaled, names, N_CLUSTERS)

if __name__ == "__main__":
    main()