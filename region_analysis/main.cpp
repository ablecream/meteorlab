#include <pcl/io/ply_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>

#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>
#include <unordered_map>
#include <vector>
#include <numeric>
#include <Eigen/Dense>
#include <unordered_set>

namespace fs = boost::filesystem;

struct RegionStats {
    int id;
    int face_count = 0;
    double area = 0.0;
    
    // Métriques géométriques
    double mean_curvature = 0.0;
    double depth_approx = 0.0;     // Profondeur estimée (axe mineur PCA)
    double elongation = 0.0;       // Rapport Longueur/Largeur
    double planarity = 0.0;        // À quel point c'est plat (0 à 1)
    
    Eigen::Vector3f direction;     // Vecteur normal moyen (orientation du flux)
};

// Calcul de l'aire d'un triangle 3D
double triangleArea(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c) {
    return 0.5 * ((b - a).cross(c - a)).norm();
}

RegionStats analyzeRegion(int region_id,
                          const pcl::PolygonMesh& mesh, 
                          const std::vector<int>& face_indices,
                          const pcl::PointCloud<pcl::PointXYZ>::Ptr& full_cloud,
                          const pcl::PointCloud<pcl::Normal>::Ptr& full_normals) {
    
    RegionStats stats;
    stats.id = region_id;
    stats.face_count = face_indices.size();

    // 1. Extraction des points de la région et calcul de l'aire
    pcl::PointCloud<pcl::PointXYZ>::Ptr region_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    std::unordered_set<int> unique_indices;
    std::vector<int> region_point_indices; // Pour retrouver les normales correspondantes

    double total_area = 0.0;
    Eigen::Vector3f avg_normal(0, 0, 0);

    for (int idx : face_indices) {
        const auto& poly = mesh.polygons[idx];
        if (poly.vertices.size() < 3) continue;

        // Récupération des sommets
        const Eigen::Vector3f& v0 = full_cloud->points[poly.vertices[0]].getVector3fMap();
        const Eigen::Vector3f& v1 = full_cloud->points[poly.vertices[1]].getVector3fMap();
        const Eigen::Vector3f& v2 = full_cloud->points[poly.vertices[2]].getVector3fMap();

        total_area += triangleArea(v0, v1, v2);

        // Accumulation des points uniques pour la PCA
        for (auto vi : poly.vertices) {
            if (unique_indices.find(vi) == unique_indices.end()) {
                unique_indices.insert(vi);
                region_cloud->push_back(full_cloud->points[vi]);
                region_point_indices.push_back(vi);
                
                // On somme les courbures pré-calculées (Correction du problème de bord)
                stats.mean_curvature += full_normals->points[vi].curvature;
                avg_normal += full_normals->points[vi].getNormalVector3fMap();
            }
        }
    }
    
    stats.area = total_area;
    if (!region_point_indices.empty()) {
        stats.mean_curvature /= region_point_indices.size();
        stats.direction = avg_normal.normalized();
    }

    // 2. Analyse PCA (Pour remplacer la BBox Volume incorrecte)
    if (region_cloud->size() > 3) {
        pcl::PCA<pcl::PointXYZ> pca;
        pca.setInputCloud(region_cloud);
        
        // Valeurs propres (eigenvalues) : λ1 >= λ2 >= λ3 representing variance along axes
        Eigen::Vector3f eigen_values = pca.getEigenValues();
        
        // L'axe principal (le plus long)
        // Eigen::Matrix3f eigen_vectors = pca.getEigenVectors(); 

        // Métriques physiques basées sur la PCA
        // λ1 = Longueur², λ2 = Largeur², λ3 = Profondeur² (approx)
        
        float L1 = sqrt(eigen_values[0]); // Plus grande dimension
        float L2 = sqrt(eigen_values[1]);
        float L3 = sqrt(eigen_values[2]); // Plus petite dimension (épaisseur/profondeur)

        stats.depth_approx = L3 * 2.0; // Approximation de l'épaisseur locale
        stats.elongation = (L2 > 0) ? L1 / L2 : 0;
        stats.planarity = (L1 + L2 > 0) ? (L1 - L2) / L1 : 0; // Simplifié
    }

    return stats;
}

std::unordered_map<int, std::vector<int>> loadLabels(const std::string& path) {
    std::unordered_map<int, std::vector<int>> region_faces;
    std::ifstream in(path);
    if (!in) throw std::runtime_error("Can't open label file: " + path);
    int face_id, label;
    while (in >> face_id >> label)
        region_faces[label].push_back(face_id);
    return region_faces;
}

void processModel(const fs::path& dir) {
    fs::path model_path = dir / "model.ply"; // ou output.ply selon ton script précédent
    fs::path labels_path = dir / "face_labels.txt";
    
    if (!fs::exists(model_path) || !fs::exists(labels_path)) {
        std::cout << "[SKIP] Missing files in " << dir << std::endl;
        return;
    }

    pcl::PolygonMesh mesh;
    pcl::io::loadPLYFile(model_path.string(), mesh);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

    // --- CORRECTION CRITIQUE : Calcul Global des Normales ---
    // On calcule les normales sur TOUT l'objet avant de découper
    std::cout << "Computing global normals for context..." << std::endl;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr global_normals(new pcl::PointCloud<pcl::Normal>());
    ne.setKSearch(20); // 20 voisins pour une bonne estimation locale
    ne.compute(*global_normals);

    auto region_faces = loadLabels(labels_path.string());
    
    std::ofstream out(dir.string() + "/meteorite_analysis.csv");
    out << "RegionID,Area_mm2,Depth_Approx_mm,MeanCurvature,Elongation,DirX,DirY,DirZ\n";

    for (const auto& [label, faces] : region_faces) {
        RegionStats stats = analyzeRegion(label, mesh, faces, cloud, global_normals);
        
        out << label << "," 
            << stats.area << ","
            << stats.depth_approx << ","
            << stats.mean_curvature << ","
            << stats.elongation << ","
            << stats.direction.x() << ","
            << stats.direction.y() << ","
            << stats.direction.z() << "\n";
    }

    out.close();
    std::cout << "Analysis saved to: " << dir.string() + "/meteorite_analysis.csv" << "\n";
}

int main(int argc, char** argv) {
    // Usage unique sur un dossier ou itération, selon tes besoins
    if (argc < 2) { 
        std::cerr << "Usage: ./analyzer <directory_containing_ply_and_labels>\n";
        return 1;
    }
    processModel(argv[1]);
    return 0;
}