#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/common.h>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <stack>
#include <queue>
#include <Eigen/Dense>
#include <unordered_set>
#include <algorithm>
#include <fstream>

using namespace pcl;
using namespace std;

// Calculate plane coefficients ax + by + cz + d = 0
Eigen::Vector4f fitPlaneLSQ(const vector<Eigen::Vector3f>& points) {
    if (points.size() < 3) return Eigen::Vector4f::Zero();

    Eigen::Vector3f centroid(0, 0, 0);
    for (const auto& p : points) centroid += p;
    centroid /= points.size();

    Eigen::MatrixXf centered(points.size(), 3);
    for (int i = 0; i < points.size(); ++i)
        centered.row(i) = points[i] - centroid;

    Eigen::Matrix3f cov = centered.transpose() * centered;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
    Eigen::Vector3f normal = solver.eigenvectors().col(0).normalized();

    float d = -normal.dot(centroid);
    return Eigen::Vector4f(normal.x(), normal.y(), normal.z(), d);
}

float angleBetweenNormals(const Eigen::Vector3f& n1, const Eigen::Vector3f& n2) {
    float d = n1.dot(n2);
    d = std::max(-1.0f, std::min(1.0f, d));
    return acos(d);
}

vector<vector<int>> buildFaceAdjacency(const vector<Vertices>& polygons) {
    unordered_map<string, vector<int>> edge_map;
    auto make_edge_key = [](int a, int b) {
        return to_string(min(a, b)) + "_" + to_string(max(a, b));
    };

    for (int i = 0; i < polygons.size(); ++i) {
        const auto& p = polygons[i];
        for (int j = 0; j < 3; ++j) {
            int a = p.vertices[j];
            int b = p.vertices[(j + 1) % 3];
            edge_map[make_edge_key(a, b)].push_back(i);
        }
    }

    vector<vector<int>> adjacency(polygons.size());
    for (const auto& pair : edge_map) {
        const auto& faces = pair.second;
        for (int i = 0; i < faces.size(); ++i)
            for (int j = i + 1; j < faces.size(); ++j) {
                adjacency[faces[i]].push_back(faces[j]);
                adjacency[faces[j]].push_back(faces[i]);
            }
    }
    return adjacency;
}

void growRegion(int seed_id, int region_id,
                vector<int>& region_ids,
                const vector<vector<int>>& adjacency,
                const vector<Eigen::Vector3f>& face_normals,
                float angle_thresh_rad) { // On retire les paramètres de plan inutiles
    queue<int> q;
    q.push(seed_id);
    region_ids[seed_id] = region_id;

    while (!q.empty()) {
        int current = q.front(); q.pop();
        
        for (int neighbor : adjacency[current]) {
            if (region_ids[neighbor] != -1) continue;
            
            // CRITÈRE PRINCIPAL : Angle entre la face actuelle et sa voisine
            float angle = angleBetweenNormals(face_normals[current], face_normals[neighbor]);
            
            // Si la surface est lisse (petit angle), on continue de propager
            if (angle < angle_thresh_rad) {
                region_ids[neighbor] = region_id;
                q.push(neighbor);
            }
        }
    }
}

void mergeSmallRegions(vector<int>& region_ids, 
                      const vector<vector<int>>& adjacency,
                      int min_region_size) {
    bool changed = true;
    while (changed) {
        changed = false;
        
        unordered_map<int, vector<int>> regions;
        for (int i = 0; i < region_ids.size(); ++i) {
            regions[region_ids[i]].push_back(i);
        }

        vector<int> small_regions;
        for (auto const& [id, faces] : regions) {
            if (faces.size() < min_region_size) {
                small_regions.push_back(id);
            }
        }

        for (int id : small_regions) {
            if (regions.find(id) == regions.end() || regions[id].empty()) continue;

            unordered_map<int, int> adjacent_counts;
            for (int fid : regions[id]) {
                for (int neighbor : adjacency[fid]) {
                    int neighbor_region = region_ids[neighbor];
                    if (neighbor_region != id) {
                        adjacent_counts[neighbor_region]++;
                    }
                }
            }
            
            if (!adjacent_counts.empty()) {
                int new_region = max_element(adjacent_counts.begin(), adjacent_counts.end(),
                    [](const pair<int, int>& a, const pair<int, int>& b) {
                        return a.second < b.second;
                    })->first;

                if (regions[id].size() < regions[new_region].size()) {
                    for (int fid : regions[id]) {
                        region_ids[fid] = new_region;
                    }
                    regions[new_region].insert(regions[new_region].end(), regions[id].begin(), regions[id].end());
                    regions.erase(id);
                    changed = true;
                }
            }
        }
    }
}

void writeFaceLabels(const string& filename, const vector<int>& region_ids) {
    ofstream outfile(filename);
    if (!outfile.is_open()) {
        cerr << "Error opening file for writing face labels: " << filename << endl;
        return;
    }

    for (size_t i = 0; i < region_ids.size(); ++i) {
        outfile << i << " " << region_ids[i] << "\n";
    }

    outfile.close();
    cout << "Face labels written to: " << filename << endl;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        PCL_ERROR("Usage: ./region_detector input.ply output.ply [labels_output.txt]\n");
        return -1;
    }

    string input_file = argv[1];
    string output_file = argv[2];
    string labels_file = (argc >= 4) ? argv[3] : "face_labels.txt";

    // Load mesh
    PolygonMesh mesh;
    if (io::loadPLYFile(input_file, mesh) != 0) {
        PCL_ERROR("Error loading PLY file\n");
        return -1;
    }

    PointCloud<PointXYZ>::Ptr input_cloud(new PointCloud<PointXYZ>());
    fromPCLPointCloud2(mesh.cloud, *input_cloud);

    const auto& faces = mesh.polygons;

    // Calculate face normals
    vector<Eigen::Vector3f> face_normals;
    for (const auto& face : faces) {
        Eigen::Vector3f a = input_cloud->points[face.vertices[0]].getVector3fMap();
        Eigen::Vector3f b = input_cloud->points[face.vertices[1]].getVector3fMap();
        Eigen::Vector3f c = input_cloud->points[face.vertices[2]].getVector3fMap();
        face_normals.push_back((b - a).cross(c - a).normalized());
    }

    auto adjacency = buildFaceAdjacency(faces);
    int face_count = faces.size();
    vector<int> region_ids(face_count, -1);

    // Automatic angle threshold calculation
    vector<float> all_angles;
    for (int i = 0; i < face_count; ++i) {
        for (int j : adjacency[i]) {
            if (i < j) {
                float angle_deg = angleBetweenNormals(face_normals[i], face_normals[j]) * 180.0f / M_PI;
                all_angles.push_back(angle_deg);
            }
        }
    }
    sort(all_angles.begin(), all_angles.end());

    float percentile = 0.78f; // Higher = more strict
    float angle_threshold_deg = all_angles[int(all_angles.size() * percentile)];
    float angle_threshold_rad = angle_threshold_deg * M_PI / 180.0f;
    float plane_dist_threshold = 0.3f; // Adjust based on your mesh scale

    cout << "Automatic angle threshold: " << angle_threshold_deg << " degrees\n";
    cout << "Plane distance threshold: " << plane_dist_threshold << endl;

    // First pass: initial region growing
    int region_id = 0;
    vector<Eigen::Vector4f> planes;
    for (int i = 0; i < face_count; ++i) {
        if (region_ids[i] != -1) continue;

        // Get initial region points
        vector<Eigen::Vector3f> region_points;
        for (int vid : faces[i].vertices) {
            region_points.push_back(input_cloud->points[vid].getVector3fMap());
        }

        // Fit initial plane
        Eigen::Vector4f plane = fitPlaneLSQ(region_points);
        planes.push_back(plane);

        // Grow region with additional parameters
        growRegion(i, region_id++, region_ids, adjacency, face_normals, angle_threshold_rad);
    }

    // Merge small regions
    const int min_region_size = 500; // Adjust based on your mesh
    mergeSmallRegions(region_ids, adjacency, min_region_size);

    // Reassign region IDs to be contiguous
    unordered_map<int, int> region_remap;
    int new_region_id = 0;
    for (int& id : region_ids) {
        if (region_remap.find(id) == region_remap.end()) {
            region_remap[id] = new_region_id++;
        }
        id = region_remap[id];
    }

    // Write face labels to file
    writeFaceLabels(labels_file, region_ids);

    // Assign random colors
    unordered_map<int, array<uint8_t, 3>> region_colors;
    srand(time(NULL));
    for (int i = 0; i < new_region_id; ++i) {
        region_colors[i] = {uint8_t(rand()%256), uint8_t(rand()%256), uint8_t(rand()%256)};
    }

    // Create colored output mesh
    PointCloud<PointXYZRGB>::Ptr colored_cloud(new PointCloud<PointXYZRGB>());
    vector<Vertices> new_faces;

    for (int i = 0; i < face_count; ++i) {
        const auto& face = faces[i];
        int rid = region_ids[i];
        auto color = region_colors[rid];

        Vertices new_face;
        for (int j = 0; j < 3; ++j) {
            PointXYZ pt = input_cloud->points[face.vertices[j]];
            PointXYZRGB pt_rgb;
            pt_rgb.x = pt.x;
            pt_rgb.y = pt.y;
            pt_rgb.z = pt.z;
            pt_rgb.r = color[0];
            pt_rgb.g = color[1];
            pt_rgb.b = color[2];

            new_face.vertices.push_back(colored_cloud->size());
            colored_cloud->push_back(pt_rgb);
        }
        new_faces.push_back(new_face);
    }

    // Save output
    PolygonMesh output_mesh;
    toPCLPointCloud2(*colored_cloud, output_mesh.cloud);
    output_mesh.polygons = new_faces;
    io::savePLYFileBinary(output_file, output_mesh);

    cout << "Mesh exported with " << new_region_id << " planar regions\n";

    return 0;
}