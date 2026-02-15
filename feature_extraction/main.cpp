#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/IO/polygon_mesh_io.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/squared_distance_3.h>

#include <boost/math/special_functions/legendre.hpp>

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <variant>
#include <filesystem>
#include <fstream>
#include <map>

namespace fs = std::filesystem;

// --- CONFIGURATION ---
const std::string INPUT_FOLDER = "..\\Nasa_obj";
const std::string OUTPUT_JSON = "descriptors.json";
const std::string RECON_FOLDER = "..\\reconstructed_debug";
const int SH_DEGREE_MAX = 20; // Try 15 or 20 for more detail
const int N_SAMPLES = 5000;   // Rays for analysis

// --- TYPES ---
typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point;
typedef CGAL::Surface_mesh<Point> Mesh;
typedef CGAL::AABB_face_graph_triangle_primitive<Mesh> Primitive;
typedef CGAL::AABB_traits<Kernel, Primitive> AABB_traits;
typedef CGAL::AABB_tree<AABB_traits> Tree;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

double K(int l, int m)
{
    double num = (2.0 * l + 1.0);
    double den = 4.0 * M_PI;
    double fact = std::tgamma(l - std::abs(m) + 1) / std::tgamma(l + std::abs(m) + 1);
    return std::sqrt((num / den) * fact);
}

double eval_Y(int l, int m, double theta, double phi)
{
    using boost::math::legendre_p;
    if (m > 0)
        return std::sqrt(2.0) * K(l, m) * std::cos(m * phi) * legendre_p(l, m, std::cos(theta));
    else if (m < 0)
        return std::sqrt(2.0) * K(l, -m) * std::sin(-m * phi) * legendre_p(l, -m, std::cos(theta));
    else
        return K(l, 0) * legendre_p(l, 0, std::cos(theta));
}

void save_reconstruction(const std::vector<std::vector<double>> &coeffs, const std::string &filename)
{
    std::ofstream out(filename);
    if (!out)
        return;

    int n_lat = 100;
    int n_lon = 200;

    std::vector<Point> verts;

    for (int i = 0; i <= n_lat; ++i)
    {
        double theta = M_PI * i / double(n_lat); // 0 to PI
        double sin_theta = std::sin(theta);
        double cos_theta = std::cos(theta);

        for (int j = 0; j <= n_lon; ++j)
        {
            double phi = 2 * M_PI * j / double(n_lon); // 0 to 2PI

            double r = 0.0;
            for (int l = 0; l < coeffs.size(); ++l)
            {
                for (int m = -l; m <= l; ++m)
                {
                    r += coeffs[l][m + l] * eval_Y(l, m, theta, phi);
                }
            }

            double x = r * sin_theta * std::cos(phi);
            double y = r * sin_theta * std::sin(phi);
            double z = r * cos_theta;

            verts.push_back(Point(x, y, z));
        }
    }

    for (const auto &v : verts)
    {
        out << "v " << v.x() << " " << v.y() << " " << v.z() << "\n";
    }

    for (int i = 0; i < n_lat; ++i)
    {
        for (int j = 0; j < n_lon; ++j)
        {
            int p1 = i * (n_lon + 1) + j + 1;
            int p2 = p1 + 1;
            int p3 = (i + 1) * (n_lon + 1) + j + 2;
            int p4 = p3 - 1;

            out << "f " << p1 << " " << p2 << " " << p3 << " " << p4 << "\n";
        }
    }
    out.close();
}

std::vector<double> process_mesh_and_reconstruct(const std::string &filepath, const std::string &name)
{
    Mesh mesh;
    if (!CGAL::IO::read_polygon_mesh(filepath, mesh))
        return {};

    Point c(0, 0, 0);
    double n = (double)mesh.number_of_vertices();
    for (auto v : mesh.vertices())
        c = Point(c.x() + mesh.point(v).x(), c.y() + mesh.point(v).y(), c.z() + mesh.point(v).z());
    Kernel::Vector_3 centroid(c.x() / n, c.y() / n, c.z() / n);

    double max_r = 0.0;
    for (auto v : mesh.vertices())
    {
        mesh.point(v) = mesh.point(v) - centroid;
        double d = std::sqrt(CGAL::squared_distance(mesh.point(v), Point(0, 0, 0)));
        if (d > max_r)
            max_r = d;
    }
    if (max_r > 0)
    {
        for (auto v : mesh.vertices())
        {
            Kernel::Vector_3 v_vec = mesh.point(v) - Point(0, 0, 0);
            mesh.point(v) = Point(0, 0, 0) + (v_vec / max_r);
        }
    }

    Tree tree(faces(mesh).first, faces(mesh).second, mesh);
    struct Sample
    {
        double theta, phi, r;
    };
    std::vector<Sample> samples;
    double golden_ratio = (1.0 + std::sqrt(5.0)) / 2.0;

    for (int i = 0; i < N_SAMPLES; ++i)
    {
        double phi = 2 * M_PI * (i / golden_ratio);
        phi -= 2 * M_PI * std::floor(phi / (2 * M_PI));
        double z = 1 - (2.0 * (i + 0.5) / N_SAMPLES);
        double theta = std::acos(z);

        double rad = std::sqrt(1 - z * z);
        Kernel::Ray_3 ray(Point(0, 0, 0), Kernel::Vector_3(rad * std::cos(phi), rad * std::sin(phi), z));

        auto hit = tree.first_intersection(ray);
        double r_val = 0.0;
        if (hit)
        {
            if (const Point *p = std::get_if<Point>(&(hit->first)))
                r_val = std::sqrt(CGAL::squared_distance(*p, Point(0, 0, 0)));
        }
        samples.push_back({theta, phi, r_val});
    }

    std::vector<std::vector<double>> coeffs(SH_DEGREE_MAX + 1);
    double weight = 4.0 * M_PI / N_SAMPLES;

    for (int l = 0; l <= SH_DEGREE_MAX; ++l)
    {
        coeffs[l].resize(2 * l + 1, 0.0);
        for (const auto &s : samples)
        {
            for (int m = -l; m <= l; ++m)
            {
                coeffs[l][m + l] += s.r * eval_Y(l, m, s.theta, s.phi) * weight;
            }
        }
    }

    std::string recon_path = RECON_FOLDER + "/" + name + "_reconstructed.obj";
    save_reconstruction(coeffs, recon_path);

    std::vector<double> descriptor;
    for (int l = 0; l <= SH_DEGREE_MAX; ++l)
    {
        double energy = 0.0;
        for (double c : coeffs[l])
            energy += c * c;
        descriptor.push_back(std::sqrt(energy));
    }
    return descriptor;
}

int main()
{
    if (!fs::exists(INPUT_FOLDER))
    {
        std::cerr << "Error: Input folder not found.\n";
        return 1;
    }

    // Create debug folder
    if (!fs::exists(RECON_FOLDER))
        fs::create_directory(RECON_FOLDER);

    std::map<std::string, std::vector<double>> results;
    std::cout << "Processing... Check '" << RECON_FOLDER << "' for 3D results.\n";

    for (const auto &entry : fs::directory_iterator(INPUT_FOLDER))
    {
        std::string ext = entry.path().extension().string();
        if (ext == ".obj" || ext == ".off" || ext == ".ply")
        {
            std::string name = entry.path().stem().string();
            std::cout << " > " << name << "... ";

            std::vector<double> desc = process_mesh_and_reconstruct(entry.path().string(), name);
            if (!desc.empty())
            {
                results[name] = desc;
                std::cout << "Done.\n";
            }
        }
    }

    // Write JSON
    std::ofstream json(OUTPUT_JSON);
    json << "{\n";
    bool first = true;
    for (const auto &[name, vec] : results)
    {
        if (!first)
            json << ",\n";
        json << "  \"" << name << "\": [";
        for (size_t i = 0; i < vec.size(); ++i)
        {
            json << vec[i];
            if (i < vec.size() - 1)
                json << ", ";
        }
        json << "]";
        first = false;
    }
    json << "\n}\n";

    return 0;
}