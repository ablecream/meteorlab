#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/vtk_lib_io.h>

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkQuadricDecimation.h>
#include <vtkAppendPolyData.h>
#include <vtkCleanPolyData.h>
#include <vtkPLYWriter.h>
#include <vtkThreshold.h>
#include <vtkGeometryFilter.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>


#include <vector>
#include <algorithm>

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " input.obj output.obj" << std::endl;
        return -1;
    }

    std::string input_file = argv[1];
    std::string output_file = argv[2];

    // Load the mesh using PCL
    pcl::PolygonMesh mesh;
    bool load_success = false;

    // Vérifie l'extension du fichier
    std::string extension = input_file.substr(input_file.find_last_of("."));

    if (extension == ".ply") {
        if (pcl::io::loadPLYFile(input_file, mesh) == 0) {
            load_success = true;
        }
    } else if (extension == ".obj") {
        if (pcl::io::loadOBJFile(input_file, mesh) == 0) {
            load_success = true;
        }
    }

    // Vérifie si le chargement a réussi
    if (!load_success) {
        std::cerr << "Error loading mesh: " << input_file << std::endl;
        return -1;
    }


    // Convert PCL PolygonMesh to PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

    std::cout << "[INFO] Conversion done " << output_file << "\n";
    // Estimate normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    ne.setKSearch(20);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normals);

    std::cout << "[INFO] Normals estimation done" << output_file << "\n";

    // Estimate principal curvatures
    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> pc;
    pc.setInputCloud(cloud);
    pc.setInputNormals(normals);
    pc.setSearchMethod(tree);
    pc.setKSearch(20);
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr pcs(new pcl::PointCloud<pcl::PrincipalCurvatures>);
    pc.compute(*pcs);

    std::cout << "[INFO] Curvature estimation done " << output_file << "\n";

    // Convert PCL PolygonMesh to VTK PolyData
    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    pcl::io::mesh2vtk(mesh, polydata);

    // Create a scalar array for mean curvature
    vtkSmartPointer<vtkFloatArray> curvatureArray = vtkSmartPointer<vtkFloatArray>::New();
    curvatureArray->SetName("MeanCurvature");
    curvatureArray->SetNumberOfComponents(1);
    curvatureArray->SetNumberOfTuples(polydata->GetNumberOfPoints());

    std::cout << "[INFO] creation of scalar array from mean curvature done " << output_file << "\n";

    // Assign mean curvature values to the scalar array
    for (vtkIdType i = 0; i < polydata->GetNumberOfPoints(); ++i) {
        float pc1 = pcs->points[i].pc1;
        float pc2 = pcs->points[i].pc2;
        float mean_curv = 0.5f * (pc1 + pc2);
        curvatureArray->SetValue(i, mean_curv);
    }

    std::cout << "[INFO] asigning mean curvature values to scalar array done " << output_file << "\n";

    // Add the scalar array to the polydata
    polydata->GetPointData()->SetScalars(curvatureArray);

    // Define curvature threshold
    // Collect mean curvature values
    std::vector<float> meanCurvatures;
    meanCurvatures.reserve(polydata->GetNumberOfPoints());

    for (vtkIdType i = 0; i < polydata->GetNumberOfPoints(); ++i) {
        float pc1 = pcs->points[i].pc1;
        float pc2 = pcs->points[i].pc2;
        float mean_curv = 0.5f * (pc1 + pc2);
        meanCurvatures.push_back(mean_curv);
    }

    // Sort to compute quantile
    std::sort(meanCurvatures.begin(), meanCurvatures.end());

    // Choose quantile
    float quantile = 0.75f;  // Adjust this between 0.7–0.95 depending on your tolerance

    // Compute threshold from quantile
    int index = static_cast<int>(quantile * meanCurvatures.size());
    float threshold = meanCurvatures[index];

    std::cout << "[INFO] threshold: " << threshold << "\n";

    // Extract high curvature regions
    vtkSmartPointer<vtkThreshold> highCurvatureThreshold = vtkSmartPointer<vtkThreshold>::New();
    highCurvatureThreshold->SetInputData(polydata);
    highCurvatureThreshold->ThresholdByUpper(threshold);
    highCurvatureThreshold->SetInputArrayToProcess(0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS, "MeanCurvature");
    highCurvatureThreshold->Update();

    
    vtkSmartPointer<vtkGeometryFilter> highCurvatureGeometry = vtkSmartPointer<vtkGeometryFilter>::New();
    highCurvatureGeometry->SetInputConnection(highCurvatureThreshold->GetOutputPort());
    highCurvatureGeometry->Update();
    
    vtkSmartPointer<vtkPolyData> highCurvatureMesh = highCurvatureGeometry->GetOutput();

    std::cout << "[INFO] Extract high curvature done " << output_file << "\n";

    // Extract low curvature regions
    vtkSmartPointer<vtkThreshold> lowCurvatureThreshold = vtkSmartPointer<vtkThreshold>::New();
    lowCurvatureThreshold->SetInputData(polydata);
    lowCurvatureThreshold->ThresholdByLower(threshold);
    lowCurvatureThreshold->SetInputArrayToProcess(0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS, "MeanCurvature");
    lowCurvatureThreshold->Update();

    vtkSmartPointer<vtkGeometryFilter> lowCurvatureGeometry = vtkSmartPointer<vtkGeometryFilter>::New();
    lowCurvatureGeometry->SetInputConnection(lowCurvatureThreshold->GetOutputPort());
    lowCurvatureGeometry->Update();

    vtkSmartPointer<vtkPolyData> lowCurvatureMesh = lowCurvatureGeometry->GetOutput();

    std::cout << "[INFO] Extract low curvature done " << output_file << "\n";

    // Decimate low curvature mesh more aggressively
    vtkSmartPointer<vtkQuadricDecimation> lowDecimate = vtkSmartPointer<vtkQuadricDecimation>::New();
    lowDecimate->SetInputData(lowCurvatureMesh);
    lowDecimate->SetTargetReduction(0.8); // Reduce 70% of triangles
    lowDecimate->Update();

    std::cout << "[INFO] Extract agressive decimation done " << output_file << "\n";

    // Decimate high curvature mesh conservatively
    vtkSmartPointer<vtkQuadricDecimation> highDecimate = vtkSmartPointer<vtkQuadricDecimation>::New();
    highDecimate->SetInputData(highCurvatureMesh);
    highDecimate->SetTargetReduction(0.2); // Reduce 10% of triangles
    highDecimate->Update();

    std::cout << "[INFO] Extract gentle decimation done " << output_file << "\n";


    // Merge the decimated meshes
    vtkSmartPointer<vtkAppendPolyData> appendFilter = vtkSmartPointer<vtkAppendPolyData>::New();
    appendFilter->AddInputData(lowDecimate->GetOutput());
    appendFilter->AddInputData(highDecimate->GetOutput());
    appendFilter->Update();

    std::cout << "[INFO] Extract merge done " << output_file << "\n";


    // Clean the merged mesh
    vtkSmartPointer<vtkCleanPolyData> cleanFilter = vtkSmartPointer<vtkCleanPolyData>::New();
    cleanFilter->SetInputConnection(appendFilter->GetOutputPort());
    cleanFilter->Update();

    std::cout << "[INFO] cleaning done " << output_file << "\n";

    // Write the final mesh to a PLY file
    vtkSmartPointer<vtkPLYWriter> writer = vtkSmartPointer<vtkPLYWriter>::New();
    writer->SetFileName(output_file.c_str());
    writer->SetInputConnection(cleanFilter->GetOutputPort());
    writer->Write();

    std::cout << "Decimation complete. Output saved to " << output_file << std::endl;

    return 0;
}
