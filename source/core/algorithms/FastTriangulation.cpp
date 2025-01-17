#include "FastTriangulation.h"
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/search/kdtree.h>
#include <iostream>
#include "core/utils/Timer.h"


namespace n2m::graphics {
const double FastTriangulation::mu                     = 2.5;
const int FastTriangulation::maximum_nearest_neighbors = 100;
double FastTriangulation::maximum_surface_angle        = M_PI / 4;
double FastTriangulation::minimum_angle                = M_PI / 18;
double FastTriangulation::maximum_angle                = 2 * M_PI / 3;

std::shared_ptr<Mesh> FastTriangulation::reconstruct (
    const std::shared_ptr<PointCloud>& cloud,
    const FastTriangulationParameters& params_) {
    Timer timer ("FastTriangulation::reconstruct");
    timer.start ();

    if (cloud->getPCLCloudPoints ()->empty ()) {
        std::cerr << "Reconstruction Error: Input vertex list is empty." <<
            std::endl;
        return nullptr;
    }

    if (cloud->getPCLCloudPointsNormals ()->points.empty ()) {
        cloud->estimateNormals ();
    }

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (
        new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud->getPCLCloudPointsNormals ());

    // Triangulation
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    gp3.setSearchRadius (params_.searchRadius);
    gp3.setMu (mu);
    gp3.setMaximumNearestNeighbors (maximum_nearest_neighbors);
    gp3.setMaximumSurfaceAngle (maximum_surface_angle);
    gp3.setMinimumAngle (minimum_angle);
    gp3.setMaximumAngle (maximum_angle);
    gp3.setNormalConsistency (params_.normalConsistency);

    gp3.setInputCloud (cloud->getPCLCloudPointsNormals ());
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);

    // Extract vertices
    pcl::PointCloud<pcl::PointXYZ> vertex_cloud;
    pcl::fromPCLPointCloud2 (triangles.cloud, vertex_cloud);

    // Extract triangle indices
    std::vector<unsigned int> output_indices;
    output_indices.reserve (triangles.polygons.size () * 3);
    for (const auto& polygon : triangles.polygons) {
        if (polygon.vertices.size () != 3) {
            std::cerr << "Triangulation Error: Encountered a polygon with " <<
                polygon.vertices.size () << " vertices." << std::endl;
            continue; // Skip non-triangular polygons
        }
        output_indices.push_back (
            static_cast<unsigned int> (polygon.vertices[0]));
        output_indices.push_back (
            static_cast<unsigned int> (polygon.vertices[1]));
        output_indices.push_back (
            static_cast<unsigned int> (polygon.vertices[2]));
    }

    // Extract vertices as a vector
    std::vector<GLfloat> output_vertices;
    output_vertices.reserve (vertex_cloud.points.size () * 3);
    for (const auto& point : vertex_cloud.points) {
        output_vertices.push_back (point.x);
        output_vertices.push_back (point.y);
        output_vertices.push_back (point.z);
    }

    // Create and upload mesh
    auto new_mesh = std::make_shared<Mesh> ();
    new_mesh->upload (output_vertices, 3, output_indices);

    return new_mesh;
}
}
