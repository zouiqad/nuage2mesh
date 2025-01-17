#include "Poisson.h"
#include <pcl/filters/filter.h>
#include <pcl/surface/poisson.h>
#include <iostream>
#include <thread>

#include "core/utils/Timer.h"

namespace n2m::graphics {
std::shared_ptr<Mesh> Poisson::reconstruct (
    const std::shared_ptr<PointCloud>& cloud,
    const PoissonReconstructionParameters& params
    ) {
    Timer timer ("Poisson::reconstruct");
    timer.start ();

    if (!cloud->getPCLCloudPoints () || cloud->getPCLCloudPoints ()->points.
        empty ()) {
        std::cerr << "Reconstruction Error: Input vertex list is empty."
            << std::endl;
        return nullptr;
    }

    if (cloud->getPCLCloudPointsNormals ()->points.empty ()) {
        cloud->estimateNormals ();
    }

    // Poisson reconstruction
    pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setDepth (params.depth);
    poisson.setSolverDivide (params.solver_divide);
    poisson.setIsoDivide (params.iso_divide);
    poisson.setSamplesPerNode (params.samples_per_node);
    poisson.setScale (params.scale);
    poisson.setConfidence (params.confidence);

    poisson.setInputCloud (cloud->getPCLCloudPointsNormals ());

    pcl::PolygonMesh output_mesh;
    poisson.reconstruct (output_mesh);

    // Convert the resulting mesh to our internal Mesh format
    pcl::PointCloud<pcl::PointXYZ> reconstructed_vertices;
    pcl::fromPCLPointCloud2 (output_mesh.cloud, reconstructed_vertices);

    // Gather indices from polygons
    std::vector<unsigned int> output_indices;
    output_indices.reserve (output_mesh.polygons.size () * 3);
    for (const auto& polygon : output_mesh.polygons) {
        if (polygon.vertices.size () == 3) {
            output_indices.push_back (
                static_cast<unsigned int> (polygon.vertices[0]));
            output_indices.push_back (
                static_cast<unsigned int> (polygon.vertices[1]));
            output_indices.push_back (
                static_cast<unsigned int> (polygon.vertices[2]));
        }
    }

    // Gather final vertex positions
    std::vector<GLfloat> output_positions;
    output_positions.reserve (reconstructed_vertices.points.size () * 3);
    for (const auto& pt : reconstructed_vertices.points) {
        output_positions.push_back (pt.x);
        output_positions.push_back (pt.y);
        output_positions.push_back (pt.z);
    }

    // Create our Mesh object
    auto new_mesh = std::make_shared<Mesh> ();
    new_mesh->upload (output_positions, 3, output_indices);


    return new_mesh;
}
} // namespace n2m::graphics
