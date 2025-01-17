#ifndef FASTTRIANGULATION_H
#define FASTTRIANGULATION_H

#include <vector>
#include <memory>
#include <glm/vec3.hpp>
#include <pcl/PolygonMesh.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "core/graphics/Mesh.h"
#include "patterns/events/FastTriangulationEvent.h"
#include <core/graphics/PointCloud.h>


namespace n2m::graphics {
struct ReconstructionParameters {
    double search_radius = 0.025;
    bool normal_consistency = false; // Normal consistency flag
    int k_search = 20; // Number of nearest neighbors for normal estimation
};

class FastTriangulation {
public:
    // Reconstruct mesh from point cloud
    static std::shared_ptr<Mesh> reconstruct(
        const std::shared_ptr<PointCloud> &cloud,
        const FastTriangulationParameters &params);

    static const double mu; // Multiplicative factor for nearest neighbors
    static const int maximum_nearest_neighbors;
    static double maximum_surface_angle;
    static double minimum_angle;
    static double maximum_angle;
};
} // n2m

#endif //FASTTRIANGULATION_H
