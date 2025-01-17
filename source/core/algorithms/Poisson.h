#ifndef Poisson_H
#define Poisson_H

#include <vector>
#include <memory>
#include <glm/vec3.hpp>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "core/graphics/PointCloud.h"
#include "core/graphics/Mesh.h"
#include "patterns/events/PoissonEvent.h"

namespace n2m::graphics {
class Poisson {
public:
    static std::shared_ptr<Mesh> reconstruct(
        const std::shared_ptr<PointCloud> &cloud,
        const PoissonReconstructionParameters &params
    );
};
} // namespace n2m::graphics

#endif // Poisson_H
