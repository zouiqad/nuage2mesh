#ifndef POISSONEVENT_H
#define POISSONEVENT_H

#include "Event.h"

namespace n2m {
struct PoissonReconstructionParameters {
    // Number of nearest neighbors for normal estimation
    // Adjust as needed for large point clouds
    int k_search = 20;

    // Octree depth - controls the resolution of the reconstruction
    // Larger values yield finer detail but increase computation time.
    // For 1 million points, something between 8-10 is often a good start.
    int depth = 8;

    // Solver divide - partitioning for conjugate gradient solver
    int solver_divide = 8;

    // Iso divide - partitioning for surface extraction
    int iso_divide = 8;

    // Min number of samples per octree node
    float samples_per_node = 1.0f;

    // Scale factor to define the bounding box (slightly larger than the data)
    float scale = 1.25f;

    // Confidence in normals from normal estimation step
    // If set to true, the recon uses the magnitude of the normal as confidence
    bool confidence = false;
};

class PoissonEvent : public Event {
public:
    PoissonEvent () = default;


    PoissonEvent (const PoissonReconstructionParameters& params)
        : parameters (params) {
    }

    PoissonReconstructionParameters parameters;
};
}
#endif //POISSONEVENT_H
