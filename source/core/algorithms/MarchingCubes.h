#ifndef MARCHINGCUBES_H
#define MARCHINGCUBES_H

#include <vector>
#include <glm/glm.hpp>
#include "../graphics/Geometry.h"

namespace n2m::graphics {
class MarchingCubes {
public:
    /**
     * @brief Reconstructs a surface mesh from a raw point cloud using
     *        a voxel grid + Marching Cubes.
     * @param points The input point cloud (no normals required).
     * @param gridResolution The resolution along each axis of the voxel grid.
     *                       Higher => more fine detail but more memory usage.
     * @return A Geometry object (triangle mesh) suitable for rendering.
     */
    static Geometry reconstructMesh (const std::vector<glm::vec3>& points,
        int gridResolution = 50);

private:
    // Each voxel cell will store some "occupancy" (e.g. 0 or 1).
    // We’ll store it in a float to easily use isoLevel=0.5 if you like.
    // (or you can store booleans if that’s simpler).
    static std::vector<float> buildOccupancyGrid (
        const std::vector<glm::vec3>& points,
        glm::vec3& minBound,
        glm::vec3& maxBound,
        int gridResolution);

    static void computeBoundingBox (const std::vector<glm::vec3>& points,
        glm::vec3& minBound,
        glm::vec3& maxBound);

    // Actually do marching cubes over the occupancy grid
    static void runMarchingCubes (const std::vector<float>& grid,
        const glm::vec3& minBound,
        const glm::vec3& maxBound,
        int gridResolution,
        std::vector<glm::vec3>& outVertices,
        std::vector<unsigned int>& outIndices);

    // Helper methods for MC
    static float getGridValue (const std::vector<float>& grid,
        int x,
        int y,
        int z,
        int N);
    static glm::vec3 interpolateEdge (const glm::vec3& p1,
        const glm::vec3& p2,
        float valP1,
        float valP2,
        float isoLevel);

    // The MC tables (edgeTable & triTable).
    static const int edgeTable[256];
    static const int triTable[256][16];
};
} // namespace n2m::graphics

#endif
