#ifndef POINT_H
#define POINT_H

#include <glm/glm.hpp>

namespace n2m {
class Point {
public:
    Point(glm::vec3 const &v) : position(v) {
        this->normal = normal = glm::vec3(0, 0, 0);
    }

    Point(double x, double y, double z) : position(
        x, y, z) {
        this->normal = normal = glm::vec3(0, 0, 0);
    }

    double get(size_t index) const {
        return position[index];
    }


    double compute_distance(const Point &p) const {
        return glm::distance<3, float>(position, p.position);
    }


    glm::vec3 position;
    glm::vec3 normal = glm::vec3(0, 0, 0);
};
} // n2m

#endif //POINT_H
