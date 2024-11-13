#pragma once
#include <vector>
#include <glm/glm.hpp>

struct PointCloudComponent {
    std::vector<glm::vec3> points;

    PointCloudComponent() = default;

    PointCloudComponent(const std::vector<glm::vec3>& points)
        : points(points) {}
};