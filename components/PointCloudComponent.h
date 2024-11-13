#pragma once
#include <vector>
#include <glm/glm.hpp>

struct PointCloudComponent {
    std::vector<glm::vec3> points;        // Stores the position of each point in the cloud
    std::vector<glm::vec3> colors;        // Optional: Stores color for each point, if needed
    bool useColors = false;               // Flag to indicate if colors are used

    PointCloudComponent() = default;

    PointCloudComponent(const std::vector<glm::vec3>& points)
        : points(points), useColors(false) {}

    PointCloudComponent(const std::vector<glm::vec3>& points, const std::vector<glm::vec3>& colors)
        : points(points), colors(colors), useColors(true) {}
};