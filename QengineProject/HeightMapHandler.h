#pragma once
#include <vector>
#include <glm/glm.hpp>
#include <fstream>
#include <sstream>
#include <iostream>

class HeightMapHandler
{
public:
	HeightMapHandler(const std::string& filePath, int maxPoints = -1);

	std::vector<glm::vec3> loadPointsFromFile(const std::string& filePath, int maxPoints);

    std::vector<glm::vec3> getHeightMapVector();

    std::vector<unsigned int> getIndices();

private:

    std::vector<glm::vec3> heightMapPoints;
    std::vector<unsigned int> indices;

    glm::vec3 calculateCentroid(const std::vector<glm::vec3>& points) {
        glm::vec3 sum(0.0f);
        for (const auto& point : points) {
            sum += point;
        }
        return sum / static_cast<float>(points.size());
    }
    std::vector<glm::vec3> centerPointsAroundOrigin(std::vector<glm::vec3>& points) {
        glm::vec3 centroid = calculateCentroid(points);
        for (auto& point : points) {
            point -= centroid;
        }
        return points;
    }
};

