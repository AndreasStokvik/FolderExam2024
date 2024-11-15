#pragma once
#include <vector>
#include <glm/glm.hpp>
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>

class HeightMapHandler
{
public:
	HeightMapHandler(const std::string& filePath, int maxPoints = -1);

    std::vector<glm::vec3> getHeightMapVector();
    std::vector<glm::vec3> getNormals();
    std::vector<unsigned int> getIndices();
    std::vector<unsigned int> setIndices(std::vector<unsigned int> delaunayIndices);
private:
    std::vector<glm::vec3> heightMapPoints;
    std::vector<glm::vec3> normals;
    std::vector<unsigned int> indices;

    std::vector<glm::vec3> loadPointsFromFile(const std::string& filePath, int maxPoints);
};

