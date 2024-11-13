#include "HeightMapHandler.h"

HeightMapHandler::HeightMapHandler(const std::string& filePath, int maxPoints) {
    heightMapPoints = loadPointsFromFile(filePath, maxPoints);
    centerPointsAroundOrigin(heightMapPoints);
}

std::vector<glm::vec3> HeightMapHandler::loadPointsFromFile(const std::string& filePath, int maxPoints = -1)
{
    std::ifstream file(filePath);
    std::vector<glm::vec3> points;
    file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    std::string line;
    int pointCount = 0;

    while (std::getline(file, line) && (maxPoints < 0 || pointCount < maxPoints)) {
        std::stringstream ss(line);
        std::string xStr, yStr, zStr;

        // Read the comma-separated values
        if (std::getline(ss, xStr, ',') &&
            std::getline(ss, yStr, ',') &&
            std::getline(ss, zStr, ',')) {

            float x = std::stof(xStr) - 607200;
            float y = std::stof(yStr) - 6750618;
            float z = std::stof(zStr) - 270;

            points.emplace_back(x, z, y); // Swap y/z for convenience
            pointCount++;
        }
        else {
            std::cerr << "Error parsing line: " << line << std::endl;
        }
    }
    std::cout << "number of points: " << pointCount << std::endl;

    return points;
}

std::vector<glm::vec3> HeightMapHandler::getHeightMapVector()
{
    return heightMapPoints;
}
