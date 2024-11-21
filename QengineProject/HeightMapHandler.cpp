#include "HeightMapHandler.h"
#include <glm/vec3.hpp>
#include <unordered_map>
#include <glm/glm.hpp>
#include <functional>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <limits>

struct Vec3Hash {
    std::size_t operator()(const glm::vec3& v) const {
        std::size_t hx = std::hash<float>{}(v.x);
        std::size_t hy = std::hash<float>{}(v.y);
        std::size_t hz = std::hash<float>{}(v.z);
        return hx ^ (hy << 1) ^ (hz << 2);
    }
};

struct Vec3Equal {
    bool operator()(const glm::vec3& v1, const glm::vec3& v2) const {
        return v1 == v2;
    }
};

struct Edge {
    glm::vec3 p1, p2;

    bool operator==(const Edge& other) const {
        return (p1 == other.p1 && p2 == other.p2) || (p1 == other.p2 && p2 == other.p1);
    }
};

struct Triangle {
    glm::vec3 p1, p2, p3;

    bool operator==(const Triangle& other) const {
        return (p1 == other.p1 || p1 == other.p2 || p1 == other.p3) &&
            (p2 == other.p1 || p2 == other.p2 || p2 == other.p3) &&
            (p3 == other.p1 || p3 == other.p2 || p3 == other.p3);
    }

    bool isPointInCircumcircle(const glm::vec3& point) const;
};

bool Triangle::isPointInCircumcircle(const glm::vec3& point) const {
    float ax = p1.x - point.x, az = p1.z - point.z;
    float bx = p2.x - point.x, bz = p2.z - point.z;
    float cx = p3.x - point.x, cz = p3.z - point.z;

    float A = ax * ax + az * az, B = bx * bx + bz * bz, C = cx * cx + cz * cz;
    float det = ax * (bz * C - B * cz) - az * (bx * C - B * cx) + A * (bx * cz - bz * cx);

    return det > 0; // Inside the circumcircle if determinant > 0
}

void insertPoint(const glm::vec3& point, std::vector<Triangle>& triangles) {
    std::vector<Triangle> badTriangles;

    // Find all triangles whose circumcircle contains the point
    for (const auto& triangle : triangles) {
        if (triangle.isPointInCircumcircle(point)) {
            badTriangles.push_back(triangle);
        }
    }

    // Find boundary edges
    std::vector<Edge> boundaryEdges;
    for (const auto& triangle : badTriangles) {
        std::vector<Edge> edges = { {triangle.p1, triangle.p2}, {triangle.p2, triangle.p3}, {triangle.p3, triangle.p1} };
        for (const auto& edge : edges) {
            auto it = std::find(boundaryEdges.begin(), boundaryEdges.end(), edge);
            if (it != boundaryEdges.end()) {
                boundaryEdges.erase(it); // Remove shared edge
            }
            else {
                boundaryEdges.push_back(edge); // Add non-shared edge
            }
        }
    }

    // Remove bad triangles
    for (const auto& triangle : badTriangles) {
        triangles.erase(std::remove(triangles.begin(), triangles.end(), triangle), triangles.end());
    }

    // Create new triangles by connecting the point to the boundary edges
    for (const auto& edge : boundaryEdges) {
        triangles.emplace_back(Triangle{ edge.p1, edge.p2, point });
    }
}

Triangle createSuperTriangle(const std::vector<glm::vec3>& points) {
    float minX = std::numeric_limits<float>::max(), maxX = std::numeric_limits<float>::lowest();
    float minZ = std::numeric_limits<float>::max(), maxZ = std::numeric_limits<float>::lowest();

    // Find bounding box of points
    for (const auto& point : points) {
        minX = std::min(minX, point.x);
        maxX = std::max(maxX, point.x);
        minZ = std::min(minZ, point.z);
        maxZ = std::max(maxZ, point.z);
    }

    // Buffer for super-triangle size
    float buffer = 10.0f;
    float centerX = (minX + maxX) / 2, centerZ = (minZ + maxZ) / 2;
    float dx = maxX - minX, dz = maxZ - minZ;
    float deltaMax = std::max(dx, dz);

    // Create super-triangle with points outside bounding box
    glm::vec3 p1(centerX - deltaMax * buffer, 0.0f, centerZ - deltaMax);
    glm::vec3 p2(centerX + deltaMax * buffer, 0.0f, centerZ - deltaMax);
    glm::vec3 p3(centerX, 0, centerZ + deltaMax);

    return Triangle{ p1, p2, p3 };
}

void finalizeTriangles(std::vector<Triangle>& triangles, const Triangle& superTriangle) {
    const glm::vec3& stP1 = superTriangle.p1;
    const glm::vec3& stP2 = superTriangle.p2;
    const glm::vec3& stP3 = superTriangle.p3;

    // Remove any triangles that share points with the super-triangle
    triangles.erase(std::remove_if(triangles.begin(), triangles.end(),
        [&](const Triangle& triangle) {
            return (triangle.p1 == stP1 || triangle.p1 == stP2 || triangle.p1 == stP3) ||
                (triangle.p2 == stP1 || triangle.p2 == stP2 || triangle.p2 == stP3) ||
                (triangle.p3 == stP1 || triangle.p3 == stP2 || triangle.p3 == stP3);
        }), triangles.end());
}

std::tuple<std::vector<glm::vec3>, std::vector<unsigned int>> trianglesToIndices(const std::vector<Triangle>& triangles) {
    std::vector<glm::vec3> uniquePoints;
    std::unordered_map<glm::vec3, int, Vec3Hash, Vec3Equal> pointToIndex;
    std::vector<unsigned int> indices;

    // Map triangles to indices
    for (const auto& triangle : triangles) {
        auto addPoint = [&](const glm::vec3& point) {
            auto it = pointToIndex.find(point);
            if (it == pointToIndex.end()) {
                int newIndex = static_cast<int>(uniquePoints.size());
                uniquePoints.push_back(point);
                pointToIndex[point] = newIndex;
            }
            return pointToIndex[point];
            };

        indices.push_back(addPoint(triangle.p1));
        indices.push_back(addPoint(triangle.p2));
        indices.push_back(addPoint(triangle.p3));
    }

    return { uniquePoints, indices };
}

HeightMapHandler::HeightMapHandler(const std::string& filePath, int maxPoints) {
    heightMapPoints = loadPointsFromFile(filePath, maxPoints);
}

std::vector<glm::vec3> HeightMapHandler::loadPointsFromFile(const std::string& filePath, int maxPoints) {
    std::ifstream file(filePath);
    std::vector<glm::vec3> points;
    file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    std::string line;
    int pointCount = 0;

    int barWidth = 50;
    int processedLines = 0;

    while (std::getline(file, line) && (maxPoints < 0 || pointCount < maxPoints)) {
        std::stringstream ss(line);
        std::string xStr, yStr, zStr;

        if (std::getline(ss, xStr, ',') && std::getline(ss, yStr, ',') && std::getline(ss, zStr, ',')) {
            float x = std::stof(xStr) - 607200;
            float y = std::stof(yStr) - 6750618;
            float z = std::stof(zStr) - 270;
            points.emplace_back(x, z, y); // Swap y/z for convenience
            pointCount++;
        }
        else {
            std::cerr << "Error parsing line: " << line << std::endl;
        }

        //processedLines++;
        //if (processedLines % 100000 == 0 || processedLines == maxPoints) {
        //    float progress = static_cast<float>(processedLines) / maxPoints;
        //    int pos = barWidth * progress;
        //    std::cout << "\r[";  // Start overwriting the same line
        //    for (int i = 0; i < barWidth; ++i) {
        //        if (i < pos) std::cout << "=";
        //        else if (i == pos) std::cout << ">";
        //        else std::cout << " ";
        //    }
        //    std::cout << "] " << int(progress * 100.0f) << "%";
        //    std::flush(std::cout);
        //}
    }
    std::cout << "number of points: " << pointCount << std::endl;

    return points;
}

std::vector<glm::vec3> HeightMapHandler::getHeightMapVector()
{
    return heightMapPoints;
}
std::vector<glm::vec3> HeightMapHandler::getNormals()
{
    return normals;
}
std::vector<unsigned int> HeightMapHandler::getIndices()
{
    /*std::cout << "Indices: ";
    for (const auto& index : indices) {
        std::cout << index << " ";
    }*/
    return indices;
}
std::vector<unsigned int> HeightMapHandler::getTriangulationIndices()
{
    std::vector<Triangle> triangles;
    Triangle superTriangle = createSuperTriangle(heightMapPoints);
    triangles.push_back(superTriangle);

    for (const auto& point : heightMapPoints) {
        insertPoint(point, triangles);
    }

    finalizeTriangles(triangles, superTriangle);
    auto result = trianglesToIndices(triangles);
    heightMapPoints = std::get<0>(result);
    indices = std::get<1>(result);
    return indices;
}

std::vector<unsigned int> HeightMapHandler::setIndices(std::vector<unsigned int> newIndices)
{
    indices = newIndices;
    return indices;
}

std::pair<std::vector<glm::vec3>, std::vector<unsigned int>> HeightMapHandler::BSplineSurface() {
    std::vector<glm::vec3> vertices;
    std::vector<unsigned int> indices;

    // Ranges of the dataset in each direction
    const unsigned int knotVectorUSize = 1600;
    const unsigned int knotVectorVSize = 1200;

    const unsigned int degree = 2;
    std::vector<float> knotVectorU;
    std::vector<float> knotVectorV;
    std::vector<glm::vec3> controlPoints = heightMapPoints;

    // Generate knot vector U
    for (int i = 0; i < knotVectorUSize + degree + 1; i++) {
        if (i < degree + 1) {
            knotVectorU.push_back(0.0f); // Start
        }
        else if (i > knotVectorUSize - 1) {
            knotVectorU.push_back(knotVectorUSize - degree); // End
        }
        else {
            knotVectorU.push_back(i-degree);
        }
    }

    // Generate knot vector V
    for (int i = 0; i < knotVectorVSize + degree + 1; i++) {
        if (i < degree + 1) {
            knotVectorV.push_back(0.0f); // Start
        }
        else if (i > knotVectorVSize - 1) {
            knotVectorV.push_back(knotVectorVSize - degree); // End
        }
        else {
            knotVectorV.push_back(i - degree);
        }
    }



    return { vertices, indices };
}