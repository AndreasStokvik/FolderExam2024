#include "HeightMapHandler.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <fstream>
#include <vector>

HeightMapHandler::HeightMapHandler(const std::string& filePath, int maxPoints) {
    heightMapPoints = loadPointsFromFile(filePath, maxPoints);
}

std::vector<glm::vec3> HeightMapHandler::loadPointsFromFile(const std::string& filePath, int maxPoints) {
    std::ifstream file(filePath);

    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filePath << std::endl;
        return {};
    }

    file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    glm::vec3 minPoint(std::numeric_limits<float>::max());
    glm::vec3 maxPoint(std::numeric_limits<float>::lowest());
    int pointCount = 0;

    std::string line;
    while (std::getline(file, line) && (maxPoints < 0 || pointCount < maxPoints)) {
        std::stringstream ss(line);
        std::string xStr, yStr, zStr;

        if (std::getline(ss, xStr, ',') && std::getline(ss, yStr, ',') && std::getline(ss, zStr, ',')) {
            float x = std::stof(xStr);
            float y = std::stof(yStr);
            float z = std::stof(zStr);

            minPoint.x = std::min(minPoint.x, x);
            minPoint.y = std::min(minPoint.y, y);
            minPoint.z = std::min(minPoint.z, z);

            maxPoint.x = std::max(maxPoint.x, x);
            maxPoint.y = std::max(maxPoint.y, y);
            maxPoint.z = std::max(maxPoint.z, z);

            pointCount++;
        }
    }

    if (pointCount == 0) {
        std::cerr << "No points were loaded in the first pass!" << std::endl;
        return {};
    }

    glm::vec3 center = (minPoint + maxPoint) * 0.5f;
    file.clear();
    file.seekg(0);
    file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    std::vector<glm::vec3> points;
    pointCount = 0;
    while (std::getline(file, line) && (maxPoints < 0 || pointCount < maxPoints)) {
        std::stringstream ss(line);
        std::string xStr, yStr, zStr;

        if (std::getline(ss, xStr, ',') && std::getline(ss, yStr, ',') && std::getline(ss, zStr, ',')) {
            float x = std::stof(xStr);
            float y = std::stof(yStr);
            float z = std::stof(zStr);

            points.emplace_back(x - center.x, z - center.z, y - center.y); // Swap y/z for convenience
            pointCount++;
        }
        else {
            std::cerr << "Error parsing line: " << line << std::endl;
        }
    }

    std::cout << "number of points: " << pointCount << std::endl;
    /*std::cout << "bounding box min: (" << minPoint.x << ", " << minPoint.y << ", " << minPoint.z << ")" << std::endl;
    std::cout << "bounding box max: (" << maxPoint.x << ", " << maxPoint.y << ", " << maxPoint.z << ")" << std::endl;
    std::cout << "center: (" << center.x << ", " << center.y << ", " << center.z << ")" << std::endl;*/

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
    return indices;
}

std::vector<glm::vec3> HeightMapHandler::generateNormals(const std::vector<unsigned int>& indices) {
    const std::vector<glm::vec3> vertices = heightMapPoints;

    normals.clear();
    normals.resize(vertices.size(), glm::vec3(0.0f));

    for (size_t i = 0; i < indices.size(); i += 3) {
        unsigned int idx0 = indices[i];
        unsigned int idx1 = indices[i + 1];
        unsigned int idx2 = indices[i + 2];

        glm::vec3 v0 = vertices[idx0];
        glm::vec3 v1 = vertices[idx1];
        glm::vec3 v2 = vertices[idx2];

        glm::vec3 edge1 = v1 - v0;
        glm::vec3 edge2 = v2 - v0;

        glm::vec3 normal = glm::cross(edge1, edge2);

        normals[idx0] += normal;
        normals[idx1] += normal;
        normals[idx2] += normal;
    }

    for (auto& normal : normals) {
        if (glm::length(normal) > 0.0f) {
            normal = glm::normalize(normal);
        }
    }

    return normals;
}

std::vector<unsigned int> HeightMapHandler::getTriangulationIndices()
{
    const auto& heightMap = getHeightMapVector();
    std::vector<Point2D> points2D;
    std::unordered_map<Point2D, unsigned int> pointIndexMap;

    for (size_t i = 0; i < heightMap.size(); ++i) {
        Point2D point(heightMap[i].x, heightMap[i].z);
        points2D.push_back(point);
        pointIndexMap[point] = static_cast<unsigned int>(i);
    }

    Delaunay delaunay;
    delaunay.insert(points2D.begin(), points2D.end());

    for (auto face = delaunay.finite_faces_begin(); face != delaunay.finite_faces_end(); ++face) {
        auto v0 = face->vertex(0)->point();
        auto v1 = face->vertex(1)->point();
        auto v2 = face->vertex(2)->point();

        indices.push_back(pointIndexMap[v0]);
        indices.push_back(pointIndexMap[v1]);
        indices.push_back(pointIndexMap[v2]);
    }

    return indices;
}

glm::vec3 HeightMapHandler::getClosestNormal(float x, float z, glm::vec3 scale) const {
    if (heightMapPoints.empty() || indices.empty()) {
        throw std::runtime_error("Height map points or indices are not initialized.");
    }

    glm::vec3 position(x, 0.0f, z);

    for (size_t i = 0; i < indices.size(); i += 3) {
        glm::vec3 v0 = heightMapPoints[indices[i]];
        glm::vec3 v1 = heightMapPoints[indices[i + 1]];
        glm::vec3 v2 = heightMapPoints[indices[i + 2]];

        if (isPointInTriangle(position, v0, v1, v2)) {
            glm::vec3 edge1 = v1 - v0;
            glm::vec3 edge2 = v2 - v0;
            glm::vec3 normal = glm::normalize(glm::cross(edge1, edge2));
            return normal;
        }
    }

    // Fallback: If no triangle was found, return the default normal
    return glm::vec3(0.0f, 1.0f, 0.0f);
}

bool HeightMapHandler::isPointInTriangle(const glm::vec3& p, const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2) const {
    glm::vec3 v0v1 = v1 - v0;
    glm::vec3 v0v2 = v2 - v0;
    glm::vec3 v0p = p - v0;

    double dot00 = glm::dot(v0v1, v0v1);
    double dot01 = glm::dot(v0v1, v0v2);
    double dot02 = glm::dot(v0v1, v0p);
    double dot11 = glm::dot(v0v2, v0v2);
    double dot12 = glm::dot(v0v2, v0p);

    double invDenom = 1.0f / (dot00 * dot11 - dot01 * dot01);
    double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
    double v = (dot00 * dot12 - dot01 * dot02) * invDenom;

    return (u >= 0.0f && v >= 0.0f && u + v <= 1.0f);
}

float HeightMapHandler::getHeightAt(float x, float z) const {
    if (heightMapPoints.empty() || indices.empty()) {
        throw std::runtime_error("Height map data or triangulation indices are missing.");
    }

    glm::vec3 p(x, 0.0f, z); // The query point (height is irrelevant for this)

    // Iterate through the triangulation indices to find the triangle containing the point
    for (size_t i = 0; i < indices.size(); i += 3) {
        const glm::vec3& v0 = heightMapPoints[indices[i]];
        const glm::vec3& v1 = heightMapPoints[indices[i + 1]];
        const glm::vec3& v2 = heightMapPoints[indices[i + 2]];

        if (isPointInTriangle(p, v0, v1, v2)) {
            // Calculate barycentric coordinates for the point within the triangle
            glm::vec3 v0v1 = v1 - v0;
            glm::vec3 v0v2 = v2 - v0;
            glm::vec3 v0p = p - v0;

            float dot00 = glm::dot(v0v2, v0v2);
            float dot01 = glm::dot(v0v2, v0v1);
            float dot02 = glm::dot(v0v2, v0p);
            float dot11 = glm::dot(v0v1, v0v1);
            float dot12 = glm::dot(v0v1, v0p);

            float invDenom = 1.0f / (dot00 * dot11 - dot01 * dot01);
            float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
            float v = (dot00 * dot12 - dot01 * dot02) * invDenom;
            float w = 1.0f - u - v;

            // Use barycentric interpolation to calculate the height
            return u * v1.y + v * v2.y + w * v0.y;
        }
    }

    // If no triangle is found, fallback to nearest neighbor height
    float nearestHeight = heightMapPoints[0].y;
    float nearestDistanceSq = std::numeric_limits<float>::max();

    for (const auto& point : heightMapPoints) {
        float dx = point.x - x;
        float dz = point.z - z;
        float distanceSq = dx * dx + dz * dz;

        if (distanceSq < nearestDistanceSq) {
            nearestHeight = point.y;
            nearestDistanceSq = distanceSq;
        }
    }

    return nearestHeight;
}

