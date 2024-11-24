#include "HeightMapHandler.h"
#include <glm/glm.hpp>
#include <glm/vec3.hpp>
#include <iostream>
#include <fstream>
#include <vector>

HeightMapHandler::HeightMapHandler(const std::string& filePath, int maxPoints) {
    heightMapPoints = loadPointsFromFile(filePath, maxPoints);
}

std::vector<glm::vec3> HeightMapHandler::loadPointsFromFile(const std::string& filePath, int maxPoints) {
    std::ifstream file(filePath);
    std::vector<glm::vec3> points;
    file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    std::string line;
    int pointCount = 0;

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

glm::vec3 HeightMapHandler::getClosestNormal(float x, float z) const {
    if (heightMapPoints.empty() || indices.empty()) {
        throw std::runtime_error("Height map points or indices are not initialized.");
    }

    glm::vec3 position(x, 0.0f, z); // Project position onto XZ-plane

    // Loop through triangles in the height map (indices are in groups of 3)
    for (size_t i = 0; i < indices.size(); i += 3) {
        // Get the vertices of the current triangle
        glm::vec3 v0 = heightMapPoints[indices[i]];
        glm::vec3 v1 = heightMapPoints[indices[i + 1]];
        glm::vec3 v2 = heightMapPoints[indices[i + 2]];

        // Check if the point lies inside the triangle using barycentric coordinates
        if (isPointInTriangle(position, v0, v1, v2)) {
            // Compute the normal for the triangle
            glm::vec3 edge1 = v1 - v0;
            glm::vec3 edge2 = v2 - v0;
            glm::vec3 normal = glm::normalize(glm::cross(edge1, edge2));
            return normal;
        }
    }

    // Fallback: If no triangle was found, return the default normal (or handle differently)
    return glm::vec3(0.0f, 1.0f, 0.0f);
}

bool HeightMapHandler::isPointInTriangle(const glm::vec3& p, const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2) const {
    glm::vec3 v0v1 = v1 - v0;
    glm::vec3 v0v2 = v2 - v0;
    glm::vec3 v0p = p - v0;

    float dot00 = glm::dot(v0v1, v0v1);
    float dot01 = glm::dot(v0v1, v0v2);
    float dot02 = glm::dot(v0v1, v0p);
    float dot11 = glm::dot(v0v2, v0v2);
    float dot12 = glm::dot(v0v2, v0p);

    float invDenom = 1.0f / (dot00 * dot11 - dot01 * dot01);
    float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
    float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

    // Check if the point is inside the triangle (barycentric coordinates must be between 0 and 1)
    return (u >= 0.0f && v >= 0.0f && u + v <= 1.0f);
}

float HeightMapHandler::getHeightAt(float x, float z) const {
    if (heightMapPoints.empty()) {
        throw std::runtime_error("Height map is empty.");
    }

    float adjustedX = x;
    float adjustedZ = z;

    int gridX = static_cast<int>(std::floor(adjustedX));
    int gridZ = static_cast<int>(std::floor(adjustedZ));

    glm::vec3 h00, h10, h01, h11;
    bool foundH00 = false, foundH10 = false, foundH01 = false, foundH11 = false;

    for (const auto& point : heightMapPoints) {
        if (!foundH00 && static_cast<int>(point.x) == gridX && static_cast<int>(point.z) == gridZ) {
            h00 = point;
            foundH00 = true;
        }
        if (!foundH10 && static_cast<int>(point.x) == gridX + 1 && static_cast<int>(point.z) == gridZ) {
            h10 = point;
            foundH10 = true;
        }
        if (!foundH01 && static_cast<int>(point.x) == gridX && static_cast<int>(point.z) == gridZ + 1) {
            h01 = point;
            foundH01 = true;
        }
        if (!foundH11 && static_cast<int>(point.x) == gridX + 1 && static_cast<int>(point.z) == gridZ + 1) {
            h11 = point;
            foundH11 = true;
        }

        if (foundH00 && foundH10 && foundH01 && foundH11) {
            break;
        }
    }

    if (!foundH00 || !foundH10 || !foundH01 || !foundH11) {
        // Fallback: Find nearest height point
        float nearestHeight = heightMapPoints[0].y;
        float nearestDistanceSq = std::numeric_limits<float>::max();

        for (const auto& point : heightMapPoints) {
            float dx = point.x - adjustedX;
            float dz = point.z - adjustedZ;
            float distanceSq = dx * dx + dz * dz;

            if (distanceSq < nearestDistanceSq) {
                nearestHeight = point.y;
                nearestDistanceSq = distanceSq;
            }
        }

        return nearestHeight; // Return the height of the nearest point
    }

    float tx = adjustedX - gridX;
    float tz = adjustedZ - gridZ;

    float h0 = glm::mix(h00.y, h10.y, tx);
    float h1 = glm::mix(h01.y, h11.y, tx);

    return glm::mix(h0, h1, tz);
}
