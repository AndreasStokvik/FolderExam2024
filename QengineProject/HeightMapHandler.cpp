#include "HeightMapHandler.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <fstream>
#include <vector>

std::ostream& operator<<(std::ostream& os, const glm::vec3& vec) {
    os << "(" << vec.x << ", " << vec.y << ", " << vec.z << ")";
    return os;
}

HeightMapHandler::HeightMapHandler(const std::string& filePath, int maxPoints, int skip, float scale) {
    heightMapPoints = loadPointsFromFile(filePath, maxPoints, skip);
    transformScale = scale;
}

std::vector<glm::vec3> HeightMapHandler::loadPointsFromFile(const std::string& filePath, int maxPoints, int skip) {
    std::ifstream file(filePath);

    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filePath << std::endl;
        return {};
    }

    file.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Skip the header

    glm::vec3 minPoint(std::numeric_limits<float>::max());
    glm::vec3 maxPoint(std::numeric_limits<float>::lowest());
    int pointCount = 0;
    int lineIndex = 0;

    std::string line;
    while (std::getline(file, line) && (maxPoints < 0 || pointCount < maxPoints)) {
        if (lineIndex % (skip + 1) != 0) {
            lineIndex++;
            continue;
        }

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
        lineIndex++;
    }

    if (pointCount == 0) {
        std::cerr << "No points were loaded in the first pass!" << std::endl;
        return {};
    }

    glm::vec3 center = (minPoint + maxPoint) * 0.5f;
    file.clear();
    file.seekg(0);
    file.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Skip the header again

    std::vector<glm::vec3> points;
    pointCount = 0;
    lineIndex = 0;
    while (std::getline(file, line) && (maxPoints < 0 || pointCount < maxPoints)) {
        if (lineIndex % (skip + 1) != 0) {
            lineIndex++;
            continue;
        }

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
        lineIndex++;
    }

    //std::cout << "Number of points: " << pointCount << std::endl;

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
        return glm::vec3(0.0f, 1.0f, 0.0f);
        //throw std::runtime_error("Height map points or indices are not initialized.");
    }

    glm::vec3 position(x, 0.0f, z);

    for (size_t i = 0; i < indices.size(); i += 3) {
        glm::vec3 v0 = heightMapPoints[indices[i]] * transformScale;
        glm::vec3 v1 = heightMapPoints[indices[i + 1]] * transformScale;
        glm::vec3 v2 = heightMapPoints[indices[i + 2]] * transformScale;

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
    glm::vec3 u = v1 - v0;
    glm::vec3 v = v2 - v0;
    glm::vec3 w = p - v0;

    double uu = glm::dot(u, u);
    double uv = glm::dot(u, v);
    double vv = glm::dot(v, v);
    double wu = glm::dot(w, u);
    double wv = glm::dot(w, v);

    double denom = uv * uv - uu * vv;
    if (std::abs(denom) < 1e-6) return false; // Degenerate triangle

    double s = (uv * wv - vv * wu) / denom;
    double t = (uv * wu - uu * wv) / denom;

    return (s >= 0.0f) && (t >= 0.0f) && (s + t <= 1.0f);
}

float HeightMapHandler::getHeightAt(float x, float y, float z) const {
    glm::vec3 queryPoint = glm::vec3(x, y, z);

    float height;

    for (size_t i = 0; i < indices.size(); i += 3) {
        const glm::vec3& v0 = heightMapPoints[indices[i]] * transformScale;
        const glm::vec3& v1 = heightMapPoints[indices[i + 1]] * transformScale;
        const glm::vec3& v2 = heightMapPoints[indices[i + 2]] * transformScale;

        // Check if the point is inside the triangle
        if (isPointInTriangle(queryPoint, v0, v1, v2)) {
            glm::vec3 v0ToV1 = v1 - v0;
            glm::vec3 v0ToV2 = v2 - v0;
            glm::vec3 v0ToQuery = queryPoint - v0;

            double d00 = glm::dot(v0ToV1, v0ToV1);
            double d01 = glm::dot(v0ToV1, v0ToV2);
            double d11 = glm::dot(v0ToV2, v0ToV2);
            double d20 = glm::dot(v0ToQuery, v0ToV1);
            double d21 = glm::dot(v0ToQuery, v0ToV2);

            double denom = d00 * d11 - d01 * d01;
            if (denom == 0) return std::numeric_limits<double>::quiet_NaN();

            double v = (d11 * d20 - d01 * d21) / denom;
            double w = (d00 * d21 - d01 * d20) / denom;
            double u = 1.0f - v - w;


            height = u * v0.y + v * v1.y + w * v2.y; // Height interpolation

            /*std::cout << "Query Point: (" << queryPoint.x << ", " << queryPoint.y << ", " << queryPoint.z << ")" << std::endl;
            std::cout << height << std::endl;*/

            return height;
        }
    }

    // Return a default value if the point is not in any triangle
    //std::cerr << "Point (" << x << ", " << z << ") is outside the height map." << std::endl;
    return 0.0;
}