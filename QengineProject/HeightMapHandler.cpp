#include "HeightMapHandler.h"
#include <glm/vec3.hpp>
#include <unordered_map>
#include <glm/glm.hpp>
#include <functional>

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
        return v1.x == v2.x && v1.y == v2.y && v1.z == v2.z;
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
    float ax = p1.x - point.x;
    float az = p1.z - point.z;
    float bx = p2.x - point.x;
    float bz = p2.z - point.z;
    float cx = p3.x - point.x;
    float cz = p3.z - point.z;

    float A = ax * ax + az * az;
    float B = bx * bx + bz * bz;
    float C = cx * cx + cz * cz;

    float det = ax * (bz * C - B * cz) - az * (bx * C - B * cx) + A * (bx * cz - bz * cx);

    return det > 0; // Inside the circumcircle if determinant > 0
}

void insertPoint(const glm::vec3& point, std::vector<Triangle>& triangles) {
    std::vector<Triangle> badTriangles;

    // Step 1: Find all triangles whose circumcircle contains the point
    for (const auto& triangle : triangles) {
        if (triangle.isPointInCircumcircle(point)) {
            badTriangles.push_back(triangle);
        }
    }

    // Step 2: Find the boundary edges of the polygon formed by badTriangles
    std::vector<Edge> boundaryEdges;
    for (const auto& triangle : badTriangles) {
        // Add all edges of the triangle
        std::vector<Edge> edges = {
            {triangle.p1, triangle.p2},
            {triangle.p2, triangle.p3},
            {triangle.p3, triangle.p1}
        };

        for (const auto& edge : edges) {
            auto it = std::find(boundaryEdges.begin(), boundaryEdges.end(), edge);
            if (it != boundaryEdges.end()) {
                // Remove if the edge is shared
                boundaryEdges.erase(it);
            }
            else {
                // Add if the edge is not shared
                boundaryEdges.push_back(edge);
            }
        }
    }

    // Step 3: Remove bad triangles
    for (const auto& triangle : badTriangles) {
        triangles.erase(std::remove(triangles.begin(), triangles.end(), triangle), triangles.end());
    }

    // Step 4: Create new triangles by connecting the point to the boundary edges
    for (const auto& edge : boundaryEdges) {
        triangles.emplace_back(Triangle{ edge.p1, edge.p2, point });
    }
}

Triangle createSuperTriangle(const std::vector<glm::vec3>& points) {
    float minX = std::numeric_limits<float>::max(), maxX = std::numeric_limits<float>::lowest();
    float minZ = std::numeric_limits<float>::max(), maxZ = std::numeric_limits<float>::lowest();

    for (const auto& point : points) {
        minX = std::min(minX, point.x);
        maxX = std::max(maxX, point.x);
        minZ = std::min(minZ, point.z);
        maxZ = std::max(maxZ, point.z);
    }

    // Super-triangle buffer size
    float buffer = 10.0f;

    // Calculate the center of the bounding box
    float centerX = (minX + maxX) / 2;
    float centerZ = (minZ + maxZ) / 2;

    // Scale the size of the bounding box to create a super-triangle that is much larger
    float dx = maxX - minX;
    float dz = maxZ - minZ;
    float deltaMax = std::max(dx, dz);

    // Create the super-triangle with points well outside the bounding box
    glm::vec3 p1(centerX - deltaMax * buffer, 0.0f, centerZ - deltaMax);  // Left-bottom point of super triangle
    glm::vec3 p2(centerX + deltaMax * buffer, 0.0f, centerZ - deltaMax);  // Right-bottom point of super triangle
    glm::vec3 p3(centerX, 0, centerZ + deltaMax);                         // Top point of super triangle

    return Triangle{ p1, p2, p3 };
}

void finalizeTriangles(std::vector<Triangle>& triangles, const Triangle& superTriangle) {
    const glm::vec3& stP1 = superTriangle.p1;
    const glm::vec3& stP2 = superTriangle.p2;
    const glm::vec3& stP3 = superTriangle.p3;

    for (const auto& triangle : triangles) {
        bool shouldRemove = (triangle.p1 == stP1 || triangle.p1 == stP2 || triangle.p1 == stP3) ||
            (triangle.p2 == stP1 || triangle.p2 == stP2 || triangle.p2 == stP3) ||
            (triangle.p3 == stP1 || triangle.p3 == stP2 || triangle.p3 == stP3);
    }

    triangles.erase(std::remove_if(triangles.begin(), triangles.end(),
        [&](const Triangle& triangle) {
            return (triangle.p1 == stP1 || triangle.p1 == stP2 || triangle.p1 == stP3) ||
                (triangle.p2 == stP1 || triangle.p2 == stP2 || triangle.p2 == stP3) ||
                (triangle.p3 == stP1 || triangle.p3 == stP2 || triangle.p3 == stP3);
        }), triangles.end());
}

void testSuperTriangle(const std::vector<glm::vec3>& points) {
    // Create the super-triangle
    Triangle superTriangle = createSuperTriangle(points);

    // Print out the points of the super-triangle for verification
    std::cout << "Super-Triangle Points:" << std::endl;
    std::cout << "p1: (" << superTriangle.p1.x << ", " << superTriangle.p1.y << ", " << superTriangle.p1.z << ")" << std::endl;
    std::cout << "p2: (" << superTriangle.p2.x << ", " << superTriangle.p2.y << ", " << superTriangle.p2.z << ")" << std::endl;
    std::cout << "p3: (" << superTriangle.p3.x << ", " << superTriangle.p3.y << ", " << superTriangle.p3.z << ")" << std::endl;
}

bool comparePoints(const glm::vec3& p1, const glm::vec3& p2) {
    return p1.x < p2.x || (p1.x == p2.x && p1.z < p2.z);
}

float crossProduct(const glm::vec3& o, const glm::vec3& a, const glm::vec3& b) {
    return (a.x - o.x) * (b.z - o.z) - (a.z - o.z) * (b.x - o.x);
}

std::vector<glm::vec3> computeConvexHull(std::vector<glm::vec3>& points) {
    if (points.size() < 3) return points;
    std::vector<glm::vec3> hull;
    std::sort(points.begin(), points.end(), comparePoints);

    // Build the lower hull
    for (const auto& point : points) {
        while (hull.size() >= 2 && crossProduct(hull[hull.size() - 2], hull.back(), point) <= 0) {
            hull.pop_back();
        }
        hull.push_back(point);
    }

    // Build the upper hull
    size_t lowerHullSize = hull.size();
    for (size_t i = points.size() - 1; i > 0; --i) {
        const auto& point = points[i];
        while (hull.size() > lowerHullSize && crossProduct(hull[hull.size() - 2], hull.back(), point) <= 0) {
            hull.pop_back();
        }
        hull.push_back(point);
    }

    // Remove last point if it's a duplicate
    if (hull.size() > 1 && hull.back() == hull.front()) {
        hull.pop_back();
    }

    return hull;
}

int getOrAddPointIndex(const glm::vec3& point,
    std::vector<glm::vec3>& uniquePoints,
    std::unordered_map<glm::vec3, int, Vec3Hash, Vec3Equal>& pointToIndex) {
    auto it = pointToIndex.find(point);
    if (it != pointToIndex.end()) {
        return it->second; // Return the existing index if the point is already in the map
    }

    int newIndex = static_cast<int>(uniquePoints.size());
    uniquePoints.push_back(point); // Add the point to the unique points list
    pointToIndex[point] = newIndex; // Map the point to its index
    return newIndex;
}

std::tuple<std::vector<glm::vec3>, std::vector<unsigned int>> trianglesToIndices(const std::vector<Triangle>& triangles) {
    std::vector<glm::vec3> uniquePoints;
    std::unordered_map<glm::vec3, int, Vec3Hash, Vec3Equal> pointToIndex;
    std::vector<unsigned int> indices;

    // Process each triangle
    for (const auto& triangle : triangles) {
        int i1 = getOrAddPointIndex(triangle.p1, uniquePoints, pointToIndex);
        int i2 = getOrAddPointIndex(triangle.p2, uniquePoints, pointToIndex);
        int i3 = getOrAddPointIndex(triangle.p3, uniquePoints, pointToIndex);

        indices.push_back(i1);
        indices.push_back(i2);
        indices.push_back(i3);
    }

    return { uniquePoints, indices }; // Return a tuple containing the points and indices
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

    while (std::getline(file, line) && (maxPoints < 0 || pointCount < maxPoints)) {
        std::stringstream ss(line);
        std::string xStr, yStr, zStr;

        if (std::getline(ss, xStr, ',') &&
            std::getline(ss, yStr, ',') &&
            std::getline(ss, zStr, ',')) {

            float x = std::stof(xStr) - 607200;
            float y = std::stof(yStr) - 6750618;
            float z = std::stof(zStr) - 270;
            
            /*float x = std::stof(xStr);
            float y = std::stof(yStr);
            float z = std::stof(zStr);*/

            points.emplace_back(x, z, y); // Swap y/z for convenience
            pointCount++;
        } else {
            std::cerr << "Error parsing line: " << line << std::endl;
        }
    }
    std::cout << "number of points: " << pointCount << std::endl;

    std::vector<Triangle> triangles;

    Triangle superTriangle = createSuperTriangle(points);
    triangles.push_back(superTriangle);

    testSuperTriangle(points);
    
    for (const auto& point : points) {
        insertPoint(point, triangles);
    }
    finalizeTriangles(triangles, superTriangle);

    auto result = trianglesToIndices(triangles);
    points = std::get<0>(result);
    indices = std::get<1>(result);

    std::cout << "Done" << std::endl;

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
std::vector<unsigned int> HeightMapHandler::setIndices(std::vector<unsigned int> newIndices)
{
    indices = newIndices;
    return indices;
}
