#pragma once
#include <vector>
#include <glm/glm.hpp>
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Delaunay_triangulation_2<K> Delaunay;
typedef K::Point_2 Point2D;

class HeightMapHandler
{
public:
	HeightMapHandler(const std::string& filePath, int maxPoints = -1);

    std::vector<glm::vec3> getHeightMapVector();
    std::vector<glm::vec3> getNormals();
    std::vector<unsigned int> getIndices();
    std::vector<glm::vec3> generateNormals(const std::vector<unsigned int>& indices);
    std::vector<unsigned int> getTriangulationIndices();
    glm::vec3 getClosestNormal(float x, float z, glm::vec3 scale) const;
    bool isPointInTriangle(const glm::vec3& p, const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2) const;
    float getHeightAt(float x, float z) const;
private:
    std::vector<glm::vec3> heightMapPoints;
    std::vector<glm::vec3> normals;
    std::vector<unsigned int> indices;

    std::vector<glm::vec3> loadPointsFromFile(const std::string& filePath, int maxPoints);
};

