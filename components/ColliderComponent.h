#pragma once
#include <glm/glm.hpp>

enum class ColliderType {
	SPHERE,
	BOX,
	CAPSULE
};

struct ColliderComponent
{
	ColliderType type;
	glm::vec3 offset;
	glm::vec3 dimensions;
	bool isTrigger;
	bool isStatic;

	ColliderComponent()
		: type(ColliderType::SPHERE), offset(0.0f), dimensions(1.0f), isTrigger(false), isStatic(false) {}

	ColliderComponent(ColliderType type, glm::vec3 offset = glm::vec3(0.0f), glm::vec3 dimensions = glm::vec3(1.0f), 
		bool isTrigger = false, bool isStatic = false)
		: type(type), offset(offset), dimensions(dimensions), isTrigger(isTrigger), isStatic(isStatic) {}
};

