#pragma once
#include <unordered_set>

class EntityManager {
public:
    EntityManager() : nextEntityId(0) {}

    // Create new entity and return its ID
    int createEntity() {
        int id = nextEntityId++;
        activeEntities.insert(id);
        return id;
    }

    // Check if entity is active
    bool isActive(int id) const {
        return activeEntities.find(id) != activeEntities.end();
    }

    // Entity Destructor
    void destroyEntity(int id) {
        activeEntities.erase(id);
    }

    // Entity Getter
    const std::unordered_set<int>& getEntities() const {
        return activeEntities;
    }

private:
    int nextEntityId;                           // Tracks the next unique ID
    std::unordered_set<int> activeEntities;     // Stores IDs of active entities
};
