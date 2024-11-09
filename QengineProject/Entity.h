#pragma once
class Entity {
public:
    Entity() : id(-1) {}
    Entity(int id) : id(id) {}

    int getId() const { return id; }

private:
    int id;
};