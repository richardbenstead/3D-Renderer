#pragma once
#include "3dPrimitives.h"
#include <vector>

struct SceneTileArray {
    // TODO: set object size and random movement from here
    SceneTileArray() {
        int size = 5;
        for (float x = -size; x <= size; x += 0.5) {
            for (float y = -size; y <= size; y += 0.5) {
                objects_.push_back(new Tile({x, y, 8}));
            }
        }
    }

    void update() {
        _time++;
        if (_time % 50 == 0) {
            for (Object *o : objects_) {
                static_cast<Tile *>(o)->setTarget(rand() % 2);
            }
        }
    }

    Vec3f getCamera() const {
        Vec3f camera{};
        camera[0] = 0.5 * sin(_time * M_PI / 180.0);
        camera[1] = 0.5 * sin(5e8 + 0.77 * _time * M_PI / 180.0);
        camera[2] = 1.0 + 1.0 * cos(0.3 * _time * M_PI / 180.0);
        return camera;
    }

    std::vector<Triangle> getTriangles() const {
        Vec3f camera = getCamera();
        std::vector<Triangle> triangles;
        for (Object *o : objects_) {
            o->update(_time);
            std::vector<Triangle> newTri = o->getTriangles(camera);
            triangles.insert(triangles.end(), std::make_move_iterator(newTri.begin()),
                             std::make_move_iterator(newTri.end()));
        }
        return triangles;
    }

    void step() {}

    std::vector<Object *> objects_;
    uint32_t _time = 0;
    bool _pause{false};
};
