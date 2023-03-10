#pragma once
#include <vector>
#include "3dPrimitives.h"

struct SceneCubeArray {
    SceneCubeArray() {
        objects_.push_back(new Cube({-3, -3, 6}));
        objects_.push_back(new Cube({0, -3, 6}));
        objects_.push_back(new Cube({3, -3, 6}));
        objects_.push_back(new Cube({-3, 0, 6}));
        objects_.push_back(new Cube({0, 0, 6}));
        objects_.push_back(new Cube({3, 0, 6}));
        objects_.push_back(new Cube({-3, 3, 6}));
        objects_.push_back(new Cube({0, 3, 6}));
        objects_.push_back(new Cube({3, 3, 6}));
    }

    void update() {
        _time++;
    }

    Vec3d getCamera() const {
        Vec3d camera{};
        camera[0] = 4.0 * sin(_time * M_PI / 180.0);
        camera[1] = 4.0 * sin(5e8 + 0.77 * _time * M_PI / 180.0);
        camera[2] = 2.0 + 2.0 * cos(0.3 * _time * M_PI / 180.0);
        return camera;
    }

    std::vector<Triangle> getTriangles() const {
        Vec3d camera = getCamera();
        std::vector<Triangle> triangles;
        for (Object *o : objects_) {
            o->update(_time);
            std::vector<Triangle> newTri = o->getTriangles(camera);
            triangles.insert(triangles.end(), newTri.begin(), newTri.end());
        }
        return triangles;
    }

    std::vector<Object *> objects_;
    uint32_t _time = 0;
};
