#pragma once
#include "3dPrimitives.h"
#include <vector>

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

    void update() { _time++; }

    Camera getCamera() const {
        Vec3f cameraPos{};
        cameraPos[0] = 1.0 * sin(_time * M_PI / 180.0);
        cameraPos[1] = 1.0 * sin(5e8 + 0.77 * _time * M_PI / 180.0);
        cameraPos[2] = 1.0 + 2.0 * cos(0.3 * _time * M_PI / 180.0);

        Vec3f orientation{0,0,1};
        return Camera{cameraPos, orientation};
    }

    std::vector<Triangle> getTriangles() const {
        Camera camera = getCamera();
        std::vector<Triangle> triangles;
        for (Object *o : objects_) {
            o->update(_time);
            std::vector<Triangle> newTri = o->getTriangles(camera);
            triangles.insert(triangles.end(), std::make_move_iterator(newTri.begin()),
                             std::make_move_iterator(newTri.end()));
        }
        return triangles;
    }

    std::vector<Object *> objects_;
    uint32_t _time = 0;
};
