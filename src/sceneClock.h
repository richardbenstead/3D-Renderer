#pragma once
#include <vector>
#include "3dPrimitives.h"

struct SceneClock {
    SceneClock() {
        auto addCube = [&](Vec3d const& centre) {
            Cube* cube = new Cube(centre);
            cube->_targetRotation = {rand() % 360, rand() % 360, rand() % 360};
            objects_.push_back(cube);
        };
        addCube(Vec3d{-2, 0, 6});
        addCube(Vec3d{0, 0, 6});
        addCube(Vec3d{2, 0, 6});
        addCube(Vec3d{-2, 2, 6});
        addCube(Vec3d{0, 2, 6});
        addCube(Vec3d{2, 2, 6});
    }

    void update() {
        _time++;
        // std::cout << _time << std::endl;
        if (_time % 200 == 0)
        {
            for (Object *o : objects_) {
                static_cast<Cube*>(o)->_targetRotation = {rand() % 360, rand() % 360, rand() % 360};
            }
        }
    }

    Vec3d getCamera() const {
        Vec3d camera{};
        camera[0] = 2.0 * sin(_time * M_PI / 180.0);
        camera[1] = 2.0 * sin(5e8 + 0.77 * _time * M_PI / 180.0);
        camera[2] = 1.0 + 1.0 * cos(0.3 * _time * M_PI / 180.0);
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
