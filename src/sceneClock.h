#pragma once
#include "3dPrimitives.h"
#include <vector>
#include <chrono>
#include <ctime>

struct SceneClock {
    SceneClock() {
        auto addCube = [&](Vec3d const &centre) {
            Cube *cube = new Cube(centre);
            auto randAngle = []() { return static_cast<double>(rand() % 360); };
            cube->_targetRotation = {randAngle(), randAngle(), randAngle()};
            objects_.push_back(cube);
        };
        addCube(Vec3d{-5, 0, 6});
        addCube(Vec3d{-3, 0, 6});
        addCube(Vec3d{-1, 0, 6});
        addCube(Vec3d{1, 0, 6});
        addCube(Vec3d{3, 0, 6});
        addCube(Vec3d{5, 0, 6});
    }

    void update() {
        if (!_pause) {
            _time++;
            for (Object *o : objects_) {
                o->update(_time);
            }
        }

        auto now = std::chrono::system_clock::now();
        std::time_t current_time = std::chrono::system_clock::to_time_t(now);

        // Convert the time to a struct tm
        struct std::tm* time_info = std::localtime(&current_time);

        // Copy the hour, minute, and second components to integer variables
        int hour = time_info->tm_hour;
        int minute = time_info->tm_min;
        int second = time_info->tm_sec;
        static_cast<Cube *>(objects_[0])->setTargetTex(hour/10);
        static_cast<Cube *>(objects_[1])->setTargetTex(hour%10);
        static_cast<Cube *>(objects_[2])->setTargetTex(minute/10);
        static_cast<Cube *>(objects_[3])->setTargetTex(minute%10);
        static_cast<Cube *>(objects_[4])->setTargetTex(second/10);
        static_cast<Cube *>(objects_[5])->setTargetTex(second%10);
    }

    void step() {
    }

    Vec3d getCamera() const {
        Vec3d camera{};
        camera[0] = 0.2 * sin(_time * M_PI / 180.0);
        camera[1] = 0.2 * sin(5e8 + 0.77 * _time * M_PI / 180.0);
        camera[2] = 1.0 + 0.5 * cos(0.3 * _time * M_PI / 180.0);
        return camera;
    }

    std::vector<Triangle> getTriangles() const {
        Vec3d camera = getCamera();
        std::vector<Triangle> triangles;
        for (Object *o : objects_) {
            std::vector<Triangle> newTri = o->getTriangles(camera);
            triangles.insert(triangles.end(), newTri.begin(), newTri.end());
        }
        return triangles;
    }

    std::vector<Object *> objects_;
    uint32_t _time = 0;
    bool _pause{false};
};
