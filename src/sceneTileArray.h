#pragma once
#include "3dPrimitives.h"
#include "font.h"
#include <vector>

struct SceneTileArray {
    using TileType = Tile<BasicShader>;
    // TODO: set object size and random movement from here
    static constexpr int XSIZE=27;
    static constexpr int YSIZE=16;
    SceneTileArray() {
        float tileSize = 0.45;
        float x= -tileSize * XSIZE / 2;
        float yStart = -tileSize * YSIZE / 2;
        for (int xi=0; xi < XSIZE; ++xi) {
            float y = yStart;
            for (int yi=0; yi < YSIZE; ++yi) {
                objects_.push_back(new TileType({x, y, 10}));
                y += tileSize;
            }
            x += tileSize;
        }
    }

    void setTile(int x, int y, int val) {
        int idx = x * YSIZE+y;
        static_cast<TileType *>(objects_[idx])->setTarget(val);
    }

    void drawNum(int x0, int y0, int val) {
        for (int y=0; y<5; ++y) {
            int rowVal = bmNumThin[val][y];
            for (int x=0; x<3; ++x) {
                int pixVal = (rowVal >> (2-x)) & 0b1;
                setTile(x0+x, y0+y, pixVal);
            }
        }
    }

    void update() {
        auto now = std::chrono::system_clock::now();
        std::time_t current_time = std::chrono::system_clock::to_time_t(now);

        // Convert the time to a struct tm
        struct std::tm *time_info = std::localtime(&current_time);

        // Copy the hour, minute, and second components to integer variables
        int hour = time_info->tm_hour;
        int minute = time_info->tm_min;
        int second = time_info->tm_sec;

        for(int x=0; x< XSIZE; ++x) {
            for(int y=0; y< YSIZE; ++y) {
                setTile(x,y,0);
            }
        }

        drawNum(1,1,hour/10);
        drawNum(5,1,hour%10);

        drawNum(10,1,minute/10);
        drawNum(14,1,minute%10);

        drawNum(19,1,second/10);
        drawNum(23,1,second%10);
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
