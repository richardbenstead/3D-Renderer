#pragma once
#include <algorithm>
#include <vector>
#include "linalg.h"
#include "utils.h"
#include "3dPrimitives.h"

class Render {
  public:
    static void drawImage(auto &canvas, auto const& scene) {
        canvas.fillScreen(0);

        std::vector<Triangle> triangles = scene.getTriangles();

        auto sortByDist = [](Triangle const &t1, Triangle const &t2) { return t1.distFromCamera > t2.distFromCamera; };
        std::sort(triangles.begin(), triangles.end(), sortByDist);

        // Draw the triangles
        for (const Triangle &t : triangles) {
            // the virtual screen -1->1 maps to the LCD screen
            auto toScreen = [&](double x, double y) -> Vec2d {
                return Vec2d{static_cast<int>(canvas.width() * (1 + x) / 2.0),
                             static_cast<int>(canvas.height() * (1 + y) / 2.0)};
            };

            Vec2d p1 = toScreen(t.p1[0], t.p1[1]);
            Vec2d p2 = toScreen(t.p2[0], t.p2[1]);
            Vec2d p3 = toScreen(t.p3[0], t.p3[1]);
            canvas.drawFilledTriangle(p1[0], p1[1], p2[0], p2[1], p3[0], p3[1], t.col);
    }
    }
};

