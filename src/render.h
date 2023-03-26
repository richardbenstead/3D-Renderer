#pragma once
#include "3dPrimitives.h"
#include "MathUtil.h"
#include "utils.h"
#include <algorithm>
#include <vector>

class Render {
  public:
    static void drawImage(auto &canvas, auto const &scene) {
        canvas.fillScreen(0);

        std::vector<Triangle> triangles = scene.getTriangles();

        auto sortByDist = [](Triangle const &t1, Triangle const &t2) { return t1.vertexDist[2] > t2.vertexDist[2]; };
        std::sort(triangles.begin(), triangles.end(), sortByDist);

        // Draw the triangles
        for (const Triangle &t : triangles) {
            // the virtual screen -1->1 maps to the image
            auto toScreen = [&](Vec2d const &p, Vec2d const &t) -> RenderVertex {
                return {Vec2d{canvas.width() * (1 + p[0]) / 2.0, canvas.height() * (1 + p[1]) / 2.0}, t};
            };

            RenderVertex rv1 = toScreen(t.p1, t.texCoords.col(0));
            RenderVertex rv2 = toScreen(t.p2, t.texCoords.col(1));
            RenderVertex rv3 = toScreen(t.p3, t.texCoords.col(2));

            canvas.drawFilledTriangle(rv1, rv2, rv3, t.vertexDist, t.ts);
        }
    }
};
