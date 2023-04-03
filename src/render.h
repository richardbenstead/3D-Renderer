#pragma once
#include "3dPrimitives.h"
#include "types.h"

class Render {
  public:
    static void drawImage(auto &canvas, auto const &scene) {
        std::vector<Triangle> triangles = scene.getTriangles();
        std::sort(triangles.begin(), triangles.end());

        // Draw the triangles
        // canvas.fillScreen(colRGB(0,0,0));
        canvas.fillFade(0, canvas.height() / 2, colRGB(0.8, 0.8, 0.9), colRGB(0.3, 0.3, 0.5));
        canvas.fillFade(canvas.height() / 2, canvas.height(), colRGB(0.2, 0.2, 0.2), colRGB(0.6, 0.6, 0.6));
        for (const Triangle &tri : triangles) {
            // the virtual screen -1->1 maps to the image
            auto toScreen = [&](Vec2f const &point, Vec2f const &texCoord) -> RenderVertex {
                return {Vec2i{canvas.width() * (1 + point[0]) / 2.0, canvas.height() * (1 + point[1]) / 2.0}, texCoord};
            };

            RenderVertex rv1 = toScreen(tri.p1, tri.texCoords.col(0));
            RenderVertex rv2 = toScreen(tri.p2, tri.texCoords.col(1));
            RenderVertex rv3 = toScreen(tri.p3, tri.texCoords.col(2));
            canvas.drawFilledTriangle(rv1, rv2, rv3, tri.vertexDist, tri.shader);
        }
    }
};
