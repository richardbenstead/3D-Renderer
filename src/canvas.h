#pragma once
#include "baryCentric.h"
#include "colRGB.h"
#include <algorithm>

template <typename T> void swapIf(const bool pred, T &v1, T &v2) {
    if (pred) {
        std::swap(v1, v2);
    }
}

template <typename I> class Canvas {
  public:
    Canvas() {}
    void fillScreen(colRGB const &col) {
        colRGB *ptr = &_image.image[0];
        std::fill(ptr, ptr + width() * height(), col);
    }

    void fillFade(int startHeight, int endHeight, colRGB const &col1, colRGB const &col2) {
        colRGB *ptr = &_image.image[startHeight * width()];
        for (int i = startHeight; i < endHeight; ++i) {
            std::fill(ptr, ptr + width(), col1.lerp(col2, float(i) / height()));
            ptr += width();
        }
    }

    int width() { return _image.GRID_SIZE; }
    int height() { return _image.GRID_SIZE; }

    inline void drawHLine(int x, int x2, int y, colRGB const &color) {
        swapIf(x2 < x, x2, x);

        if ((y >= 0) && (y < height()) && (x < width())) {
            x = std::max(0, x);
            x2 = std::min(width(), x2);
            int w = x2 - x;

            uint32_t ind = x + y * height();
            colRGB *ptr = &_image.image[ind];
            while (w-- > 0) {
                *ptr = color;
                ++ptr;
            }
        }
    }

    template <typename TexShader>
    inline void drawFastTexturedHLine(int x, int x2, int y, BaryCentric const &bc, TexShader const &texShader) {
        swapIf(x2 < x, x2, x);

        if ((y >= 0) && (y < height()) && (x < width())) {
            [[likely]];
            x = std::max(0, x);
            x2 = std::min(width(), x2);

            colRGB *ptr = &_image.image[x + y * height()];
            Vec2f tex1, tex2;
            float dist1, dist2;
            std::tie(tex1, dist1) = bc.ConvertImageToTextureDist({x, y});
            std::tie(tex2, dist2) = bc.ConvertImageToTextureDist({x2, y});
            Vec2f mTex = (tex2-tex1) / static_cast<float>(x2-x);
            float mDist = (dist2-dist1) / static_cast<float>(x2-x);
            int i=0;
            for (; x <= x2; ++x, ++ptr, ++i) {
                Vec2f tex = tex1 + i * mTex;
                *ptr = texShader.GetValue(tex[0], tex[1], dist1+i*mDist);
            }
        }
    }

    template <typename TexShader>
    inline void drawFilledTriangle(RenderVertex &v0, RenderVertex &v1, RenderVertex &v2, Vec3f const &vertexDist,
                                   TexShader const &texShader) {
        // get the barycentric mapping function
        BaryCentric bc(v0, v1, v2, vertexDist);

        // Sort the y-coordinates in ascending order
        swapIf(v0.imageCoord.y() > v1.imageCoord.y(), v0, v1);
        swapIf(v1.imageCoord.y() > v2.imageCoord.y(), v1, v2);
        swapIf(v0.imageCoord.y() > v1.imageCoord.y(), v0, v1);

        // Calculate the slope and y-intercept for each line
        int16_t const v1v0y = v1.imageCoord.y() - v0.imageCoord.y();
        int16_t const v2v0y = v2.imageCoord.y() - v0.imageCoord.y();
        int16_t const v2v1y = v2.imageCoord.y() - v1.imageCoord.y();
        constexpr int16_t SCALE = 256;

        if (!v2v0y) {
            drawFastTexturedHLine(std::min({v0.imageCoord.x(), v1.imageCoord.x(), v2.imageCoord.x()}),
                                  std::max({v0.imageCoord.x(), v1.imageCoord.x(), v2.imageCoord.x()}),
                                  v0.imageCoord.y(), bc, texShader);
            return;
        }

        // We draw any triangle in two parts, with a horizontal line that intersects point v1 (v0 at top and v2 at
        // bottom)
        int16_t m2 = ((v2.imageCoord.x() - v0.imageCoord.x()) * SCALE) / v2v0y;
        int16_t curx1, curx2, m1, b1 = 0;
        int16_t scanlineY = 0;

        /*
        std::cout << "v0.x: " << v0.imageCoord.x() << " v1.x: " << v1.imageCoord.x() << " v2.x: " << v2.imageCoord.x()
        << std::endl; std::cout << "v0.y: " << v0.imageCoord.y() << " v1.y: " << v1.imageCoord.y() << " v2.y: " <<
        v2.imageCoord.y() << std::endl; std::cout << "v0->v2 m: " << m2 << std::endl;
        */
        if (v1v0y) {
            m1 = ((v1.imageCoord.x() - v0.imageCoord.x()) * SCALE) / v1v0y;
            // std::cout << "v0->v1 m: " << m1 << std::endl;
            for (; scanlineY < v1v0y; scanlineY++) {
                curx1 = v0.imageCoord.x() + (m1 * scanlineY) / SCALE + b1;
                curx2 = v0.imageCoord.x() + (m2 * scanlineY) / SCALE;
                drawFastTexturedHLine(curx1, curx2, v0.imageCoord.y() + scanlineY, bc, texShader);
            }
        }

        if (v2v1y) {
            m1 = (v2.imageCoord.x() - v1.imageCoord.x()) * SCALE / v2v1y;
            b1 = (v1.imageCoord.x() - v0.imageCoord.x()) - (m1 * v1v0y) / SCALE;
            // std::cout << "v1->v2 m: " << m1 << std::endl;
            for (; scanlineY <= v2v0y; scanlineY++) {
                curx1 = v0.imageCoord.x() + (m1 * scanlineY) / SCALE + b1;
                curx2 = v0.imageCoord.x() + (m2 * scanlineY) / SCALE;
                drawFastTexturedHLine(curx1, curx2, v0.imageCoord.y() + scanlineY, bc, texShader);
            }
        }
    }
    I _image;
};
