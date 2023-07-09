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
            std::fill(ptr, ptr + width(), std::lerp(col1, col2, float(i) / height()));
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
    inline void drawTexturedHLine(int x, int x2, int y, BaryCentric const &bc, TexShader const &texShader) {
        if ((y >= 0) & (y < height()) & (x < width()) & (x2 >= 0)) {
            [[likely]];
            x = std::max(0, x);
            x2 = std::min(width(), x2);

            class Iter {
                public:
                    Iter(colRGB* c) : ptr(c) {}
                    inline void operator++() {++ptr;}
                    inline void setVal(colRGB c) {
                        *ptr=c;
                    }
                private:
                    colRGB* ptr;
            };


            Iter iter{&_image.image[x + y * height()]};
            texShader.drawTexturedHLine(x, x2, y, bc, iter);
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
            [[unlikely]];
            drawTexturedHLine(std::min({v0.imageCoord.x(), v1.imageCoord.x(), v2.imageCoord.x()}),
                                  std::max({v0.imageCoord.x(), v1.imageCoord.x(), v2.imageCoord.x()}),
                                  v0.imageCoord.y(), bc, texShader);
            return;
        }

        // Triangles are drawn in two parts, with a horizontal line that intersects point v1 (v0 at top and v2 at bottom)
        int16_t m2 = ((v2.imageCoord.x() - v0.imageCoord.x()) * SCALE) / v2v0y;
        int16_t curx1, curx2, m1, b1 = 0;
        int16_t scanlineY = 0;

        if (v1v0y) {
            [[likely]];
            m1 = ((v1.imageCoord.x() - v0.imageCoord.x()) * SCALE) / v1v0y;
            int16_t x1Offs = 0, x2Offs = 0;
            if ((m2 > m1)) {
                for (; scanlineY < v1v0y; scanlineY++, x1Offs += m1, x2Offs += m2) {
                    curx1 = v0.imageCoord.x() + x1Offs / SCALE;
                    curx2 = v0.imageCoord.x() + x2Offs / SCALE;
                    drawTexturedHLine(curx1, curx2, v0.imageCoord.y() + scanlineY, bc, texShader);
                }
            } else {
                for (; scanlineY < v1v0y; scanlineY++, x1Offs += m1, x2Offs += m2) {
                    curx1 = v0.imageCoord.x() + x1Offs / SCALE;
                    curx2 = v0.imageCoord.x() + x2Offs / SCALE;
                    drawTexturedHLine(curx2, curx1, v0.imageCoord.y() + scanlineY, bc, texShader);
                }
            }
        }

        if (v2v1y) {
            [[likely]];
            m1 = (v2.imageCoord.x() - v1.imageCoord.x()) * SCALE / v2v1y;
            b1 = (v1.imageCoord.x() - v0.imageCoord.x()) - (m1 * v1v0y) / SCALE;
            if ((m2 * scanlineY) > ((m1 * scanlineY) + b1 * SCALE)) {
                for (; scanlineY <= v2v0y; scanlineY++) {
                    curx1 = v0.imageCoord.x() + (m1 * scanlineY) / SCALE + b1;
                    curx2 = v0.imageCoord.x() + (m2 * scanlineY) / SCALE;
                    drawTexturedHLine(curx1, curx2, v0.imageCoord.y() + scanlineY, bc, texShader);
                }
            } else {
                for (; scanlineY <= v2v0y; scanlineY++) {
                    curx1 = v0.imageCoord.x() + (m1 * scanlineY) / SCALE + b1;
                    curx2 = v0.imageCoord.x() + (m2 * scanlineY) / SCALE;
                    drawTexturedHLine(curx2, curx1, v0.imageCoord.y() + scanlineY, bc, texShader);
                }
            }
        }
    }
    I _image;
};
