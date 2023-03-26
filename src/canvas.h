#pragma once
#include <algorithm>

template <typename I> class Canvas {
  public:
    Canvas() {}
    void fillScreen(uint16_t col) {
        auto [r, g, b] = fromColor565(col);
        Pixel pix(r, g, b);

        Pixel *ptr = &_image.image[0];
        for (int i = 0; i < width(); ++i) {
            *ptr = pix;
            ptr++;
        }
        for (int i = 1; i < height(); ++i) {
            memcpy(&_image.image[height() * i], &_image.image[0], sizeof(Pixel) * width());
        }
    }

    int width() { return _image.GRID_SIZE; }
    int height() { return _image.GRID_SIZE; }

    inline void drawHLine(int x, int x2, int y, uint16_t color) {
        swapIf(x2 < x, x2, x);

        if ((y >= 0) && (y < height()) && (x < width())) {
            x = std::max(0, x);
            x2 = std::min(width(), x2);
            int w = x2 - x;

            auto [r, g, b] = fromColor565(color);
            uint32_t ind = x + y * height();
            Pixel *ptr = &_image.image[ind];
            Pixel rgb(r, g, b);
            while (w-- > 0) {
                *ptr = rgb;
                ++ptr;
            }
        }
    }

    struct BaryCentric {
        BaryCentric(RenderVertex const &b, RenderVertex const &c, RenderVertex const &a, Vec3d const &vertexDist)
            : _vertexDist(vertexDist) {
            _a = a.imageCoord;
            v0 = b.imageCoord - a.imageCoord;
            v1 = c.imageCoord - a.imageCoord;
            d00 = v0.dot(v0);
            d01 = v0.dot(v1);
            d11 = v1.dot(v1);
            denom = d00 * d11 - d01 * d01;

            texCoords.col(0) = a.textureCoord;
            texCoords.col(1) = b.textureCoord;
            texCoords.col(2) = c.textureCoord;
        }

        std::pair<Vec2d, double> ConvertImageToTextureDist(Vec2d const &p) const {
            Vec2d v2 = p - _a;
            double d20 = v2.dot(v0);
            double d21 = v2.dot(v1);

            Vec3d bWeights;
            bWeights[0] = (d11 * d20 - d01 * d21) / denom;
            bWeights[1] = (d00 * d21 - d01 * d20) / denom;
            bWeights[2] = 1.0f - bWeights[0] - bWeights[1];
            return {texCoords * bWeights, bWeights.dot(_vertexDist)};
        }
        Vec2d _a;
        double d00, d01, d11, denom;
        Vec3d _vertexDist; // dist from camera
        Vec2d v0, v1;
        Eigen::Matrix<double, 2, 3> texCoords; // mapping from vertices to texture coordinates
    };

    inline void drawFastTexturedHLine(int x, int x2, int y, BaryCentric const &bc, TextureShader const &texShader) {
        swapIf(x2 < x, x2, x);

        if ((y >= 0) && (y < height()) && (x < width())) {
            x = std::max(0, x);
            x2 = std::min(width(), x2);

            Pixel *ptr = &_image.image[x + y * height()];
            for (; x <= x2; ++x, ++ptr) {
                auto [tex, dist] = bc.ConvertImageToTextureDist({x, y});
                *ptr = texShader.GetValue(tex[0], tex[1], dist);
            }
        }
    }

    inline void drawFilledTriangle(RenderVertex &v0, RenderVertex &v1, RenderVertex &v2, Vec3d const &vertexDist,
                                   TextureShader const &texShader) {
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

        int16_t m2 = ((v2.imageCoord.x() - v0.imageCoord.x()) * SCALE) / v2v0y;
        // We draw any triangle as two triangles, joined by a horizontal line that intersects point v1 (v0 at top and v2
        // at bottom)
        int16_t curx1, curx2, m1, b1 = 0;
        int16_t scanlineY = 0;

        if (v1v0y) {
            m1 = ((v1.imageCoord.x() - v0.imageCoord.x()) * SCALE) / (1 + v1v0y);
            for (; scanlineY < v1v0y; scanlineY++) {
                curx1 = v0.imageCoord.x() + (m1 * scanlineY) / SCALE + b1;
                curx2 = v0.imageCoord.x() + (m2 * scanlineY) / SCALE;
                drawFastTexturedHLine(curx1, curx2, v0.imageCoord.y() + scanlineY, bc, texShader);
            }
        }

        if (v2v1y) {
            m1 = (v2.imageCoord.x() - v1.imageCoord.x()) * SCALE / (1 + v2v1y);
            b1 = (v1.imageCoord.x() - v0.imageCoord.x()) - (m1 * v1v0y) / SCALE;
            for (; scanlineY <= v2v0y; scanlineY++) {
                curx1 = v0.imageCoord.x() + (m1 * scanlineY) / SCALE + b1;
                curx2 = v0.imageCoord.x() + (m2 * scanlineY) / SCALE;
                drawFastTexturedHLine(curx1, curx2, v0.imageCoord.y() + scanlineY, bc, texShader);
            }
        }
    }
    I _image;
};
