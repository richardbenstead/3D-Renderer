#pragma once
#include <algorithm>

template <typename I> class Canvas {
  public:
    Canvas() {}
    void fillScreen(uint16_t col) {
        auto [r, g, b] = fromColor565(col);
        Pixel pix(r,g,b);

        Pixel* ptr = &_image.image[0];
        for (int i = 0; i < width(); ++i) {
            *ptr = pix;
            ptr++;
        }
        for (int i = 1; i < height(); ++i ) {
            memcpy(&_image.image[height() * i], &_image.image[0], sizeof(Pixel) * width());
        }
    }

    int width() { return _image.GRID_SIZE; }
    int height() { return _image.GRID_SIZE; }

    inline void drawFastHLine(int x, int y, int w, uint16_t color) {
        if ((y < 0) || (y >= height()) || (x >= width()))
            return;
        if (x < 0) {
            w += x;
            x = 0;
        }
        if (x + w > width()) {
            w = width() - x;
        }

        auto [r, g, b] = fromColor565(color);
        uint32_t ind = x + y * height();
        Pixel* ptr = &_image.image[ind];
        Pixel rgb(r,g,b);
        while (w-- > 0) {
            *ptr = rgb;
            ++ptr;
        }
    }

    inline void drawHLine(int a, int b, int y, uint16_t color) {
        if (b > a) {
            drawFastHLine(a, y, b - a, color);
        } else {
            drawFastHLine(b, y, a - b, color);
        }
    }
    inline void drawFilledTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2,
                                   uint16_t color) {
        // Sort the y-coordinates in ascending order
        if (y0 > y1) {
            std::swap(y0, y1);
            std::swap(x0, x1);
        }
        if (y1 > y2) {
            std::swap(y1, y2);
            std::swap(x1, x2);
        }
        if (y0 > y1) {
            std::swap(y0, y1);
            std::swap(x0, x1);
        }

        // Calculate the slope and y-intercept for each line
        int16_t m1 = (x1 - x0) * 256 / (y1 - y0 + 1);
        int16_t b1 = x0 - m1 * y0 / 256;
        int16_t m2 = (x2 - x0) * 256 / (y2 - y0 + 1);
        int16_t b2 = x0 - m2 * y0 / 256;

        // Draw the horizontal lines between the pairs of points with the same y value
        int16_t curx1 = x0;
        int16_t curx2 = x0;
        for (int16_t scanlineY = y0; scanlineY <= y1; scanlineY++) {
            curx1 = m1 * scanlineY / 256 + b1;
            curx2 = m2 * scanlineY / 256 + b2;
            drawHLine(curx1, curx2, scanlineY, color);
        }

        m1 = (x2 - x1) * 256 / (y2 - y1 + 1);
        b1 = x1 - m1 * y1 / 256;
        for (int16_t scanlineY = y1; scanlineY <= y2; scanlineY++) {
            curx1 = m1 * scanlineY / 256 + b1;
            curx2 = m2 * scanlineY / 256 + b2;
            drawHLine(curx1, curx2, scanlineY, color);
        }
    }
    I _image;
};

