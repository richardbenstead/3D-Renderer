#pragma once
#include <cmath>

struct colRGB {
    colRGB() = default;
    colRGB(colRGB const &) = default;
    colRGB(float r, float g, float b) {
        _dat[0] = r;
        _dat[1] = g;
        _dat[2] = b;
    }

    colRGB lerp(const colRGB &y, float w) const {
        return colRGB(std::lerp(r(), y.r(), w), std::lerp(g(), y.g(), w), std::lerp(b(), y.b(), w));
    }
    colRGB operator*(float w) const { return colRGB(r() * w, g() * w, b() * w); }
    colRGB operator+(const colRGB &w) const { return colRGB(r() + w.r(), g() + w.g(), b() + w.b()); }

    float r() const { return _dat[0]; }
    float g() const { return _dat[1]; }
    float b() const { return _dat[2]; }

    float _dat[3] = {};
};
