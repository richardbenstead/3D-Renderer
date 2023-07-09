#pragma once

struct colRGB {
    colRGB() = default;
    colRGB(colRGB const &) = default;
    colRGB(float r, float g, float b) {
        _dat[0] = r;
        _dat[1] = g;
        _dat[2] = b;
    }

    colRGB operator*(float w) const { return colRGB(r() * w, g() * w, b() * w); }
    colRGB operator+(const colRGB &w) const { return colRGB(r() + w.r(), g() + w.g(), b() + w.b()); }
    colRGB& operator+=(const colRGB &w) {
        _dat[0] += w.r();
        _dat[1] += w.g();
        _dat[2] += w.b();
        return *this;
    }
    colRGB operator-(const colRGB &w) const { return colRGB(r() - w.r(), g() - w.g(), b() - w.b()); }
    colRGB operator/(float w) const { return colRGB(r() / w, g() / w, b() / w); }

    float r() const { return _dat[0]; }
    float g() const { return _dat[1]; }
    float b() const { return _dat[2]; }

    float _dat[3] = {};
};
