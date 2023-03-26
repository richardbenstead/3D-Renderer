#pragma once

#include <stdint.h>
#include <type_traits>
#include <Eigen/Dense>

using Vec2d = Eigen::Vector2d;
using Vec3d = Eigen::Vector3d;
using Vec4d = Eigen::Vector4d;
using Mat3d = Eigen::Matrix3d;

// Calculates the distance from a point to a line
double DistanceToLine(Vec3d const &line_point1, Vec3d const &line_point2) {
    Vec3d line = line_point2 - line_point1;
    double t = line_point1.dot(line) / line.norm();
    if (t < 0) {
        return line_point1.norm();
    } else if (t > 1) {
        return line_point2.norm();
    } else {
        Vec3d projection = line_point1 + t * line;
        return projection.norm();
    }
}

Vec3d Normal(Vec3d const &p1, Vec3d const &p2, Vec3d const &p3) {
    Vec3d cross = (p2 - p1).cross(p3 - p1);
    return cross.normalized();
}

double NormToPoint(Vec3d const &t1, Vec3d const &t2, Vec3d const &t3, Vec3d const &p) {
    // Calculate the normal of the triangle
    Vec3d normal = Normal(t1, t2, t3);
    Vec3d vecToP = (p - t1).normalized();
    double facePoint = -normal.dot(vecToP);
    return facePoint;
}

double ShortestDistance(Vec3d const &t1, Vec3d const &t2, Vec3d const &t3) {
    // Calculate the normal of the triangle
    Vec3d normal = Normal(t1, t2, t3);

    // Calculate the distance from the point to the plane of the triangle
    double d = normal.dot(t1);

    // If the point is on the same side of the plane as the normal, the shortest
    // distance is the distance from the point to the plane
    if (d * normal.dot(t1) > 0) {
        return std::abs(d);
    }

    // Otherwise, the shortest distance is the distance from the point to one of the edges or vertices of the triangle
    return std::min({DistanceToLine(t1, t2), DistanceToLine(t2, t3), DistanceToLine(t3, t1), t1.norm(), t2.norm(), t3.norm()});
}

struct RenderVertex {
    RenderVertex(Vec2d const& im, Vec2d const& tx) : imageCoord(im), textureCoord(tx) {}
    Vec2d imageCoord;
    Vec2d textureCoord;
};

uint_fast8_t lerp8(uint_fast8_t a, uint_fast8_t b, float progress) {
    // Cast to int, avoid horrible values when b-a is less than zero
    return a + (int_fast8_t)((int_fast8_t)b - (int_fast8_t)a) * progress;
}

static uint16_t color565(uint8_t r, uint8_t g, uint8_t b) { return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3); }

static std::tuple<float, float, float> fromColor565(uint16_t col) {
    uint8_t baseR = (col & 0xf800) >> 8;
    uint8_t baseG = (col & 0x07e0) >> 3;
    uint8_t baseB = (col & 0x001f) << 3;
    return {static_cast<float>(baseR) / 0xF8,
                static_cast<float>(baseG) / 0xFC,
                static_cast<float>(baseB) / 0xF8};
}

uint16_t lerpCol(uint16_t col1, uint16_t col2, float progress) {
    uint8_t baseR = (col1 & 0xf800) >> 8;
    uint8_t baseG = (col1 & 0x07e0) >> 3;
    uint8_t baseB = (col1 & 0x001f) << 3;

    uint8_t tgtR = (col2 & 0xf800) >> 8;
    uint8_t tgtG = (col2 & 0x07e0) >> 3;
    uint8_t tgtB = (col2 & 0x001f) << 3;

    uint8_t r = lerp8(baseR, tgtR, progress);
    uint8_t g = lerp8(baseG, tgtG, progress);
    uint8_t b = lerp8(baseB, tgtB, progress);
    return color565(r, g, b);
}

template <typename T> T fract(T x) { return x - floor(x); }

