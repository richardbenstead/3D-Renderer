#pragma once
#include <Eigen/Dense>

using Vec2i = Eigen::Vector2i;
using Vec2d = Eigen::Vector2d;
using Vec3d = Eigen::Vector3d;
using Vec4d = Eigen::Vector4d;
using Mat3d = Eigen::Matrix3d;

struct RenderVertex {
    RenderVertex(Vec2i const &im, Vec2d const &tx) : imageCoord(im), textureCoord(tx) {}
    Vec2i imageCoord;
    Vec2d textureCoord;
};

