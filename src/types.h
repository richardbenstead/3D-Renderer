#pragma once
#include <Eigen/Dense>

using Vec2i = Eigen::Vector2i;
using Vec3i = Eigen::Vector3i;
using Vec2f = Eigen::Vector2f;
using Vec3f = Eigen::Vector3f;
using Vec4f = Eigen::Vector4f;
using Mat3f = Eigen::Matrix3f;

struct RenderVertex {
    RenderVertex(Vec2i const &&im, Vec2f const &tx) : imageCoord{im}, textureCoord{tx} {}
    RenderVertex(RenderVertex const &t2) = delete;
    RenderVertex &operator=(RenderVertex const &t2) = delete;
    RenderVertex(RenderVertex &&t2) = default;
    RenderVertex &operator=(RenderVertex &&t2) noexcept {
        imageCoord = t2.imageCoord;
        textureCoord = t2.textureCoord;
        return *this;
    }
    Vec2i imageCoord;
    Vec2f textureCoord;
};
