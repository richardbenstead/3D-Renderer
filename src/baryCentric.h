#pragma once
#include "types.h"

struct BaryCentric {
    BaryCentric(RenderVertex const &b, RenderVertex const &c, RenderVertex const &a, Vec3d const &vertexDist)
        : _vertexDist(vertexDist) {
        _a = a.imageCoord;
        v0 = b.imageCoord - a.imageCoord;
        v1 = c.imageCoord - a.imageCoord;
        d00 = v0.cast<double>().dot(v0.cast<double>());
        d01 = v0.cast<double>().dot(v1.cast<double>());
        d11 = v1.cast<double>().dot(v1.cast<double>());
        denom = d00 * d11 - d01 * d01;

        texCoords.col(0) = a.textureCoord;
        texCoords.col(1) = b.textureCoord;
        texCoords.col(2) = c.textureCoord;
    }

    inline std::pair<Vec2d, double> ConvertImageToTextureDist(Vec2i const &p) const {
        Vec2i v2 = p - _a;
        double d20 = v2.cast<double>().dot(v0.cast<double>());
        double d21 = v2.cast<double>().dot(v1.cast<double>());

        Vec3d bWeights;
        bWeights[0] = (d11 * d20 - d01 * d21) / denom;
        bWeights[1] = (d00 * d21 - d01 * d20) / denom;
        bWeights[2] = 1.0f - bWeights[0] - bWeights[1];
        return {texCoords * bWeights, bWeights.dot(_vertexDist)};
    }
    Vec2i _a;
    double d00, d01, d11, denom;
    Vec3d _vertexDist; // dist from camera
    Vec2i v0, v1;
    Eigen::Matrix<double, 2, 3> texCoords; // mapping from vertices to texture coordinates
};
