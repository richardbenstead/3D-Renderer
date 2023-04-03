#pragma once
#include "types.h"

struct BaryCentric {
    BaryCentric(RenderVertex const &b, RenderVertex const &c, RenderVertex const &a, Vec3f const &vertexDist)
        : _vertexDist(vertexDist) {
        _a = a.imageCoord;
        v0 = b.imageCoord - a.imageCoord;
        v1 = c.imageCoord - a.imageCoord;
        d00 = v0.cast<float>().dot(v0.cast<float>());
        d01 = v0.cast<float>().dot(v1.cast<float>());
        d11 = v1.cast<float>().dot(v1.cast<float>());
        denom = d00 * d11 - d01 * d01;

        texCoords.col(0) = a.textureCoord;
        texCoords.col(1) = b.textureCoord;
        texCoords.col(2) = c.textureCoord;
    }

    inline std::pair<Vec2f, float> ConvertImageToTextureDist(Vec2i const &p) const {
        Vec2i v2 = p - _a;
        float d20 = v2.cast<float>().dot(v0.cast<float>());
        float d21 = v2.cast<float>().dot(v1.cast<float>());

        Vec3f bWeights;
        bWeights[0] = (d11 * d20 - d01 * d21) / denom;
        bWeights[1] = (d00 * d21 - d01 * d20) / denom;
        bWeights[2] = 1.0f - bWeights[0] - bWeights[1];
        return {texCoords * bWeights, bWeights.dot(_vertexDist)};
    }
    Vec2i _a;
    float d00, d01, d11, denom;
    Vec3f _vertexDist; // dist from camera
    Vec2i v0, v1;
    Eigen::Matrix<float, 2, 3> texCoords; // mapping from vertices to texture coordinates
};
