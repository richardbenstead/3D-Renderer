#pragma once
#include "linalg.h"
#include "MathUtil.h"
#include <iostream>

uint16_t mapColor(float value) {
    // Map the value to the range 0-6
    float hue = value * 5;

    // Calculate the red, green, and blue values
    float r, g, b;
    if (hue < 1) {
        r = hue;
        g = 0;
        b = 0;
    } else if (hue < 2) {
        r = 1;
        g = hue - 1;
        b = 0;
    } else if (hue < 3) {
        r = 3 - hue;
        g = 1;
        b = hue - 2;
    } else if (hue < 4) {
        r = hue - 3;
        g = 4 - hue;
        b = 1;
    } else {
        r = 1;
        g = hue - 4;
        b = 1;
    }

    // Return the color as a 16-bit value
    return (((uint16_t)(r * 31)) << 11) | (((uint16_t)(g * 63)) << 5) | ((uint16_t)(b * 31));
}

bool FacesCamera(Vec3d t1, Vec3d t2, Vec3d t3) {
    // Calculate the normal of the triangle
    Vec3d normal = Normal(t1, t2, t3);
    double d = normal.dot(t1);
    return d > 0;
}

class Triangle {
  public:
    Vec2d p1, p2, p3;
    double distFromCamera;
    bool facesCamera;
    uint16_t col;
};

class Object {
  public:
    Object(Vec3d const &centre) : _targetCentre(centre) {}
    virtual std::vector<Triangle> getTriangles(Vec3d const &) = 0;
    Vec3d _centre{0, 0, 10};
    Vec3d _rotation{};
    Vec3d _targetRotation{};
    Vec3d _targetCentre{};
    Vec3d _velocity{};
    Vec3d _angularVelocity{0.3, 0.6, 10};

    void update([[maybe_unused]] uint32_t time) {
        _rotation += _angularVelocity;
        _angularVelocity += 0.1 * (_targetRotation - _rotation);
        _centre += _velocity;
        _velocity += 0.1 * (_targetCentre - _centre);

        const double maxAngularSpeed{20};
        _angularVelocity *= 0.95;
        if (_angularVelocity.norm() > maxAngularSpeed) {
            _angularVelocity = _angularVelocity.normalized() * maxAngularSpeed;
        }

        _velocity *= 0.95;
        const double maxSpeed{0.1};
        if (_velocity.norm() > maxSpeed) {
            _velocity = _velocity.normalized() * maxSpeed;
        }
    }

  protected:
    Eigen::MatrixXd rotate(const Eigen::MatrixXd &vertices, const Vec3d &angles) {
        // Create quaternions for each rotation
        Eigen::Quaterniond q_x(Eigen::AngleAxisd(angles[0] * M_PI / 180.0, Eigen::Vector3d::UnitX()));
        Eigen::Quaterniond q_y(Eigen::AngleAxisd(angles[1] * M_PI / 180.0, Eigen::Vector3d::UnitY()));
        Eigen::Quaterniond q_z(Eigen::AngleAxisd(angles[2] * M_PI / 180.0, Eigen::Vector3d::UnitZ()));

        // Rotate the vertices
        return (q_z * q_y * q_x).toRotationMatrix() * vertices;
    }
};

class Cube : public Object {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Cube(Vec3d const &centre) : Object(centre) {}

    static double arrVert[8][3];

    std::vector<Triangle> getTriangles(Vec3d const &camera) {
        // Rotate + project the vertices onto the 2D plane
        Eigen::Matrix<double, 3, 8> vertices = Eigen::Matrix<double, 3, 8>::Map(arrVert[0]);
        Eigen::Matrix<double, 3, 8> matPfc = rotate(vertices, _rotation).colwise() + (_centre - camera);

        std::array<Vec2d, 8> projectedVertices;
        for (int i = 0; i < 8; i++) {
            projectedVertices[i] = {matPfc(0, i) / matPfc(2, i),
                                    matPfc(1, i) / matPfc(2, i)}; // focal plane 1 unit behind lens
        }

        std::vector<Triangle> triangles;
        triangles.reserve(12);

        // Create the triangles that make up the cube
        auto makeTri = [&](int i, int j, int k, uint16_t col) {
            bool facesCamera = FacesCamera(matPfc.col(i), matPfc.col(j), matPfc.col(k));

            if (facesCamera) {
                // shine if we face the light
                double facesLight = std::max(0.0, NormToPoint(matPfc.col(i), matPfc.col(j), matPfc.col(k), {2, 2, 0}));
                col = lerpCol(col, 0xffff, facesLight / 2);

                // dark if we are far away
                double dist = matPfc.col(k)[2]; // ShortestDistance(matPfc.col(i), matPfc.col(j), matPfc.col(k));
                double fade = std::clamp(sqrt(dist / 20), 0.0, 2.0) / 2;
                col = lerpCol(col, 0, fade);
                triangles.push_back({projectedVertices[i], projectedVertices[j], projectedVertices[k], dist, facesCamera, col});
            }
        };

        makeTri(0, 1, 2, mapColor(0.2));
        makeTri(2, 3, 0, mapColor(0.2));
        makeTri(1, 5, 6, mapColor(0.3));
        makeTri(6, 2, 1, mapColor(0.3));
        makeTri(7, 6, 5, mapColor(0.4));
        makeTri(5, 4, 7, mapColor(0.4));
        makeTri(4, 0, 3, mapColor(0.5));
        makeTri(3, 7, 4, mapColor(0.5));
        makeTri(4, 5, 1, mapColor(0.6));
        makeTri(1, 0, 4, mapColor(0.6));
        makeTri(3, 2, 6, mapColor(0.7));
        makeTri(6, 7, 3, mapColor(0.7));
        return triangles;
    }
};

double Cube::arrVert[8][3] = {{-0.5, -0.5, -0.5}, {0.5, -0.5, -0.5}, {0.5, 0.5, -0.5}, {-0.5, 0.5, -0.5},
                              {-0.5, -0.5, 0.5},  {0.5, -0.5, 0.5},  {0.5, 0.5, 0.5},  {-0.5, 0.5, 0.5}};
