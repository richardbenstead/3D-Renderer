#pragma once
#include "MathUtil.h"
#include "utils.h"
#include <iostream>

struct TextureShader {
    TextureShader(int val) : _val{val} {}
    constexpr static uint8_t texture[10][8] = {
        // clang-format off
                {0b00000000,
                  0b01111110,
                  0b01000010,
                  0b01000010,
                  0b01000010,
                  0b01000010,
                  0b01111110,
                  0b00000000},
                {0b00000000,
                  0b00011000,
                  0b00111000,
                  0b00011000,
                  0b00011000,
                  0b00011000,
                  0b01111110,
                  0b00000000},
                {0b00000000,
                  0b01111110,
                  0b00000010,
                  0b00111110,
                  0b01100000,
                  0b01000000,
                  0b01111110,
                  0b00000000},
                {0b00000000,
                  0b01111110,
                  0b00000010,
                  0b01111110,
                  0b00000110,
                  0b00000010,
                  0b01111110,
                  0b00000000},
                {0b00000000,
                  0b01000010,
                  0b01000010,
                  0b01111110,
                  0b00000110,
                  0b00000010,
                  0b00000010,
                  0b00000000},
                {0b00000000,
                  0b01111110,
                  0b01000000,
                  0b01111100,
                  0b00000110,
                  0b00000010,
                  0b01111110,
                  0b00000000},
                {0b00000000,
                  0b01111110,
                  0b01000000,
                  0b01111100,
                  0b01000110,
                  0b01000010,
                  0b01111110,
                  0b00000000},
                {0b00000000,
                  0b01111110,
                  0b00000110,
                  0b00000110,
                  0b00001110,
                  0b00001100,
                  0b00001100,
                  0b00000000},
                {0b00000000,
                  0b01111110,
                  0b01000010,
                  0b01111110,
                  0b01000110,
                  0b01000010,
                  0b01111110,
                  0b00000000},
                {0b00000000,
                  0b01111110,
                  0b01000010,
                  0b01111110,
                  0b00000110,
                  0b00000010,
                  0b01111110,
                  0b00000000}};
    // clang-format on

    void setAttr(double facesLight, uint16_t baseCol) {
        _specular = sqrt(facesLight / 2);
        _baseCol = baseCol;
    }

    Pixel GetValue(float x, float y, float dist) const {
        /*
        [[maybe_unused]] double rv = rand() % 100;
        [[maybe_unused]] bool inCentre = sqrt(pow(x - 0.5, 2) + pow(y - 0.5, 2)) < 0.5;
        [[maybe_unused]] Pixel basePix = Pixel(x + inCentre, y, rv / 20.0);
        */

        int xInd = std::clamp(static_cast<int>(x * 8), 0, 7);
        int yInd = std::clamp(static_cast<int>(y * 8), 0, 7);
        bool pixelVal = (texture[_val][yInd] >> xInd) & 1;
        float specular = pixelVal ? 0.5 : 0.0;

        auto [r, g, b] = fromColor565(_baseCol);
        float sf = !pixelVal;
        Pixel const base(r * sf, g * sf, b * sf);
        double fade = std::clamp(8.0f / ((dist * dist)-10), 0.0f, 1.5f);
        Pixel col = base * fade;
        return col.lerp(Pixel(1,1,1), std::min(1.0, specular * _specular));
    }
    double _specular{};
    int _val{};
    uint16_t _baseCol;
};


bool FacesCamera(Vec3d t1, Vec3d t2, Vec3d t3) {
    // Calculate the normal of the triangle
    Vec3d normal = Normal(t1, t2, t3);
    double d = normal.dot(t1);
    return d > 0;
}

class Triangle {
  public:
    Vec2d p1, p2, p3;
    Eigen::Matrix<double, 2, 3> texCoords;
    Vec3d vertexDist;
    TextureShader ts;
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
        _angularVelocity += Vec3d{rand()%10 - 5, rand()%10 - 5, rand()%10 - 5}*0.1;
        _centre += _velocity;
        _velocity += 0.1 * (_targetCentre - _centre);
        _velocity += Vec3d{rand()%10 - 5, rand()%10 - 5, rand()%10 - 5}*0.001;

        const double maxAngularSpeed{40};
        _angularVelocity *= 0.90;
        if (_angularVelocity.norm() > maxAngularSpeed) {
            _angularVelocity = _angularVelocity.normalized() * maxAngularSpeed;
        }

        _velocity *= 0.95;
        const double maxSpeed{0.2};
        if (_velocity.norm() > maxSpeed) {
            _velocity = _velocity.normalized() * maxSpeed;
        }
    }

  protected:
    Eigen::MatrixXd rotate(const Eigen::MatrixXd &vertices, const Vec3d &angles) {
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

    TextureShader ts[10] = {TextureShader{0}, TextureShader{1}, TextureShader{2}, TextureShader{3}, TextureShader{4},
                            TextureShader{5}, TextureShader{6}, TextureShader{7}, TextureShader{8}, TextureShader{9}};

    int faceTexMap[6] = {5, 1, 3, 4, 0, 2};
    std::vector<Triangle> getTriangles(Vec3d const &camera) {
        // Rotate base vertices and offset from camera
        Eigen::Matrix<double, 3, 8> const vertices = Eigen::Matrix<double, 3, 8>::Map(arrVert[0]);
        Eigen::Matrix<double, 3, 8> const matPfc = rotate(vertices, _rotation).colwise() + (_centre - camera);

        // project relative 3d coordinates to screen
        Eigen::Matrix<double, 2, 8> const projectedVertices =
            (Eigen::Matrix<double, 2, 3>() << 0.6, 0, 0, 0, 1, 0).finished() * matPfc *
            Eigen::DiagonalMatrix<double, 8>{matPfc.row(2)}.inverse();

        std::vector<Triangle> triangles;
        triangles.reserve(12);

        // Create the triangles that make up the cube
        auto makeTri = [&](int const *vert, Eigen::Matrix<double, 2, 3> const texCoords, TextureShader ts,
                           uint16_t const col) {
            bool const facesCamera = FacesCamera(matPfc.col(vert[0]), matPfc.col(vert[1]), matPfc.col(vert[2]));

            if (facesCamera) {
                double const facesLight = std::max(
                    0.0, NormToPoint(matPfc.col(vert[0]), matPfc.col(vert[1]), matPfc.col(vert[2]), {2, 2, 0}));
                ts.setAttr(facesLight, col);
                Vec3d vertexDist{matPfc(2, vert[0]), matPfc(2, vert[1]), matPfc(2, vert[2])};
                triangles.push_back({projectedVertices.col(vert[0]), projectedVertices.col(vert[1]),
                                     projectedVertices.col(vert[2]), texCoords, vertexDist, ts});
            }
        };

        auto const texCoords1 = (Eigen::Matrix<double, 2, 3>() << 0, 0, 1, 0, 1, 0).finished();
        auto const texCoords2 = (Eigen::Matrix<double, 2, 3>() << 1, 1, 0, 1, 0, 1).finished();
        uint16_t const colours[] = {color565(200, 100, 100),   color565(100, 200, 100),   color565(100, 100, 200),
                                    color565(200, 200, 100), color565(100, 200, 200), color565(200, 100, 200)};
        int const vertexList[12][3] = {{0, 1, 2}, {2, 3, 0}, {1, 5, 6}, {6, 2, 1}, {5,4,7}, {7, 6, 5},
                                       {4, 0, 3}, {3, 7, 4}, {4, 5, 1}, {1, 0, 4}, {3, 2, 6}, {6, 7, 3}};

        for (int i = 0; i < 6; ++i) {
            makeTri(vertexList[i * 2], texCoords1, ts[faceTexMap[i]], colours[i]);
            makeTri(vertexList[i * 2 + 1], texCoords2, ts[faceTexMap[i]], colours[i]);
        }

        return triangles;
    }
    Vec3d const faceRotation[6] = {{0, 0, 0}, {0, 90, 0}, {0, 180, 0}, {0, 270, 0}, {90, 0, 0}, {270, 0, 0}};
    int oppFace[6] = {2,3,0,1,5,4};

    void setTargetTex(int tex) {
        // find if an existing face maps to tex
        for(int i=0; i<6; ++i) {
            if (faceTexMap[i] == tex) {
                _targetFace = i;
                _targetRotation = faceRotation[_targetFace];
                return;
            }
        }
        _targetFace = oppFace[_targetFace];
        _targetRotation = faceRotation[_targetFace];
        faceTexMap[_targetFace] = tex;
        std::cout << "remapped, tgt: " << tex << " face: " << _targetFace << std::endl;
    }
    int _targetFace{};
};

double Cube::arrVert[8][3] = {{-0.5, -0.5, -0.5}, {0.5, -0.5, -0.5}, {0.5, 0.5, -0.5}, {-0.5, 0.5, -0.5},
                              {-0.5, -0.5, 0.5},  {0.5, -0.5, 0.5},  {0.5, 0.5, 0.5},  {-0.5, 0.5, 0.5}};
