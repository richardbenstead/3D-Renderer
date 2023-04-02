#pragma once
#include <array>

#include "3dUtils.h"
#include "colRGB.h"
#include "font.h"


struct TextureShader {
    TextureShader(int val) : _val{val} {}
    static constexpr auto texture = bmNum;

    void setAttr(double facesLight, colRGB baseCol) {
        _specular = sqrt(facesLight / 2);
        _baseCol = baseCol;
    }

    inline colRGB GetValue(float x, float y, float dist) const {
        // double rv = rand() % 100;
        // [[maybe_unused]] bool inCentre = sqrt(pow(x - 0.5, 2) + pow(y - 0.5, 2)) < 0.5;
        // [[maybe_unused]] colRGB basePix = colRGB(x + inCentre, y, rv / 20.0);

        int xInd = static_cast<int>(x * 8) & 7;
        int yInd = static_cast<int>(y * 8) & 7;
        bool pixelVal = (texture[_val][yInd] >> xInd) & 0b1;
        double specular = pixelVal ? 0.5 : 0.0;

        colRGB const base = !pixelVal ? _baseCol : colRGB(1, 1, 1);
        double fade = std::clamp(8.0f / ((dist * dist) - 10), 0.0f, 1.5f);
        colRGB col = base * fade;
        return col.lerp(colRGB(1, 1, 1), std::min(1.0, specular * _specular));
    }
    double _specular{};
    int _val{};
    colRGB _baseCol;
};

class Triangle {
  public:
    Triangle() = default;
    Triangle(Triangle const &t2) = delete;
    Triangle &operator=(Triangle const &t2) = delete;
    Triangle(Triangle &&t2) = default;
    Triangle &operator=(Triangle &&t2) = default;

    Triangle(Eigen::Matrix<double, 2, 3> const& points, Eigen::Matrix<double, 2, 3> const &_texCoords,
             Vec3d const &_vertexDist, TextureShader const &_ts)
        : p1(points.col(0)), p2(points.col(1)), p3(points.col(2)), texCoords(_texCoords), vertexDist(_vertexDist), shader(_ts) {}

    Vec2d p1{0, 0}, p2{0, 0}, p3{0, 0};
    Eigen::Matrix<double, 2, 3> texCoords = Eigen::Matrix<double, 2, 3>::Zero();
    Vec3d vertexDist{-1, -1, -1};
    TextureShader shader{0};

    std::partial_ordering operator<=>(Triangle const &t2) {
        int sumFurther = (vertexDist[0] > t2.vertexDist[0]) + (vertexDist[0] > t2.vertexDist[1]) +
                         (vertexDist[0] > t2.vertexDist[2]) + (vertexDist[1] > t2.vertexDist[0]) +
                         (vertexDist[1] > t2.vertexDist[1]) + (vertexDist[1] > t2.vertexDist[2]) +
                         (vertexDist[2] > t2.vertexDist[0]) + (vertexDist[2] > t2.vertexDist[1]) +
                         (vertexDist[2] > t2.vertexDist[2]);
        if (sumFurther >= 5) {
            return std::partial_ordering::less;
        }
        if (sumFurther <= 4) {
            return std::partial_ordering::greater;
        }

        if (vertexDist[0] > t2.vertexDist.maxCoeff() || vertexDist[1] > t2.vertexDist.maxCoeff() ||
            vertexDist[2] > t2.vertexDist.maxCoeff()) {
            return std::partial_ordering::less;
        }

        if (t2.vertexDist[0] > vertexDist.maxCoeff() || t2.vertexDist[1] > vertexDist.maxCoeff() ||
            t2.vertexDist[2] > vertexDist.maxCoeff()) {
            return std::partial_ordering::greater;
        }
        return std::partial_ordering::unordered;
    }
};

class Object {
  public:
    virtual ~Object() {}
    Object() = default;
    Object(Object &&) = delete;
    Object(Object const &) = delete;

    Object(Vec3d const &centre) : _targetCentre(centre) {}
    virtual std::vector<Triangle> getTriangles(Vec3d const &) = 0;
    Vec3d _centre{0, 0, 10};
    Vec3d _rotation{0, 0, 0};
    Vec3d _targetRotation{0, 0, 0};
    Vec3d _targetCentre{0, 0, 0};
    Vec3d _velocity{0, 0, 0};
    Vec3d _angularVelocity{0.3, 0.6, 10};

    void update([[maybe_unused]] uint32_t time) {
        _rotation += _angularVelocity;
        _angularVelocity += 0.1 * (_targetRotation - _rotation);

        auto getRand = []() { return static_cast<float>(rand() % 10 - 5); };
        _angularVelocity += Vec3d{getRand(), getRand(), getRand()} * 0.1;
        _centre += _velocity;
        _velocity += 0.1 * (_targetCentre - _centre);
        //_velocity += Vec3d{getRand(), getRand(), getRand()} * 0.001;

        static constexpr double maxAngularSpeed{20};
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
        return (q_z * q_y * q_x).toRotationMatrix() * vertices;
    }
};

class Cube : public Object {
  public:
    Cube(Vec3d const &centre) : Object(centre) {}

    static constexpr double arrVert[8][3] = {{-0.5, -0.5, -0.5}, {0.5, -0.5, -0.5}, {0.5, 0.5, -0.5}, {-0.5, 0.5, -0.5},
                                             {-0.5, -0.5, 0.5},  {0.5, -0.5, 0.5},  {0.5, 0.5, 0.5},  {-0.5, 0.5, 0.5}};

    TextureShader ts[11] = {TextureShader{0}, TextureShader{1}, TextureShader{2}, TextureShader{3},
                            TextureShader{4}, TextureShader{5}, TextureShader{6}, TextureShader{7},
                            TextureShader{8}, TextureShader{9}, TextureShader{10}};

    int faceTexMap[6] = {1, 10, 3, 10, 4, 5};

    std::vector<Triangle> getTriangles(Vec3d const &camera) {
        // Rotate base vertices and offset from camera
        Eigen::Matrix<double, 3, 8> const vertices = Eigen::Matrix<double, 3, 8>::Map(arrVert[0]);
        Eigen::Matrix<double, 3, 8> const matPfc = rotate(vertices, _rotation).colwise() + (_centre - camera);
        using TexCoords_t = Eigen::Matrix<double, 2, 3>;

        // project relative 3d coordinates to screen
        Eigen::Matrix<double, 2, 8> const projectedVertices =
            (Eigen::Matrix<double, 2, 3>() << 0.6, 0, 0, 0, 1, 0).finished() * matPfc *
            Eigen::DiagonalMatrix<double, 8>{matPfc.row(2)}.inverse();

        std::vector<Triangle> triangles;
        triangles.reserve(12);

        // Create the triangles that make up the cube
        auto makeTri = [&](Vec3i const& arrVertInd, TexCoords_t const &texCoords, TextureShader &ts, colRGB const col) {
            if (FacesCamera(matPfc(Eigen::all, {arrVertInd[0], arrVertInd[1], arrVertInd[2]}))) {
                double const facesLight = std::max(0.0, NormToPoint(matPfc(Eigen::all, arrVertInd), {2, 2, 0}));
                ts.setAttr(facesLight, col);
                triangles.emplace_back(Triangle(projectedVertices(Eigen::all, arrVertInd), texCoords,
                            matPfc(Eigen::all, arrVertInd).colwise().norm(), ts));
            }
        };

        TexCoords_t const texCoords1 = (TexCoords_t() << 0, 0, 1, 0, 1, 0).finished();
        TexCoords_t const texCoords2 = (TexCoords_t() << 1, 1, 0, 1, 0, 1).finished();
        float low = 0.5f;
        float high = 0.9f;
        colRGB const colours[] = {colRGB(high, low, low),  colRGB(low, high, low),  colRGB(low, low, high),
                                  colRGB(high, high, low), colRGB(low, high, high), colRGB(high, low, high)};

        // vertices triangles of faces of the cube
        static constexpr int vertexList[12][3] = {{0, 1, 2}, {2, 3, 0}, {1, 5, 6}, {6, 2, 1}, {7, 6, 5}, {5, 4, 7},
                                                  {4, 0, 3}, {3, 7, 4}, {4, 5, 1}, {1, 0, 4}, {3, 2, 6}, {6, 7, 3}};

        for (int i = 0; i < 6; ++i) {
            makeTri(Vec3i::Map(vertexList[i * 2]), texCoords1, ts[faceTexMap[i]], colours[i]);
            makeTri(Vec3i::Map(vertexList[i * 2+1]), texCoords2, ts[faceTexMap[i]], colours[i]);
        }

        return triangles;
    }
    Vec3d const faceRotation[6] = {{0, 0, 0}, {0, 90, 0}, {180, 0, 0}, {0, 270, 0}, {90, 0, 0}, {270, 0, 0}};
    static constexpr int oppFace[6] = {2, 3, 0, 1, 5, 4};

    void setTargetTex(int tex) {
        // find if an existing face maps to tex
        for (int i = 0; i < 6; ++i) {
            if (faceTexMap[i] == tex) {
                _targetFace = i;
                _targetRotation = faceRotation[_targetFace];
                return;
            }
        }
        _targetFace = oppFace[_targetFace];
        _targetRotation = faceRotation[_targetFace];
        faceTexMap[_targetFace] = tex;
    }
    int _targetFace{};
};
