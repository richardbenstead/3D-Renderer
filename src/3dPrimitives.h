#pragma once
#include <array>
#include <memory>
#include <type_traits>

#include "3dUtils.h"
#include "colRGB.h"
#include "font.h"

static constexpr float low = 0.1f;
static constexpr float high = 1.0f;

class Camera {
    public:
        Camera(Vec3f const pos, Vec3f const orientation) :
            position{pos},
            counterRotation{[&]() {
                Vec3f target(0, 0, 1);
                Vec3f const v = orientation.normalized();
                Vec3f const axis = v.cross(target);
                float angle = std::acos(v.dot(target));
                return Eigen::AngleAxisf(angle, axis);
            }()}
        {}

    Vec3f const position;
    Eigen::Matrix3f const counterRotation;
};

struct IShader {
    virtual void setAttr(float facesLight, bool facesCamera) = 0;
    virtual bool isTex() { return false; }
};

struct BasicShader : public IShader {
    BasicShader(colRGB const col1, colRGB const col2) : _col1(col1), _col2(col2) {}
    static constexpr auto texture = bmNum;

    void setAttr(float facesLight, bool facesCamera) {
        _specular = sqrt(facesLight / 2);
        _baseCol = facesCamera ? _col1 : _col2;
    }

    inline void drawTexturedHLine(int x, int x2, int y, BaryCentric const &bc, auto& iter) const {
        float const dist1 = bc.ConvertImageToDist({x, y});
        if (x==x2) {
            [[unlikely]];
            iter.setVal(GetValue(dist1));
        } else {
            float const dist2 = bc.ConvertImageToDist({x2, y});
            colRGB col1 = GetValue(dist1);
            colRGB const col2 = GetValue(dist2);
            colRGB mCol = (col2 - col1)/static_cast<float>(x2-x);

            for (; x<=x2; ++iter, ++x) {
                iter.setVal(col1);
                col1 += mCol;
            }
        }
    }

    inline colRGB GetValue(float const dist) const {
        constexpr float specular = 0.2;
        float const fade = std::min(40.0f / (dist * dist + 0.1f), 1.5f);
        colRGB const col = _baseCol * fade;
        float const sVal = specular * _specular;
        return col + colRGB(sVal, sVal, sVal);
    }
    float _specular{};
    colRGB _col1, _col2, _baseCol;
};

struct TextureShader : public IShader {
    TextureShader(int val, colRGB col) : _val{val}, _baseCol(col) {}
    static constexpr auto texture = bmNum;

    void setAttr(float facesLight, [[maybe_unused]] bool facesCamera) { _specular = sqrt(facesLight / 2); }
    bool isTex() { return true; }

    inline colRGB GetValue(float x, float y, float dist) const {
        // float rv = rand() % 100;
        // [[maybe_unused]] bool inCentre = sqrt(pow(x - 0.5, 2) + pow(y - 0.5, 2)) < 0.5;
        // [[maybe_unused]] colRGB basePix = colRGB(x + inCentre, y, rv / 20.0);

        bool pixelVal = false;
        if (_val > 0) {
            int xInd = static_cast<int>(x * 8) & 7;
            int yInd = static_cast<int>(y * 8) & 7;
            pixelVal = (texture[_val][yInd] >> xInd) & 0b1;
        }
        float specular = pixelVal ? 0.5 : 0.0;

        colRGB const base = !pixelVal ? _baseCol : colRGB(1.0f, 1.0f, 1.0f);
        float fade = std::clamp(20.0f / ((dist * dist) - 50), 0.0f, 1.5f);

        colRGB col = base * fade;
        float const sVal = specular * _specular;
        return col + colRGB(sVal, sVal, sVal);
    }

    inline void drawTexturedHLine(int x, int x2, int y, BaryCentric const &bc, auto& iter) const {
        Vec2f tex1, tex2;
        float dist1, dist2;
        std::tie(tex1, dist1) = bc.ConvertImageToTextureDist({x, y});
        if (x==x2) {
            [[unlikely]];
            iter.setVal(GetValue(tex1[0], tex1[1], dist1));
        } else {
            std::tie(tex2, dist2) = bc.ConvertImageToTextureDist({x2, y});
            Vec2f mTex= (tex2 - tex1) / static_cast<float>(x2 - x);
            float mDist = (dist2 - dist1) / static_cast<float>(x2 - x);

            int i = 0;
            for (; x <= x2; ++x, ++iter, ++i) {
                Vec2f tex = tex1 + i * mTex;
                iter.setVal(GetValue(tex[0], tex[1], dist1 + i * mDist));
            }
        }
    }

    float _specular{};
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

    Triangle(Eigen::Matrix<float, 2, 3> const &points, Eigen::Matrix<float, 2, 3> const &_texCoords,
             Vec3f const &_vertexDist, std::shared_ptr<IShader> &_ts)
        : p1(points.col(0)), p2(points.col(1)), p3(points.col(2)), texCoords(_texCoords), vertexDist(_vertexDist),
          shader(_ts) {}

    Vec2f p1{0, 0}, p2{0, 0}, p3{0, 0};
    Eigen::Matrix<float, 2, 3> texCoords = Eigen::Matrix<float, 2, 3>::Zero();
    Vec3f vertexDist{-1, -1, -1};
    std::shared_ptr<IShader> shader{0};

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
    using TexCoords_t = Eigen::Matrix<float, 2, 3>;
    virtual ~Object() {}
    Object() = default;
    Object(Object &&) = delete;
    Object(Object const &) = delete;

    Object(Vec3f const &centre) : _targetCentre(centre) {}
    virtual std::vector<Triangle> getTriangles(Camera const &) = 0;

    Vec3f _centre{0, 0, 10};
    Vec3f _rotation{0, 0, 0};
    Vec3f _targetRotation{0, 0, 0};
    Vec3f _targetCentre{0, 0, 0};
    Vec3f _velocity{0, 0, 0};
    Vec3f _angularVelocity{0.3, 0.6, 10};

    void update([[maybe_unused]] uint32_t time) {
        _rotation += _angularVelocity;
        _angularVelocity += 0.1 * (_targetRotation - _rotation);

        auto getRand = []() { return static_cast<float>(rand() % 10 - 5); };
        _angularVelocity += Vec3f{getRand(), getRand(), getRand()} * 0.1;
        _centre += _velocity;
        _velocity += 0.1 * (_targetCentre - _centre);
        // _velocity += Vec3f{getRand(), getRand(), getRand()} * 0.0001;

        static constexpr float maxAngularSpeed{30};
        _angularVelocity *= 0.80;

        if (_angularVelocity.norm() > maxAngularSpeed) {
            _angularVelocity = _angularVelocity.normalized() * maxAngularSpeed;
        }

        _velocity *= 0.8;
        const float maxSpeed{0.3};
        if (_velocity.norm() > maxSpeed) {
            _velocity = _velocity.normalized() * maxSpeed;
        }
    }

  protected:
    Eigen::MatrixXf rotate(const Eigen::MatrixXf &vertices, const Vec3f &angles) {
        Eigen::Quaternionf q_x(Eigen::AngleAxisf(angles[0] * M_PI / 180.0, Eigen::Vector3f::UnitX()));
        Eigen::Quaternionf q_y(Eigen::AngleAxisf(angles[1] * M_PI / 180.0, Eigen::Vector3f::UnitY()));
        Eigen::Quaternionf q_z(Eigen::AngleAxisf(angles[2] * M_PI / 180.0, Eigen::Vector3f::UnitZ()));
        return (q_z * q_y * q_x).toRotationMatrix() * vertices;
    }
};

// 3D mesh of triangles. Base class for tile and cube
template <typename Mesh> class MeshObject : public Object {
  public:
    MeshObject(Vec3f const &centre) : Object(centre) {}
    std::vector<Triangle> getTriangles(Camera const &camera) {
        auto [matPfc, faces, tsInd, texCoord, arrTextureShaders] = static_cast<Mesh *>(this)->getMesh(camera);
        using shaderType = std::remove_cvref_t<decltype(arrTextureShaders[0])>;

        // project relative 3d coordinates to screen
        Eigen::DiagonalMatrix<float, Mesh::NUM_VERTICES> diagDist{matPfc.row(2)};
        Eigen::Matrix<float, 2, Mesh::NUM_VERTICES> const projectedVertices =
            (Eigen::Matrix<float, 2, 3>() << 1, 0, 0, 0, 1, 0).finished() * matPfc * diagDist.inverse();

        std::vector<Triangle> triangles;
        triangles.reserve(Mesh::NUM_FACES);
        for (int i = 0; i < Mesh::NUM_FACES; ++i) {
            Vec3i arrVertInd = Vec3i::Map(faces[i]);
            TexCoords_t const &texCoords = texCoord[i];

            // TODO: check if shader wants to omit render if we don't face camera
            bool const facesCamera = FacesCamera(matPfc(Eigen::all, {arrVertInd[0], arrVertInd[1], arrVertInd[2]}));
            float const facesLight = std::max(0.0f, NormToPoint(matPfc(Eigen::all, arrVertInd), {2, 2, 0}));
            shaderType *pShader = new shaderType(arrTextureShaders[tsInd[i]]); // copy shader to heap
            std::shared_ptr<IShader> shader(static_cast<IShader *>(pShader));
            shader->setAttr(facesLight, facesCamera);
            triangles.emplace_back(Triangle(projectedVertices(Eigen::all, arrVertInd), texCoords,
                                            matPfc(Eigen::all, arrVertInd).colwise().norm(), shader));
        };

        return triangles;
    }
};

class Cube : public MeshObject<Cube> {
  public:
    Cube(Vec3f const &centre) : MeshObject<Cube>(centre) {}
    static constexpr int NUM_VERTICES{8}, NUM_FACES{12};
    static constexpr float vertices[NUM_VERTICES][3] = {{-0.5, -0.5, -0.5}, {0.5, -0.5, -0.5}, {0.5, 0.5, -0.5},
                                                        {-0.5, 0.5, -0.5},  {-0.5, -0.5, 0.5}, {0.5, -0.5, 0.5},
                                                        {0.5, 0.5, 0.5},    {-0.5, 0.5, 0.5}};

    static constexpr int faces[NUM_FACES][3] = {{0, 1, 2}, {2, 3, 0}, {1, 5, 6}, {6, 2, 1}, {7, 6, 5}, {5, 4, 7},
                                                {4, 0, 3}, {3, 7, 4}, {4, 5, 1}, {1, 0, 4}, {3, 2, 6}, {6, 7, 3}};

    TextureShader arrTextureShaders[11] = {
        TextureShader{0, colRGB(high, low, low)},  TextureShader{1, colRGB(high, low, low)},
        TextureShader{2, colRGB(low, high, low)},  TextureShader{3, colRGB(low, low, high)},
        TextureShader{4, colRGB(high, low, high)}, TextureShader{5, colRGB(high, high, low)},
        TextureShader{6, colRGB(low, high, high)}, TextureShader{7, colRGB(high, low, high)},
        TextureShader{8, colRGB(high, high, low)}, TextureShader{9, colRGB(high, low, low)},
        TextureShader{10, colRGB(low, high, low)}};

    int faceTexMap[6] = {1, 10, 3, 10, 4, 5};
    using matVertex = Eigen::Matrix<float, 3, NUM_VERTICES>;

    inline auto getMesh(Camera const &camera) {
        // Rotate base vertices and offset from camera
        matVertex const matPfc = rotate(matVertex::Map(vertices[0]), _rotation).colwise() + (_centre - camera.position);

        std::array<int, NUM_FACES> tsInd;
        std::array<TexCoords_t, NUM_FACES> texCoord;
        for (int i = 0; i < 6; ++i) {
            tsInd[i * 2] = faceTexMap[i];
            tsInd[i * 2 + 1] = faceTexMap[i];
            texCoord[i * 2] = (TexCoords_t() << 0, 0, 1, 0, 1, 0).finished();
            texCoord[i * 2 + 1] = (TexCoords_t() << 1, 1, 0, 1, 0, 1).finished();
        }

        return std::make_tuple(matPfc, faces, tsInd, texCoord, arrTextureShaders);
    };

    Vec3f const faceRotation[6] = {{0, 0, 0}, {0, 90, 0}, {180, 0, 0}, {0, 270, 0}, {90, 0, 0}, {270, 0, 0}};
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

class Tile : public MeshObject<Tile> {
  public:
    Tile(Vec3f const &centre) : MeshObject<Tile>(centre) {}

    static constexpr int NUM_VERTICES{4};
    static constexpr int NUM_FACES{2};
    static constexpr float size = 0.2;
    static constexpr float vertices[NUM_VERTICES][3] = {
        {-size, -size, 0}, {size, -size, 0}, {size, size, 0}, {-size, size, 0}};
    static constexpr int faces[NUM_FACES][3] = {{0, 1, 2}, {2, 3, 0}};
    static constexpr float faceRotation[2][3] = {{0, 0, 0}, {0, 180, 0}};

    BasicShader arrTextureShaders[1] = {BasicShader{colRGB{high, low, low}, colRGB{low, high, low}}};

    inline auto getMesh(Camera const &camera) {
        // Rotate base vertices and offset from camera
        using matVertex = Eigen::Matrix<float, 3, NUM_VERTICES>;
        
        matVertex const matPfc =
            camera.counterRotation * (this->rotate(matVertex::Map(vertices[0]), this->_rotation).colwise() + (this->_centre - camera.position));

        std::array<int, NUM_FACES*2> tsInd;
        std::array<Object::TexCoords_t, NUM_FACES*2> texCoord;
        for (int i = 0; i < 2; ++i) {
            tsInd[i * 2] = 0;
            tsInd[i * 2 + 1] = 0;
            texCoord[i * 2] = (Object::TexCoords_t() << 0, 0, 1, 0, 1, 0).finished();
            texCoord[i * 2 + 1] = (Object::TexCoords_t() << 1, 1, 0, 1, 0, 1).finished();
        }

        return std::make_tuple(matPfc, faces, tsInd, texCoord, arrTextureShaders);
    };

    void setTarget(int tex) { this->_targetRotation = Vec3f::Map(faceRotation[tex]); }
};
