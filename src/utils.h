#pragma once

#include "tuple"
#include <array>
#include <cmath>
#include <Eigen/Dense>

static constexpr uint16_t IMAGE_SIZE = 1200;

struct Pixel {
    Pixel() = default;
    Pixel(Pixel const&) = default;
    Pixel(float r, float g, float b) {
        _dat[0] = r;
        _dat[1] = g;
        _dat[2] = b;
    }


    Pixel lerp(const Pixel& y, float w) const {
        return Pixel(std::lerp(r(), y.r(), w),
                    std::lerp(g(), y.g(), w),
                    std::lerp(b(), y.b(), w));
    }
    Pixel operator*(float w) const {
        return Pixel(r() * w, g()*w, b()*w);
    }

    float r() const { return _dat[0]; }
    float g() const { return _dat[1]; }
    float b() const { return _dat[2]; }

    float _dat[3]={};
};


template<typename T>
void swapIf(const bool pred, T& v1, T& v2) {
    if (pred) {
        std::swap(v1, v2);
    }
}

template <int16_t GS> class Image {
  public:
    using Pixel_t=Pixel;
    static constexpr int16_t GRID_SIZE{GS};
    static constexpr int32_t ARR_SIZE{GS * GS};

    constexpr static int32_t POS(int16_t i, int16_t j) { return i + GRID_SIZE * j; };
    std::array<Pixel, ARR_SIZE> image{};
};

template <int16_t N = 256> class Palette {
    static float square(const float x) { return x * x; };
    constexpr static auto paletteFns = std::make_tuple(
        [](const float idx) {
            return Pixel(std::exp(-square(idx - 0.3) * 20), std::exp(-square(idx - 0.6) * 20),
                         std::exp(-square(idx - 0.7) * 20));
        },
        [](const float idx) {
            return Pixel(std::exp(-square(idx - 0.8) * 10), std::exp(-square(idx - 0.5) * 10),
                         std::exp(-square(idx - 0.4) * 5));
        });

  public:
    static constexpr int16_t SIZE{N};
    Palette() { updatePalette(); };

    void nextPalette() {
        mPaletteId = (mPaletteId + 1) % std::tuple_size_v<decltype(paletteFns)>;
        updatePalette();
    }

    Pixel &operator()(float f) { return mPalette[static_cast<int>(f * SIZE) & (SIZE - 1)]; }

  private:
    void updatePalette() {
        for (int i = 1; i < SIZE; ++i) {
            []<std::size_t... I>(const auto &fn, const auto &tup, std::index_sequence<I...>) {
                (fn(std::get<I>(tup), I), ...);
            }
            (
                [&](const auto &t, const size_t ind) {
                    if (ind == mPaletteId)
                        mPalette[i] = t(static_cast<float>(i) / SIZE);
                    return 0;
                },
                paletteFns, std::make_index_sequence<std::tuple_size_v<decltype(paletteFns)>>());
        }
    }

    size_t mPaletteId{};
    Pixel mPalette[SIZE];
};

// random float
inline float randf(float max) { return (rand() % (int)(max * 100)) / 100.0; };

// random float centered on zero
inline float randfc(float max) { return (rand() % (int)(max * 100)) / 100.0 - max / 2.0; };

