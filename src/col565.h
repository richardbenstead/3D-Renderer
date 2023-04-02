#pragma once

inline uint_fast8_t lerp8(uint_fast8_t a, uint_fast8_t b, float progress) {
    // Cast to int, avoid horrible values when b-a is less than zero
    return a + (int_fast8_t)((int_fast8_t)b - (int_fast8_t)a) * progress;
}

using col565 = uint16_t;

static col565 makeCol565(uint8_t r, uint8_t g, uint8_t b) { return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3); }

static std::tuple<float, float, float> fromColor565(col565 col) {
    uint8_t baseR = (col & 0xf800) >> 8;
    uint8_t baseG = (col & 0x07e0) >> 3;
    uint8_t baseB = (col & 0x001f) << 3;
    return {static_cast<float>(baseR) / 0xF8, static_cast<float>(baseG) / 0xFC, static_cast<float>(baseB) / 0xF8};
}

uint16_t lerpCol(col565 const col1, col565 const col2, float const progress) {
    uint8_t baseR = (col1 & 0xf800) >> 8;
    uint8_t baseG = (col1 & 0x07e0) >> 3;
    uint8_t baseB = (col1 & 0x001f) << 3;

    uint8_t tgtR = (col2 & 0xf800) >> 8;
    uint8_t tgtG = (col2 & 0x07e0) >> 3;
    uint8_t tgtB = (col2 & 0x001f) << 3;

    uint8_t r = lerp8(baseR, tgtR, progress);
    uint8_t g = lerp8(baseG, tgtG, progress);
    uint8_t b = lerp8(baseB, tgtB, progress);
    return makeCol565(r, g, b);
}
