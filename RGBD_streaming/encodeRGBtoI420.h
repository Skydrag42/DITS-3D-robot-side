#pragma once

#include <cmath>
#include <vector>
#include <algorithm>

void encodeRGBAtoI420(const uint8_t* srcRGBA, uint8_t* dstI420, int width, int height);

void decodeI420toRGBA(const uint8_t* srcI420, uint8_t* dstRGBA, int width, int height);

void concatI420Vertical(const uint8_t* frame1, const uint8_t* frame2, uint8_t* dest, int width, int height);

void deconcatI420Vertical(const uint8_t* concatFrame, uint8_t* frame1, uint8_t* frame2, int width, int height);

void encodeRGBAtoI420_libyuv(const uint8_t* srcRGBA, uint8_t* dstI420, int width, int height);

void decodeI420toRGBA_libyuv(const uint8_t* srcI420, uint8_t* dstRGBA, int width, int height);