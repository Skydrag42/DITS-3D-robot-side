#pragma once

#include <cmath>
#include <vector>


void encode_yuv420(const void* source, uint8_t* dest, int width, int height);

void decode_depth_z16(const void* source, uint16_t* dest, int width, int height);
