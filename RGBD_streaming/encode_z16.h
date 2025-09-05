#pragma once

#include <cmath>
#include <vector>


void initialize_z16_tables();

void encode_yuv420_fast(const void* source, uint8_t* dest, int width, int height);

void decode_depth_z16_fast(const void* source, uint16_t* dest, int width, int height);

void initialize_delta_table();

void decode_depth_z16_ultra_fast(const void* source, uint16_t* dest, int width, int height);

void encode_yuv420(const void* source, uint8_t* dest, int width, int height);

void decode_depth_z16(const void* source, uint16_t* dest, int width, int height);