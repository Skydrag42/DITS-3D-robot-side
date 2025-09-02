#include <iostream>
#include <vector>
#include <cstdint>
#include <cmath>
#include <algorithm>
#include <limits>
#include <functional>
#include "encode_z16.h"  // contient encode_yuv420 et decode_depth_z16

// Renomer ce fichier : Test_z16_yuv.cpp

// --- Génération de données de profondeur synthétiques ---
void generateSyntheticDepth(std::vector<uint16_t>& depth, int width, int height) {
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            float fx = static_cast<float>(x) / width;
            float fy = static_cast<float>(y) / height;
            float val = (fx + fy + 0.5f * std::sin(10 * fx * fy)) * 4096.0f;
            if (val < 0) val = 0;
            if (val > 4095) val = 4095;  // limitation à la plage effective
            depth[y * width + x] = static_cast<uint16_t>(val);
        }
    }
}

void generateHorizontalRamp(std::vector<uint16_t>& depth, int width, int height) {
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            depth[y * width + x] = static_cast<uint16_t>((x * 4095) / (width - 1));
        }
    }
}

void generateVerticalRamp(std::vector<uint16_t>& depth, int width, int height) {
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            depth[y * width + x] = static_cast<uint16_t>((y * 4095) / (height - 1));
        }
    }
}

void generateConstantDepth(std::vector<uint16_t>& depth, int width, int height, uint16_t value) {
    std::fill(depth.begin(), depth.end(), value);
}

// --- Comparaison entre original et reconstruit ---
void evaluate(const std::vector<uint16_t>& original, const std::vector<uint16_t>& reconstructed,
    const std::string& label) {
    uint64_t totalDiff = 0;
    uint16_t maxDiff = 0;
    size_t count = original.size();

    uint16_t minVal = std::numeric_limits<uint16_t>::max();
    uint16_t maxVal = 0;

    for (size_t i = 0; i < count; i++) {
        int diff = std::abs((int)original[i] - (int)reconstructed[i]);
        totalDiff += diff;
        if (diff > maxDiff) maxDiff = diff;
        if (reconstructed[i] < minVal) minVal = reconstructed[i];
        if (reconstructed[i] > maxVal) maxVal = reconstructed[i];
    }

    double avgDiff = static_cast<double>(totalDiff) / count;

    std::cout << "===== Résultats du test: " << label << " =====\n";
    std::cout << "Nombre de pixels : " << count << "\n";
    std::cout << "Erreur moyenne   : " << avgDiff << "\n";
    std::cout << "Erreur max       : " << maxDiff << "\n";
    std::cout << "Plage attendue   : 0..4095\n";
    std::cout << "Valeurs reconstruites: min=" << minVal << " max=" << maxVal << "\n";
    std::cout << "============================================\n\n";
}

// --- Pipeline encode/décode/test ---
void runTest(std::function<void(std::vector<uint16_t>&, int, int)> generator,
    const std::string& label, int width, int height) {
    std::vector<uint16_t> depthIn(width * height);
    std::vector<uint16_t> depthOut(width * height);
    std::vector<uint8_t> yuv420(width * height * 3 / 2);

    generator(depthIn, width, height);  // génération de l’image de test

    encode_yuv420(depthIn.data(), yuv420.data(), width, height);
    decode_depth_z16(yuv420.data(), depthOut.data(), width, height);

    evaluate(depthIn, depthOut, label);
}

int main() {
    const int width = 1280;
    const int height = 720;

    runTest(generateSyntheticDepth, "Sinusoïde + diagonale", width, height);
    runTest(generateHorizontalRamp, "Rampe horizontale", width, height);
    runTest(generateVerticalRamp, "Rampe verticale", width, height);

    // Cas spéciaux : on utilise un lambda qui fixe la valeur constante
    runTest([&](std::vector<uint16_t>& d, int w, int h) { generateConstantDepth(d, w, h, 1000); }, "Constant (1000)", width, height);
    runTest([&](std::vector<uint16_t>& d, int w, int h) { generateConstantDepth(d, w, h, 2000); }, "Constant (2000)", width, height);
    runTest([&](std::vector<uint16_t>& d, int w, int h) { generateConstantDepth(d, w, h, 3500); }, "Constant (3500)", width, height);

    return 0;
}
