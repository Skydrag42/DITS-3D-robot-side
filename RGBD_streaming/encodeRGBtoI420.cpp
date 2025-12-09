#include <cstdint>
#include <algorithm> // std::min, std::max

#include "libyuv.h"

#pragma comment(lib, "yuv.lib")



void encodeRGBAtoI420_libyuv(const uint8_t* srcRGBA, uint8_t* dstI420, int width, int height) {
    // Calculate plane pointers
    uint8_t* y_plane = dstI420;
    uint8_t* u_plane = dstI420 + (width * height);
    uint8_t* v_plane = dstI420 + (width * height) + (width * height / 4);

    // Single libyuv function call replaces all your loops!
    int result = libyuv::ARGBToI420(
        srcRGBA, width*4,        // Source RGBA data and stride
        y_plane, width,            // Y plane and stride  
        u_plane, width/2,        // U plane and stride
        v_plane, width/2,        // V plane and stride
        width, height
    );

}

void decodeI420toRGBA_libyuv(const uint8_t* srcI420, uint8_t* dstRGBA, int width, int height) {
    // Calculate plane pointers from I420 buffer
    const uint8_t* y_plane = srcI420;
    const uint8_t* u_plane = srcI420 + (width * height);
    const uint8_t* v_plane = srcI420 + (width * height) + (width * height / 4);

    int result = libyuv::I420ToARGB(
        y_plane, width,            // Y plane and stride
        u_plane, width / 2,        // U plane and stride
        v_plane, width / 2,        // V plane and stride
        dstRGBA, width * 4,        // Destination RGBA data and stride
        width, height
    );
}

// --- Encodage RGBA -> I420 ---
void encodeRGBAtoI420(const uint8_t* srcRGBA, uint8_t* dstI420, int width, int height) {
    const int W = width;
    const int H = height;

    uint8_t* Yp = dstI420;
    uint8_t* Up = dstI420 + W * H;
    uint8_t* Vp = Up + (W / 2) * (H / 2);

    // --- Plan Y ---
    for (int y = 0; y < H; ++y) {
        const int rowStart = y * W * 4;
        for (int x = 0; x < W; ++x) {
            const int idx = rowStart + x * 4;
            const int R = srcRGBA[idx + 0];
            const int G = srcRGBA[idx + 1];
            const int B = srcRGBA[idx + 2];

            // Y = 0.299*R + 0.587*G + 0.114*B (approximation entière)
            Yp[y * W + x] = (77 * R + 150 * G + 29 * B + 128) >> 8;
        }
    }

    // --- Plan U/V ---
    for (int y = 0; y < H; y += 2) {
        const int row0 = y * W * 4;
        const int row1 = (y + 1) * W * 4;
        for (int x = 0; x < W; x += 2) {
            int Rsum = 0, Gsum = 0, Bsum = 0;
            int idxs[4] = { row0 + x * 4, row0 + (x + 1) * 4, row1 + x * 4, row1 + (x + 1) * 4 };
            for (int i = 0; i < 4; ++i) {
                Rsum += srcRGBA[idxs[i] + 0];
                Gsum += srcRGBA[idxs[i] + 1];
                Bsum += srcRGBA[idxs[i] + 2];
            }

            // Moyenne des 4 pixels
            int Ravg = Rsum / 4;
            int Gavg = Gsum / 4;
            int Bavg = Bsum / 4;

            // U/V BT.601 full range (approximation entière)
            int U = ((-43 * Ravg - 84 * Gavg + 128 * Bavg + 128) >> 8) + 128;
            int V = ((128 * Ravg - 107 * Gavg - 21 * Bavg + 128) >> 8) + 128;

            U = std::clamp(U, 0, 255);
            V = std::clamp(V, 0, 255);

            const int ci = (y / 2) * (W / 2) + (x / 2);
            Up[ci] = static_cast<uint8_t>(U);
            Vp[ci] = static_cast<uint8_t>(V);
        }
    }
}

// --- Décodage I420 -> RGBA ---
void decodeI420toRGBA(const uint8_t* srcI420, uint8_t* dstRGBA, int width, int height) {
    const int W = width;
    const int H = height;

    const uint8_t* Yp = srcI420;
    const uint8_t* Up = srcI420 + W * H;
    const uint8_t* Vp = Up + (W / 2) * (H / 2);

    for (int y = 0; y < H; ++y) {
        const int uvRow = (y / 2) * (W / 2);
        for (int x = 0; x < W; ++x) {
            const int Y = Yp[y * W + x];
            const int U = Up[uvRow + x / 2] - 128;
            const int V = Vp[uvRow + x / 2] - 128;


            int R = Y + ((179 * V) >> 7);
            int G = Y - ((44 * U + 91 * V) >> 7);
            int B = Y + ((227 * U) >> 7);

            R = std::clamp(R, 0, 255);
            G = std::clamp(G, 0, 255);
            B = std::clamp(B, 0, 255);

            const int idx = (y * W + x) * 4;
            dstRGBA[idx + 0] = static_cast<uint8_t>(R);
            dstRGBA[idx + 1] = static_cast<uint8_t>(G);
            dstRGBA[idx + 2] = static_cast<uint8_t>(B);
            dstRGBA[idx + 3] = 255; // alpha opaque
        }
    }
}

// Concatène deux frames I420 verticalement : frame1 sur le dessus, frame2 en dessous 
void concatI420Vertical(const uint8_t* frame1, const uint8_t* frame2, uint8_t* dest, int width, int height) 
{ 
    const int Ysize = width * height; const int Usize = (width / 2) * (height / 2); const int Vsize = Usize; 
    // Copier Y 
    memcpy(dest, frame1, Ysize); 
    memcpy(dest + Ysize, frame2, Ysize); 
    // Copier U 
    memcpy(dest + 2 * Ysize, frame1 + Ysize, Usize); 
    memcpy(dest + 2 * Ysize + Usize, frame2 + Ysize, Usize); 
    // Copier V 
    memcpy(dest + 2 * Ysize + 2 * Usize, frame1 + Ysize + Usize, Vsize); 
    memcpy(dest + 2 * Ysize + 2 * Usize + Vsize, frame2 + Ysize + Usize, Vsize); } 

// Déconcatène une frame I420 verticale en deux frames : frame1 en haut, frame2 en bas 
void deconcatI420Vertical(const uint8_t* concatFrame, uint8_t* frame1, uint8_t* frame2, int width, int height) 
{ 
    const int Ysize = width * height; const int Usize = (width / 2) * (height / 2);
    const int Vsize = Usize; 
    // Copier Y 
    memcpy(frame1, concatFrame, Ysize); 
    memcpy(frame2, concatFrame + Ysize, Ysize); 
    // Copier U 
    memcpy(frame1 + Ysize, concatFrame + 2 * Ysize, Usize); 
    memcpy(frame2 + Ysize, concatFrame + 2 * Ysize + Usize, Usize); 
    // Copier V 
    memcpy(frame1 + Ysize + Usize, concatFrame + 2 * Ysize + 2 * Usize, Vsize); 
    memcpy(frame2 + Ysize + Usize, concatFrame + 2 * Ysize + 2 * Usize + Vsize, Vsize); 
}