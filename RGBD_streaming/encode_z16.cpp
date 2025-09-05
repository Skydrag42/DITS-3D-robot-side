#include "encode_z16.h"
#include <cmath>

// Nouveau encode z_16
// permet de tester les valeurs de g_w et g_np face à la compression


// =======================
// Paramètres globaux
// =======================
static int g_w = 4096;   // Valeur par défaut
static int g_np = 512;    // Valeur par défaut

void set_z16_params(int w, int np)
{
    g_w = w;
    g_np = np;
}

// =======================
// Tables globales
// =======================
static float L_table[65536];
static uint8_t Y_table[65536];
static uint8_t Ha_table[65536];
static uint8_t Hb_table[65536];

static float L_bar_table[256];
static int m_table[256];
static float L0_base_table[256];
static float mod_table[256];

static float delta_table[4][256];

// =======================
// Initialisation des tables
// =======================
void initialize_z16_tables()
{
    const int w = g_w;
    const int np = g_np;
    const float p = (float)np / (float)w;
    const float p_half = p / 2.0f;
    const float p_quarter = p / 4.0f;
    const float p_eighth = p / 8.0f;

    for (int depth = 0; depth < 65536; depth++)
    {
        float L = ((float)depth + 0.5f) / (float)w;
        L_table[depth] = L;
        Y_table[depth] = (uint8_t)(L * 255.0f);

        float phase_a = L / p_half;
        float Ha = fmodf(phase_a, 2.0f);
        if (Ha > 1.0f) Ha = 2.0f - Ha;
        Ha_table[depth] = (uint8_t)(Ha * 255.0f);

        float phase_b = (L - p_quarter) / p_half;
        float Hb = fmodf(phase_b, 2.0f);
        if (Hb > 1.0f) Hb = 2.0f - Hb;
        Hb_table[depth] = (uint8_t)(Hb * 255.0f);
    }

    for (int i = 0; i < 256; i++)
    {
        float L_bar = (float)i / 255.0f;
        L_bar_table[i] = L_bar;

        float m_float = 4.0f * L_bar / p - 0.5f;
        int m = ((int)floorf(m_float)) % 4;
        if (m < 0) m += 4;
        m_table[i] = m;

        float mod_term = fmodf(L_bar - p_eighth, p);
        if (mod_term < 0.0f) mod_term += p;
        mod_table[i] = mod_term;
        L0_base_table[i] = L_bar - mod_term - p_eighth;
    }
}

void initialize_delta_table()
{
    const int w = g_w;
    const int np = g_np;
    const float p = (float)np / (float)w;
    const float p_half = p / 2.0f;

    for (int i = 0; i < 256; i++)
    {
        float val = (float)i / 255.0f;
        delta_table[0][i] = p_half * val;
        delta_table[1][i] = p_half * val;
        delta_table[2][i] = p_half * (1.0f - val);
        delta_table[3][i] = p_half * (1.0f - val);
    }
}

// =======================
// Encodeurs
// =======================
void encode_yuv420_fast(const void* source, uint8_t* dest, int width, int height)
{
    const uint16_t* src = (const uint16_t*)source;
    int uv_index = 0;

    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            uint16_t depth = src[x + width * y];
            dest[x + width * y] = Y_table[depth];

            if (y % 2 == 0 && x % 2 == 0)
            {
                dest[width * height + uv_index] = Ha_table[depth];
                dest[width * height + (width * height / 4) + uv_index] = Hb_table[depth];
                uv_index++;
            }
        }
    }
}

void encode_yuv420(const void* source, uint8_t* dest, int width, int height)
{
    const int w = g_w;
    const int np = g_np;
    const float p = (float)np / (float)w;

    int uv_index = 0;

    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            uint16_t depth = ((uint16_t*)source)[x + width * y];
            float L = ((float)depth + 0.5f) / (float)w;

            float phase_a = L / (p / 2.0f);
            float Ha = fmodf(phase_a, 2.0f);
            if (Ha > 1.0f) Ha = 2.0f - Ha;

            float phase_b = (L - (p / 4.0f)) / (p / 2.0f);
            float Hb = fmodf(phase_b, 2.0f);
            if (Hb > 1.0f) Hb = 2.0f - Hb;

            dest[x + width * y] = (uint8_t)(L * 255.0f);

            if (y % 2 == 0 && x % 2 == 0)
            {
                dest[width * height + uv_index] = (uint8_t)(Ha * 255.0f);
                dest[width * height + (width * height / 4) + uv_index] = (uint8_t)(Hb * 255.0f);
                uv_index++;
            }
        }
    }
}

// =======================
// Décodeurs
// =======================
void decode_depth_z16_fast(const void* source, uint16_t* dest, int width, int height)
{
    const int w = g_w;
    const int np = g_np;
    const float p = (float)np / (float)w;
    const float p_quarter = p / 4.0f;
    const float p_half = p / 2.0f;

    const uint8_t* src = (const uint8_t*)source;

    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            uint8_t y_val = src[x + width * y];
            float L_bar = L_bar_table[y_val];

            int u_idx = (x / 2) + (y / 2) * (width / 2);
            int v_idx = u_idx;

            uint8_t u_val = src[width * height + u_idx];
            uint8_t v_val = src[width * height + (width * height / 4) + v_idx];

            float Ha_bar = L_bar_table[u_val];
            float Hb_bar = L_bar_table[v_val];

            int m = m_table[y_val];
            float L0 = L0_base_table[y_val] + p_quarter * (float)m;

            float delta;
            switch (m)
            {
            case 0: delta = p_half * Ha_bar; break;
            case 1: delta = p_half * Hb_bar; break;
            case 2: delta = p_half * (1.0f - Ha_bar); break;
            case 3: delta = p_half * (1.0f - Hb_bar); break;
            default: delta = 0.0f; break;
            }

            float reconstructed = (float)w * (L0 + delta);
            reconstructed = fmaxf(0.0f, fminf(65535.0f, reconstructed));

            dest[x + width * y] = (uint16_t)reconstructed;
        }
    }
}

void decode_depth_z16_ultra_fast(const void* source, uint16_t* dest, int width, int height)
{
    const int w = g_w;
    const int np = g_np;
    const float p_quarter = (float)np / (float)w / 4.0f;

    const uint8_t* src = (const uint8_t*)source;

    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            uint8_t y_val = src[x + width * y];

            int u_idx = (x / 2) + (y / 2) * (width / 2);
            uint8_t u_val = src[width * height + u_idx];
            uint8_t v_val = src[width * height + (width * height / 4) + u_idx];

            int m = m_table[y_val];
            float L0 = L0_base_table[y_val] + p_quarter * (float)m;

            float delta;
            if (m <= 1)
                delta = delta_table[m][m == 0 ? u_val : v_val];
            else
                delta = delta_table[m][m == 2 ? u_val : v_val];

            float reconstructed = (float)w * (L0 + delta);
            reconstructed = fmaxf(0.0f, fminf(65535.0f, reconstructed));

            dest[x + width * y] = (uint16_t)reconstructed;
        }
    }
}

void decode_depth_z16(const void* source, uint16_t* dest, int width, int height)
{
    const int w = g_w;
    const int np = g_np;
    const float p = (float)np / (float)w;

    const uint8_t* src = (const uint8_t*)source;

    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            float L_bar = src[x + width * y] / 255.0f;

            int u_idx = (x / 2) + (y / 2) * (width / 2);
            int v_idx = (x / 2) + (y / 2) * (width / 2);

            float Ha_bar = src[width * height + u_idx] / 255.0f;
            float Hb_bar = src[width * height + (width * height / 4) + v_idx] / 255.0f;

            float m_float = 4.0f * L_bar / p - 0.5f;
            int m = ((int)floorf(m_float)) % 4;
            if (m < 0) m += 4;

            float mod_term = fmodf(L_bar - (p / 8.0f), p);
            if (mod_term < 0.0f) mod_term += p;

            float L0 = L_bar - mod_term + (p / 4.0f) * (float)m - (p / 8.0f);

            float delta = 0.0f;
            switch (m)
            {
            case 0: delta = (p / 2.0f) * Ha_bar; break;
            case 1: delta = (p / 2.0f) * Hb_bar; break;
            case 2: delta = (p / 2.0f) * (1.0f - Ha_bar); break;
            case 3: delta = (p / 2.0f) * (1.0f - Hb_bar); break;
            }

            float reconstructed = (float)w * (L0 + delta);
            reconstructed = fmaxf(0.0f, fminf(65535.0f, reconstructed));

            dest[x + width * y] = (uint16_t)reconstructed;
        }
    }
}
