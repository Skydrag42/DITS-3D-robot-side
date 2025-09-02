#include "encode_z16.h"
#include <cmath>

void encode_yuv420(const void* source, uint8_t* dest, int width, int height)
{
    // Paramètres selon le papier de recherche
    int w = 4096;
    int np = 64;
    float p = (float)np / (float)w;

    // Index pour les composantes U et V (sous-échantillonnage 4:2:0)
    int uv_index = 0;

    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            // Lecture de la valeur de profondeur 16-bit
            uint16_t depth = ((uint16_t*)source)[x + width * y];


            float L = ((float)depth + 0.5f) / (float)w;

            // Onde triangulaire Ha(d)
            float phase_a = L / (p / 2.0f);
            float Ha = fmodf(phase_a, 2.0f);
            if (Ha > 1.0f) Ha = 2.0f - Ha;

            // Onde triangulaire Hb(d) - déphasée de p/4
            float phase_b = (L - (p / 4.0f)) / (p / 2.0f);
            float Hb = fmodf(phase_b, 2.0f);
            if (Hb > 1.0f) Hb = 2.0f - Hb;

            // Composante Y (luminance) - L mappé sur [0, 255]
            dest[x + width * y] = (uint8_t)(L * 255.0f);

            // Sous-échantillonnage 4:2:0 pour U et V
            if (y % 2 == 0 && x % 2 == 0)
            {
                // Composante U (Ha)
                dest[width * height + uv_index] = (uint8_t)(Ha * 255.0f);
                // Composante V (Hb)
                dest[width * height + (width * height / 4) + uv_index] = (uint8_t)(Hb * 255.0f);
                uv_index++;
            }
        }
    }
}

void decode_depth_z16(const void* source, uint16_t* dest, int width, int height)
{
    // Mêmes paramètres que l'encodage
    int w = 4096;           // 2^16
    int np = 64;
    float p = (float)np / (float)w;

    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            // Lecture des composantes YUV
            float L_bar = ((uint8_t*)source)[x + width * y] / 255.0f;

            // Indices pour les composantes U et V (sous-échantillonnage 4:2:0)
            int u_idx = (x / 2) + (y / 2) * (width / 2);
            int v_idx = (x / 2) + (y / 2) * (width / 2);

            float Ha_bar = ((uint8_t*)source)[width * height + u_idx] / 255.0f;
            float Hb_bar = ((uint8_t*)source)[width * height + (width * height / 4) + v_idx] / 255.0f;

            // Calcul de m(L̄) selon l'équation du papier
            float m_float = 4.0f * L_bar / p - 0.5f;
            int m = ((int)floorf(m_float)) % 4;
            if (m < 0) m += 4;  // Gestion des valeurs négatives

            // Calcul de L0(L̄) selon l'équation du papier
            float mod_term = fmodf(L_bar - (p / 8.0f), p);
            if (mod_term < 0.0f) mod_term += p;  // Assurer une valeur positive

            float L0 = L_bar - mod_term + (p / 4.0f) * (float)m - (p / 8.0f);

            // Calcul du delta δ(L̄,H̄a,H̄b) selon l'équation du papier
            float delta = 0.0f;
            switch (m)
            {
            case 0:
                delta = (p / 2.0f) * Ha_bar;
                break;
            case 1:
                delta = (p / 2.0f) * Hb_bar;
                break;
            case 2:
                delta = (p / 2.0f) * (1.0f - Ha_bar);
                break;
            case 3:
                delta = (p / 2.0f) * (1.0f - Hb_bar);
                break;
            }

            // Reconstruction finale : d̄ = w · [L0(L̄) + δ(L̄,H̄a,H̄b)]
            float reconstructed_depth = (float)w * (L0 + delta);

            // Clamping pour éviter les débordements
            if (reconstructed_depth < 0.0f) reconstructed_depth = 0.0f;
            if (reconstructed_depth > 65535.0f) reconstructed_depth = 65535.0f;

            dest[x + width * y] = (uint16_t)reconstructed_depth;
        }
    }
}