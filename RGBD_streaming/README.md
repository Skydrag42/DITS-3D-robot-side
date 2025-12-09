# RGBD Streaming

Cette partie du projet s'occupe de la mise sur le réseau du flux RGBD.

**Note :** Une version compilée (Release) est disponible dans la section Releases sur l'interface GitHub du projet.

## Dépendances

Ces codes nécessitent les bibliothèques suivantes :

- **libyuv** – s'installe automatiquement lors de la compilation avec ce fichier CMake ([Documentation libyuv](https://chromium.googlesource.com/libyuv/libyuv/))
- **Intel RealSense SDK 2.0** – pour récupérer le flux RGBD de la caméra ([SDK Intel RealSense](https://github.com/IntelRealSense/librealsense))
- **GStreamer** – pour réaliser la compression vidéo et la mise en réseau par RTP ([Documentation GStreamer](https://gstreamer.freedesktop.org/documentation/))
- **OpenCV** – pour le traitement d'images
- **CMake** (version 3.12 minimum) – pour la compilation
- **pkg-config** – pour la gestion des dépendances

## Installation des dépendances (Windows)

### GStreamer
1. Téléchargez les installateurs GStreamer depuis le [site officiel](https://gstreamer.freedesktop.org/download/)
2. Installez les deux packages :
   - **gstreamer-1.0-msvc-x86_64-X.XX.X.msi** (runtime)
   - **gstreamer-1.0-devel-msvc-x86_64-X.XX.X.msi** (development)
3. Ajoutez les chemins d'installation aux variables d'environnement système

### Intel RealSense SDK 2.0
1. Téléchargez l'installateur depuis [Intel RealSense Releases](https://github.com/IntelRealSense/librealsense/releases)
2. Installez le SDK dans le répertoire par défaut : `C:\Program Files (x86)\Intel RealSense SDK 2.0`
3. Le CMakeLists.txt est configuré pour ce chemin par défaut

### OpenCV
1. Téléchargez OpenCV depuis le [site officiel](https://opencv.org/releases/)
2. Installez et configurez la variable d'environnement `OpenCV_DIR`

## Compilation

### Configuration et compilation

1. Clonez le projet et naviguez dans le répertoire :
```bash
cd RGBD_streaming
```

2. Créez un répertoire de build :
```bash
mkdir build
cd build
```

3. Générez les fichiers de compilation avec CMake :
```bash
cmake ..
```

4. Compilez le projet :
```bash
cmake --build . --config Release
```

### Exécutables générés

Après la compilation, les exécutables suivants seront disponibles dans le répertoire `build` :

- **send** – Capture et envoie le flux RGBD sur le réseau
- **receive** – Reçoit et décode le flux RGBD
- **Tests_z16_yuv** – Tests de conversion Z16 vers YUV
- **rs_tests** – Tests du SDK RealSense
- **quality_over_network_capture** – Tests de qualité avec capture
- **quality_over_network_openbag** – Tests de qualité à partir de fichiers bag
- **quality_over_network_save_to_raw** – Sauvegarde des données brutes
- **local_compression** – Tests de compression locale

### Notes importantes

- **libyuv** sera téléchargé et compilé automatiquement lors de la première compilation s'il n'est pas trouvé localement
- Sur Windows, la DLL `realsense2.dll` sera automatiquement copiée dans le répertoire de l'exécutable `rs_tests`
- Le projet utilise le standard C++20

## Structure du projet

### Codes d'expérimentation

- `send.cpp`, `send.h`
- `encode_z16.cpp`, `encode_z16.h` – Comprend les fonctions pour la transformation Z16 → YUV d'après [1] et utilise des lookup tables de manière à accélérer le traitement
- `encodeRGBtoI420.cpp`, `encodeRGBtoI420.h` – Fonctions utilisant la bibliothèque libyuv pour convertir les frames au format RGB vers le format I420 pour préparer à la compression vidéo, et fonction pour concaténer ou déconcaténer des frames

### Codes de test de qualité

- `rs_test.cpp`, `rs_test.h`
- `quality_over_network...`
- `local_compression.cpp`

## Fonctionnement général de send

Inspiré (fortement) de [ce tutoriel GStreamer en C++](https://www.it-jim.com/blog/gstreamer-cpp-tutorial/) pour l'utilisation de GStreamer.

On récupère les frames depuis la caméra en utilisant le SDK Intel RealSense. Il y a 2 types de frames :

- **Frames couleurs** au format RGB (on utilise ARGB parfois)
- **Frames de profondeur** (depth map) au format Z16 (encodent la profondeur sur 16 bits par pixel)

Les frames RGB sont transformées en YUV, les frames profondeur Z16 sont transformées en YUV, puis les frames après transformations sont concaténées verticalement (pour assurer leur synchronisation à la réception). On utilise GStreamer pour encoder en H.264 les frames et les envoyer sur le réseau en RTP.

![schéma achitecture](img1.png)  
![schéma architecture](img2.png)

### Configuration de l'encodage vidéo

Cette ligne de code permet de choisir les paramètres d'encodage vidéo H.264 :

```cpp
gchar* pipeStr = g_strdup_printf(
    "appsrc is-live=true name=mysrc format=time "
    "caps=video/x-raw,format=I420 ! "
    "queue max-size-buffers=1 leaky=downstream ! "
    "videoconvert ! "
    "nvh264enc repeat-sequence-header=true preset=low-latency tune=ultra-low-latency zerolatency=true "
    "rc-mode=cbr bitrate=20000 gop-size=1 bframes=0 cabac=false "
    "qp-min-i=20 qp-max-i=30 qp-min-p=20 qp-max-p=30 !"
    " video/x-h264,profile=baseline ! "
    "rtph264pay config-interval=1 pt=96 mtu=1200 ! "
    "queue max-size-buffers=1 leaky=downstream ! "
    "udpsink host=%s port=5000 sync=false async=false",
    host
);
```

Les paramètres ont été choisis de manière à minimiser la latence.

## Références

[1] Pece, F., Kautz, J., & Weyrich, T. (2011). "Adapting Standard Video Codecs for Depth Streaming". [https://jankautz.com/publications/depth-streaming.pdf](https://jankautz.com/publications/depth-streaming.pdf)