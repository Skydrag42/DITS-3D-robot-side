# DITS-3D-robot-side

This is the robot side repo of the DITS-3D project (https://github.com/Skydrag42/DITS-3D) :
 It contains  : 
 - A python server and scripts to control the poppy robot using dynamixel SDK.
 - A C++ server that encodes and distributes over the network RGBD stream from the intel realsense camera. It uses GStreamer (https://gstreamer.freedesktop.org/) for h264 enoding and rtp transport and a pre-encoding that we implemented to transform RGBD stream into a YUV formated stream compatible with Gstreamer. The transformation from the Depth flow to YUV stream is implemented according to this research document (http://reality.cs.ucl.ac.uk/projects/depth-streaming/depth-streaming.pdf). An important part of the C++ application Gstreamer is inspired by this tutorial :   
https://www.it-jim.com/blog/gstreamer-cpp-tutorial/

# Install 
In order to install this project you will need : 
- Gstreamer (https://gstreamer.freedesktop.org/download/#windows) (If you have trouble installing Gstreamer see this repo : https://github.com/stuw-u/GStreamerUnityHardwareDecoding/tree/main)
- OpenCV
- librealsense

Pour utiliser les codes DITS-3D-robot-side : 

Telecharger la release disponible sous github, extraire le contenu du .zip et placer le dossier extrait 'out' dans le dossier RGBD_streaming.
```bash
git clone https://github.com/Skydrag42/DITS-3D-robot-side.git

cd .\DITS-3D-robot-side\python_master_server 

python -m venv venv

.\venv\Scripts\activate

pip install opencv-python dynamixel_sdk matplotlib numpy 

# Brancher le robot en alimentation puis en USB-A à l'ordinateur

python .\master_server.py
```
Le programme master_server est maintenant lancé et est en attente d'instruction  réseau de la part du programme utilisateur. 

# Authors 
This project was made by J. Charvet and R. Debache. during an internship at ENSEIRB-MATMECA (https://enseirb-matmeca.bordeaux-inp.fr/fr).
