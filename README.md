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

# Authors 
This project was made by J. Charvet and R. Debache. during an internship at ENSEIRB-MATMECA (https://enseirb-matmeca.bordeaux-inp.fr/fr).
