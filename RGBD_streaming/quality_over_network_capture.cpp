// rs_tests.cpp : définit le point d'entrée de l'application.
//

#include "rs_tests.h"
#include "encode_z16.h"
#include "encodeRGBtoI420.h"
#include <chrono>
#include <thread>
#include <map>
#include <opencv2/opencv.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <fstream>
#include <string>

#define WIDTH 1280
#define HEIGHT 720

int record_raw() {

    std::cout << "Starting raw recording program" << std::endl;
    rs2::pipeline rs_pipe;
    rs2::config rs_cfg;
    rs_cfg.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, 30);
    rs_cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_RGBA8, 30);

    rs2::pipeline_profile selection = rs_pipe.start(rs_cfg);

    rs2::device selected_device = selection.get_device();
    auto depth_sensor = selected_device.first<rs2::depth_sensor>();

    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f);
    if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
        depth_sensor.set_option(RS2_OPTION_LASER_POWER, depth_sensor.get_option_range(RS2_OPTION_LASER_POWER).max);

    rs2::decimation_filter dec_filter;
    rs2::threshold_filter thr_filter;
    rs2::spatial_filter spatial_filter;
    rs2::temporal_filter temporal_filter;
    rs2::hole_filling_filter hole;

    // === Spatial Filter ===
    spatial_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
    spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5f);
    spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);
    spatial_filter.set_option(RS2_OPTION_HOLES_FILL, 2);

    // === Temporal Filter ===
    temporal_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.4f);
    temporal_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);
    temporal_filter.set_option(RS2_OPTION_HOLES_FILL, 1);

    // === Hole Filling Filter ===
    hole.set_option(RS2_OPTION_HOLES_FILL, 1);

    std::cout << "Recording to recording to z16 Press ENTER to stop..." << std::endl;
    int i = 0;
    std::string filename_z16 = "z16_video3.raw";
    std::string filename_ARGB = "ARGB_video3.raw";

    // === Writting RAW files ===
    std::ofstream ofs_Z16(filename_z16, std::ios::binary);
    std::ofstream ofs_ARGB(filename_ARGB, std::ios::binary);

    while (true)
    {
        i++;
        auto frames = rs_pipe.wait_for_frames();
        frames = thr_filter.process(frames);
        frames = spatial_filter.process(frames);
        frames = temporal_filter.process(frames);

        auto color = frames.get_color_frame();
        auto depth = frames.get_depth_frame();
        std::cout << "Frame " << i << std::endl;

        
        ofs_Z16.write((char*)depth.get_data(), WIDTH * HEIGHT * 2);  // Z16 = 16 bits per pixel (2*8)
        ofs_ARGB.write((char*)color.get_data(), WIDTH * HEIGHT * 4); // RGBA8 = 4*8 bits per pixel

       
        cv::Mat depthMat(HEIGHT, WIDTH, CV_16U, (uint16_t*)depth.get_data());

        cv::Mat depth8U;
        depthMat.convertTo(depth8U, CV_8U, 255.0 / 4096); // suppose max 10m

        cv::imshow("Depth (Live)", depth8U);

        cv::Mat colorMat(HEIGHT, WIDTH, CV_8UC4, (uint8_t*)color.get_data());
        cv::imshow("Color (Live)", colorMat);



        int key = cv::waitKey(30);
        if (key == 'q' || key == 27)
            break;
    }

    ofs_Z16.close();
    ofs_ARGB.close();

    // === Reading RAW files ===
    std::ifstream ifs_Z16(filename_z16, std::ios::binary);
    std::ifstream ifs_ARGB(filename_ARGB, std::ios::binary);

    if (!ifs_Z16.is_open() || !ifs_ARGB.is_open()) {
        std::cerr << "Impossible d'ouvrir les fichiers RAW" << std::endl;
        return -1;
    }

    size_t frame_size_z16 = WIDTH * HEIGHT * 2;  // uint16_t
    size_t frame_size_argb = WIDTH * HEIGHT * 4; // uint8_t x4

    std::vector<uint8_t> buffer_depth(frame_size_z16);
    std::vector<uint8_t> buffer_color(frame_size_argb);

    while (true)
    {
 
        ifs_Z16.read((char*)buffer_depth.data(), frame_size_z16);
        if (!ifs_Z16) break; // check if end of file.


        ifs_ARGB.read((char*)buffer_color.data(), frame_size_argb);
        if (!ifs_ARGB) break; // check if end of file.


        cv::Mat depthMat(HEIGHT, WIDTH, CV_16U, buffer_depth.data());
        cv::Mat colorMat(HEIGHT, WIDTH, CV_8UC4, buffer_color.data());


        cv::Mat depth8U;
        depthMat.convertTo(depth8U, CV_8U, 255.0 / 4096); // suppose max 10m

        cv::imshow("Depth (Playback)", depth8U);
        cv::imshow("Color (Playback)", colorMat);

        int key = cv::waitKey(30);
        if (key == 'q' || key == 27)
            break;
    }

    ifs_Z16.close();
    ifs_ARGB.close();
    return 0;
}

int main() {
    record_raw();
}
