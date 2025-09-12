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

int benchmark()
{
    std::cout << "Starting program" << std::endl;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline rs_pipe;
    rs2::config rs_cfg;
    // Use a configuration object to request depth from the pipeline
    rs_cfg.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, 30);
    rs_cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_RGBA8, 30);
    // Start streaming with the above configuration
    rs2::pipeline_profile selection = rs_pipe.start(rs_cfg);

    // enable laser max power
    rs2::device selected_device = selection.get_device();
    auto depth_sensor = selected_device.first<rs2::depth_sensor>();

    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
    {
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter
    }
    if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
    {
        // Query min and max values:
        auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
        depth_sensor.set_option(RS2_OPTION_LASER_POWER, range.max); // Set max power
    }

    // Declare filters
    rs2::decimation_filter dec_filter;  // Decimation - reduces depth frame density
    rs2::threshold_filter thr_filter;   // Threshold  - removes values outside recommended range
    rs2::spatial_filter spat_filter;    // Spatial    - edge-preserving spatial smoothing
    rs2::temporal_filter temp_filter;   // Temporal   - reduces temporal noise

    std::cout << "starting main loop" << std::endl;
    long long time = 0;
    int frameCount = 0;
    int imageSize = WIDTH * HEIGHT;

    uint16_t* image_raw = new uint16_t[imageSize];
    uint8_t* image_enc = new uint8_t[1382400];
    uint16_t* image_dec = new uint16_t[imageSize];

    while (true)
    {

        auto frames = rs_pipe.wait_for_frames();

        frames = thr_filter.process(frames);
        frames = spat_filter.process(frames);
        frames = temp_filter.process(frames);

        auto color = frames.get_color_frame();
        auto depth = frames.get_depth_frame();

        image_raw = (uint16_t*)depth.get_data();

        

        //std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        encode_yuv420(image_raw, image_enc, WIDTH, HEIGHT);

        std::cout << "raw[0]=" << image_raw[100] << ";" << std::endl;
        std::cout << "enc[0]=" << (int)image_enc[100] << ";" << std::endl;
        decode_depth_z16(image_enc, image_dec, WIDTH, HEIGHT);
        std::cout << "dec[0]=" << image_dec[100] << ";" << std::endl;

        //std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

        uint64_t diff = 0;
        uint64_t raw_depth_value = 0;
        uint16_t max = 0;
        uint64_t count = 0;
        float depth_mean = 0;
        for (int y = 0; y < HEIGHT; y++)
        {
            for (int x = 0; x < WIDTH; x++)
            {
                raw_depth_value += image_raw[x + y * WIDTH];
                diff += abs(image_raw[x + y * WIDTH] - image_dec[x + y * WIDTH]);
                if (image_raw[x + y * WIDTH] > max) max = image_raw[x + y * WIDTH];
                if (abs(image_raw[x + y * WIDTH] - image_dec[x + y * WIDTH]) > 30) count+=1;
            }
        }
        diff = diff / (HEIGHT * WIDTH);
        depth_mean = raw_depth_value / (float)(HEIGHT * WIDTH);
        float countp = 100*count / (float)(HEIGHT * WIDTH);

        // cv::Mat img(depth.get_height(), depth.get_width(), CV_8UC3);
        // img.data = image;
        // cv::imshow("Image", img);
        // cv::waitKey(1);


        //time = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();

        //std::cout << "Current time = " << time << std::endl;

        std::cout << "Ecart moyen=" << diff << " / ";
        std::cout << "Profondeur raw moyenne=" << depth_mean << " / ";
        std::cout << "Ecart moyen normalise=" << diff / depth_mean << std::endl;
        std::cout << "Max=" << max << std::endl;
        std::cout << "diffPixel=" << countp << std::endl;

        // Convertir image_raw et image_dec en cv::Mat
        cv::Mat rawMat(HEIGHT, WIDTH, CV_16UC1, image_raw);
        cv::Mat decMat(HEIGHT, WIDTH, CV_16UC1, image_dec);

        // Convertir en 8 bits
        cv::Mat rawMat8U, decMat8U;
        double minVal, maxVal;
        cv::minMaxLoc(rawMat, &minVal, &maxVal);

        // Normalisation identique
        rawMat.convertTo(rawMat8U, CV_8UC1, 255.0 / maxVal);
        decMat.convertTo(decMat8U, CV_8UC1, 255.0 / maxVal);

        // Concaténer horizontalement
        cv::Mat combined;
        cv::hconcat(rawMat8U, decMat8U, combined);

        // === Redimensionner pour que ça tienne dans la fenêtre ===
        // Taille cible en largeur max (par exemple 1280 pixels)
        int target_width = 1280;
        int target_height = 720;

        // Calcul du facteur d’échelle
        double scale = std::min(
            (double)target_width / combined.cols,
            (double)target_height / combined.rows
        );

        // Redimensionner si nécessaire
        cv::Mat combined_resized;
        if (scale < 1.0) {
            cv::resize(combined, combined_resized, cv::Size(), scale, scale, cv::INTER_AREA);
        }
        else {
            combined_resized = combined;
        }

        // Afficher
        cv::imshow("Raw vs Decoded (Left: Raw, Right: Decoded)", combined_resized);
        cv::waitKey(1);


    }

    delete[] image_raw;
    delete[] image_enc;
    delete[] image_dec;
    return 0;
}

int benchmark2() {

    int imageSize = WIDTH * HEIGHT;

    uint16_t max = 0;

    uint16_t* image_raw = new uint16_t[imageSize];

    for (int y = 0; y < HEIGHT; y++)
    {
        for (int x = 0; x < WIDTH; x++)
        {
            image_raw[x + y * WIDTH] = 3000;

            if (image_raw[x + y * WIDTH] > max) max = image_raw[x + y * WIDTH];
        }
    }

    uint8_t* image_enc = new uint8_t[1382400];
    uint16_t* image_dec = new uint16_t[imageSize];

    encode_yuv420(image_raw, image_enc, WIDTH, HEIGHT);

    decode_depth_z16(image_enc, image_dec, WIDTH, HEIGHT);


    uint64_t diff = 0;
    uint64_t raw_depth_value = 0;
    float depth_mean = 0;
    for (int y = 0; y < HEIGHT; y++)
    {
        for (int x = 0; x < WIDTH; x++)
        {
            raw_depth_value += image_raw[x + y * WIDTH];
            diff += abs(image_raw[x + y * WIDTH] - image_dec[x + y * WIDTH]);
        }
    }
    diff = diff / (HEIGHT * WIDTH);
    depth_mean = raw_depth_value / (float)(HEIGHT * WIDTH);


    std::cout << "Ecart moyen=" << diff << " / ";
    std::cout << "Profondeur raw moyenne=" << depth_mean << " / ";
    std::cout << "Ecart moyen normalise=" << diff / depth_mean << std::endl;
    std::cout << "Max=" << max << std::endl;


    delete[] image_dec;
    delete[] image_raw;
    delete[] image_enc;
    return 0;
}

int emit_yuv_from_depth()
{
    std::cout << "Starting program" << std::endl;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline rs_pipe;
    rs2::config rs_cfg;
    // Use a configuration object to request depth from the pipeline
    rs_cfg.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, 30);
    rs_cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_RGBA8, 30);
    // Start streaming with the above configuration
    rs2::pipeline_profile selection = rs_pipe.start(rs_cfg);

    // enable laser max power
    rs2::device selected_device = selection.get_device();
    auto depth_sensor = selected_device.first<rs2::depth_sensor>();

    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
    {
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter
    }
    if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
    {
        // Query min and max values:
        auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
        depth_sensor.set_option(RS2_OPTION_LASER_POWER, range.max); // Set max power
    }

    gst_init(nullptr, nullptr);
    GError* error = nullptr;
    GstElement* gst_pipe = gst_parse_launch(//I420
        "appsrc name=source format=GST_FORMAT_TIME \
        caps=video/x-raw,format=I420,width=1280,height=720,framerate=30/1 \
        ! x264enc tune=zerolatency bitrate=10000 key-int-max=30 ! video/x-h264,profile=high \
        ! rtph264pay pt=96 ! udpsink host=127.0.0.1 port=5000 ",
        &error
    );
    if (gst_pipe == NULL) {
        std::cout << "Couldn't parse launch gstreamer pipeline: " << error->message << std::endl;
        return 1;
    }
    GstElement* gst_source = gst_bin_get_by_name(GST_BIN(gst_pipe), "source");

    gst_element_set_state(gst_pipe, GST_STATE_PLAYING);

    std::cout << "starting main loop" << std::endl;
    long long time = 0;
    int frameCount = 0;
    int imageSize = WIDTH * HEIGHT * 1.5;
    uint8_t* image = new uint8_t[imageSize];

    

    while (true) // Application still alive?
    {
        // Wait for the next set of frames from the camera
        auto frames = rs_pipe.wait_for_frames();

        auto color = frames.get_color_frame();
        auto depth = frames.get_depth_frame();
        

        if (depth.get_height()>HEIGHT || depth.get_width()>WIDTH) {
            continue;
        }


        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        encode_yuv420(depth.get_data(), image, WIDTH, HEIGHT);
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

        GstBuffer* buffer = gst_buffer_new_and_alloc(imageSize);

        GstMapInfo m;

        gst_buffer_map(buffer, &m, GST_MAP_WRITE);

        memcpy(m.data, image, imageSize);

        gst_buffer_unmap(buffer, &m);

        buffer->pts = uint64_t(frameCount / 30 * GST_SECOND);

        GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(gst_source), buffer);

        ++frameCount;



        /*
        cv::Mat img(depth.get_height(), depth.get_width(), CV_8UC3);
        img.data = image;
        cv::imshow("Image", img);
        cv::waitKey(1);
        */


        time = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();

        //std::cout << "Current time = " << time << std::endl;
    }

    gst_element_set_state(gst_pipe, GST_STATE_NULL);

    gst_app_src_end_of_stream(GST_APP_SRC(gst_source));

    //gst_object_unref(gst_source);
    gst_object_unref(gst_pipe);
    delete[] image;
    return 0;
}

int benchmark3() {
    std::cout << "Starting program test RGB I420 transform" << std::endl;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline rs_pipe;
    rs2::config rs_cfg;
    // Use a configuration object to request depth from the pipeline
    rs_cfg.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, 30);
    rs_cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_RGBA8, 30);
    // Start streaming with the above configuration
    rs2::pipeline_profile selection = rs_pipe.start(rs_cfg);

    // enable laser max power
    rs2::device selected_device = selection.get_device();
    auto depth_sensor = selected_device.first<rs2::depth_sensor>();

    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
    {
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter
    }
    if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
    {
        // Query min and max values:
        auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
        depth_sensor.set_option(RS2_OPTION_LASER_POWER, range.max); // Set max power
    }

    // Declare filters
    rs2::decimation_filter dec_filter;  // Decimation - reduces depth frame density
    rs2::threshold_filter thr_filter;   // Threshold  - removes values outside recommended range
    rs2::spatial_filter spat_filter;    // Spatial    - edge-preserving spatial smoothing
    rs2::temporal_filter temp_filter;   // Temporal   - reduces temporal noise

    std::cout << "starting main loop" << std::endl;
    long long time = 0;
    int frameCount = 0;
    int imageSize = WIDTH * HEIGHT;

    uint8_t* image_raw = new uint8_t[imageSize*4];
    uint8_t* image_enc = new uint8_t[imageSize*1.5];
    uint8_t* image_dec = new uint8_t[imageSize*8];
    uint8_t* image_combined = new uint8_t[imageSize * 2 * 1.5];

    while (true)
    {

        auto frames = rs_pipe.wait_for_frames();

        frames = thr_filter.process(frames);
        frames = spat_filter.process(frames);
        frames = temp_filter.process(frames);

        auto color = frames.get_color_frame();
        auto depth = frames.get_depth_frame();

        image_raw = (uint8_t*)color.get_data();



        //std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        encodeRGBAtoI420(image_raw, image_enc, WIDTH, HEIGHT);

        concatI420Vertical(image_enc, image_enc, image_combined, WIDTH, HEIGHT);
        //std::cout << "raw[0]=" << image_raw[100] << ";" << std::endl;
        //std::cout << "enc[0]=" << (int)image_enc[100] << ";" << std::endl;
        decodeI420toRGBA(image_combined, image_dec, WIDTH, HEIGHT*2);
        
        cv::Mat combinedMat(HEIGHT*2, WIDTH, CV_8UC4, image_dec);

        // Afficher
        cv::imshow("Raw vs Decoded (Left: Raw, Right: Decoded)", combinedMat);
        cv::waitKey(1);


    }

    delete[] image_raw;
    delete[] image_enc;
    delete[] image_dec;
    return 0;
}

//int benchmark4() {
//    std::cout << "Starting program test RGB I420 transform" << std::endl;
//
//    // Declare RealSense pipeline, encapsulating the actual device and sensors
//    rs2::pipeline rs_pipe;
//    rs2::config rs_cfg;
//    // Use a configuration object to request depth from the pipeline
//    rs_cfg.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, 30);
//    rs_cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_RGBA8, 30);
//    // Start streaming with the above configuration
//    rs2::pipeline_profile selection = rs_pipe.start(rs_cfg);
//
//    // enable laser max power
//    rs2::device selected_device = selection.get_device();
//    auto depth_sensor = selected_device.first<rs2::depth_sensor>();
//
//    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
//    {
//        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter
//    }
//    if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
//    {
//        // Query min and max values:
//        auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
//        depth_sensor.set_option(RS2_OPTION_LASER_POWER, range.max); // Set max power
//    }
//
//    // Declare filters
//    rs2::decimation_filter dec_filter;  // Decimation - reduces depth frame density
//    rs2::threshold_filter thr_filter;   // Threshold  - removes values outside recommended range
//    rs2::spatial_filter spat_filter;    // Spatial    - edge-preserving spatial smoothing
//    rs2::temporal_filter temp_filter;   // Temporal   - reduces temporal noise
//
//    std::cout << "starting main loop" << std::endl;
//    long long time = 0;
//    int frameCount = 0;
//    int imageSize = WIDTH * HEIGHT;
//
//    uint8_t* image_color_I420 = new uint8_t[imageSize * 1.5];
//    uint8_t* image_depth_I420 = new uint8_t[imageSize * 1.5];
//    uint8_t* image_dec = new uint8_t[imageSize * 8];
//    uint8_t* image_combined = new uint8_t[imageSize * 2 * 1.5];
//
//    while (true)
//    {
//
//        auto frames = rs_pipe.wait_for_frames();
//
//        frames = thr_filter.process(frames);
//        frames = spat_filter.process(frames);
//        frames = temp_filter.process(frames);
//
//        auto color = frames.get_color_frame();
//        auto depth = frames.get_depth_frame();
//
//        //std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
//
//        encodeRGBAtoI420((uint8_t*)color.get_data(), image_color_I420, WIDTH, HEIGHT);
//
//        encode_yuv420((uint16_t*)depth.get_data(), image_depth_I420, WIDTH, HEIGHT);
//
//        concatI420Vertical(image_color_I420, image_depth_I420, image_combined, WIDTH, HEIGHT);
//
//
//        //std::cout << "raw[0]=" << image_raw[100] << ";" << std::endl;
//        //std::cout << "enc[0]=" << (int)image_enc[100] << ";" << std::endl;
//        //decodeI420toRGBA(image_combined, image_dec, WIDTH, HEIGHT * 2);
//
//        //cv::Mat combinedMat(HEIGHT * 2, WIDTH, CV_8UC4, image_dec);
//
//        // Afficher
//        //cv::imshow("Raw vs Decoded (Left: Raw, Right: Decoded)", combinedMat);
//        //cv::waitKey(1);
//
//
//    }
//
//    delete[] image_color_I420;
//    delete[] image_depth_I420;
//    delete[] image_dec;
//    delete[] image_combined;
//    return 0;
//}

int benchmark4() {
    std::cout << "Starting program test RGB I420 transform" << std::endl;

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
    rs2::spatial_filter spat_filter;
    rs2::temporal_filter temp_filter;

    std::cout << "starting main loop" << std::endl;

    int imageSize = WIDTH * HEIGHT;
    uint8_t* image_color_I420 = new uint8_t[imageSize * 1.5];
    uint8_t* image_depth_I420 = new uint8_t[imageSize * 1.5];
    uint8_t* image_dec = new uint8_t[imageSize * 8];
    uint8_t* image_combined = new uint8_t[imageSize * 2 * 1.5];

    int frameCount = 0;
    auto startTime = std::chrono::steady_clock::now();

    while (true)
    {
        auto frames = rs_pipe.wait_for_frames();
        frames = thr_filter.process(frames);
        frames = spat_filter.process(frames);
        frames = temp_filter.process(frames);

        auto color = frames.get_color_frame();
        auto depth = frames.get_depth_frame();

        //encodeRGBAtoI420((uint8_t*)color.get_data(), image_color_I420, WIDTH, HEIGHT);
        encode_yuv420((uint16_t*)depth.get_data(), image_depth_I420, WIDTH, HEIGHT);
        //concatI420Vertical(image_color_I420, image_depth_I420, image_combined, WIDTH, HEIGHT);

        decodeI420toRGBA(image_depth_I420, image_dec, WIDTH, HEIGHT);

        cv::Mat combinedMat(HEIGHT, WIDTH, CV_8UC4, image_dec);

        
        cv::imshow("Color and Depth(Transformed)", combinedMat);
        cv::waitKey(1);

        frameCount++;
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime).count();

        if (elapsed >= 1) {  // toutes les secondes
            std::cout << "FPS: " << frameCount / elapsed << std::endl;
            frameCount = 0;
            startTime = currentTime;
        }
    }

    delete[] image_color_I420;
    delete[] image_depth_I420;
    delete[] image_dec;
    delete[] image_combined;

    return 0;
}


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

    std::cout << "Recording to recording to z16 Press q to stop..." << std::endl;
    int i = 0;
    std::string filename_z16 = "z16_video1.raw";
    std::string filename_ARGB = "ARGB_video1.raw";

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
        depthMat.convertTo(depth8U, CV_8U, 255.0 / 4096);

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

    uint16_t* depth = new uint16_t[frame_size_z16];
    uint8_t* color = new uint8_t[frame_size_argb];

    while (true)
    {

        ifs_Z16.read((char*)depth, frame_size_z16);
        if (!ifs_Z16) break; // check if end of file.


        ifs_ARGB.read((char*)color, frame_size_argb);
        if (!ifs_ARGB) break; // check if end of file.


        cv::Mat depthMat(HEIGHT, WIDTH, CV_16U, depth);
        cv::Mat colorMat(HEIGHT, WIDTH, CV_8UC4, color);


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

double mse(uint16_t* reference, uint16_t* received, int height, int width){
    double sum = 0;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {

            sum += pow(reference[x + y * width] - received[x + y * width], 2);
        }
    }

    return sum / (height * width);
}

double psnr(double mse) {
    return 10 * log10(pow(2, 13) / mse);
}


int quality_test() {
    

    std::string filename_z16_received = "z16_video1_received.raw";
    std::string filename_z16_reference = "z16_video1.raw";

    // === Reading RAW files ===
    std::ifstream ifs_Z16_received(filename_z16_received, std::ios::binary);
    std::ifstream ifs_Z16_reference(filename_z16_reference, std::ios::binary);

    if (!ifs_Z16_reference.is_open() || !ifs_Z16_received.is_open()) {
        std::cerr << "Impossible d'ouvrir les fichiers RAW" << std::endl;
        return -1;
    }

    size_t frame_size_z16 = WIDTH * HEIGHT * 2;  // uint16_t

    uint16_t* depth_received = new uint16_t[frame_size_z16];
    uint16_t* depth_reference = new uint16_t[frame_size_z16];

    double PSNR = 0;
    double MSE = 0;

    while (true)
    {

        ifs_Z16_received.read((char*)depth_received, frame_size_z16);
        if (!ifs_Z16_received) break; // check if end of file.


        ifs_Z16_reference.read((char*)depth_reference, frame_size_z16);
        if (!ifs_Z16_reference) break; // check if end of file.

        
        MSE = mse(depth_reference, depth_received, HEIGHT, WIDTH);
        PSNR = psnr(MSE);

        std::cout << "PSNR =" << PSNR << " MSE = " << MSE<< std::endl;

    }

    ifs_Z16_received.close();
    ifs_Z16_reference.close();
    return 0;

}

int main() {
    quality_test();
}
