//
// Created by IT-JIM 
// VIDEO2: Decode a video file with opencv and send to a gstreamer pipeline via appsrc

#include <iostream>
#include <string>
#include <thread>
#include <atomic>
#include <cmath>
#include <sstream>
#include <algorithm>
#include <chrono>

#include "encode_z16.h"
#include "encodeRGBtoI420.h"

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <librealsense2/rs.hpp> 
#include <opencv2/opencv.hpp>

//======================================================================================================================
/// A simple assertion function + macro
inline void myAssert(bool b, const std::string &s = "MYASSERT ERROR !") {
    if (!b)
        throw std::runtime_error(s);
}

#define MY_ASSERT(x) myAssert(x, "MYASSERT ERROR :" #x)

//======================================================================================================================
/// Check GStreamer error, exit on error
inline void checkErr(GError *err) {
    if (err) {
        std::cerr << "checkErr : " << err->message << std::endl;
        exit(0);
    }
}

//======================================================================================================================
/// Our global data, serious gstreamer apps should always have this !
struct GoblinData {
    /// pipeline
    GstElement *pipeline = nullptr;
    /// appsrc
    GstElement *srcVideo = nullptr;
    /// Video file name
    std::string fileName;
    /// Appsrc flag: when it's true, send the frames
    std::atomic_bool flagRunV{false};

};

//======================================================================================================================
/// Process a single bus message, log messages, exit on error, return false on eof
static bool busProcessMsg(GstElement *pipeline, GstMessage *msg, const std::string &prefix) {
    using namespace std;

    GstMessageType mType = GST_MESSAGE_TYPE(msg);
    cout << "[" << prefix << "] : mType = " << mType << " ";
    switch (mType) {
        case (GST_MESSAGE_ERROR):
            // Parse error and exit program, hard exit
            GError *err;
            gchar *dbg;
            gst_message_parse_error(msg, &err, &dbg);
            cout << "ERR = " << err->message << " FROM " << GST_OBJECT_NAME(msg->src) << endl;
            cout << "DBG = " << dbg << endl;
            g_clear_error(&err);
            g_free(dbg);
            exit(1);
        case (GST_MESSAGE_EOS) :
            // Soft exit on EOS
            cout << " EOS !" << endl;
            return false;
        case (GST_MESSAGE_STATE_CHANGED):
            // Parse state change, print extra info for pipeline only
            cout << "State changed !" << endl;
            if (GST_MESSAGE_SRC(msg) == GST_OBJECT(pipeline)) {
                GstState sOld, sNew, sPenging;
                gst_message_parse_state_changed(msg, &sOld, &sNew, &sPenging);
                cout << "Pipeline changed from " << gst_element_state_get_name(sOld) << " to " <<
                     gst_element_state_get_name(sNew) << endl;
            }
            break;
        case (GST_MESSAGE_STEP_START):
            cout << "STEP START !" << endl;
            break;
        case (GST_MESSAGE_STREAM_STATUS):
            cout << "STREAM STATUS !" << endl;
            break;
        case (GST_MESSAGE_ELEMENT):
            cout << "MESSAGE ELEMENT !" << endl;
            break;

            // You can add more stuff here if you want

        default:
            cout << endl;
    }
    return true;
}

//======================================================================================================================
/// Run the message loop for one bus
void codeThreadBus(GstElement *pipeline, GoblinData &data, const std::string &prefix) {
    using namespace std;
    GstBus *bus = gst_element_get_bus(pipeline);

    int res;
    while (true) {
        GstMessage *msg = gst_bus_timed_pop(bus, GST_CLOCK_TIME_NONE);
        MY_ASSERT(msg);
        res = busProcessMsg(pipeline, msg, prefix);
        gst_message_unref(msg);
        if (!res)
            break;
    }
    gst_object_unref(bus);
    cout << "BUS THREAD FINISHED : " << prefix << endl;
}

//======================================================================================================================
/// Take video from IntelRealsense SDK and send data to appsrc
void codeThreadSrcV(GoblinData &data) {
    using namespace std;
    using namespace cv;

    // Find width, height, FPS
    int imW = 1280;
    int imH = 720;
    double fps = 30;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline rs_pipe;
    rs2::config rs_cfg;

    // Use a configuration object to request depth from the pipeline
    rs_cfg.enable_stream(RS2_STREAM_DEPTH, imW, imH, RS2_FORMAT_Z16, 30);
    rs_cfg.enable_stream(RS2_STREAM_COLOR, imW, imH, RS2_FORMAT_RGBA8, 30);

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
    //rs2::temporal_filter temp_filter;   // Temporal   - reduces temporal noise
    rs2::align align_to_color(RS2_STREAM_COLOR);

    ostringstream oss; // attention si concatene pas oublier le fois 2
    oss << "video/x-raw,format=I420,width=" << imW << ",height=" << imH * 2 << ",framerate=" << int(lround(fps)) << "/1";
    cout << "CAPS=" << oss.str() << endl;
    GstCaps *capsVideo = gst_caps_from_string(oss.str().c_str()); 
    g_object_set(data.srcVideo, "caps", capsVideo, nullptr);
    gst_caps_unref(capsVideo);

    // Play the pipeline AFTER we have set up the final caps
    MY_ASSERT(gst_element_set_state(data.pipeline, GST_STATE_PLAYING));


    int bufferSize = imH * imW * 3/2;
    uint8_t* image = new uint8_t[bufferSize];
    uint16_t* depth_raw = new uint16_t[bufferSize * 2/3];
    uint16_t* depth_decoded = new uint16_t[bufferSize * 2 / 3];
    uint8_t* depth_encoded = new uint8_t[bufferSize];
    uint8_t* frame_combined = new uint8_t[bufferSize * 2];
    uint8_t* frame_combined_decoded = new uint8_t[bufferSize * 2/3 * 4 * 2];
    uint8_t* image_color_encoded_yuv = new uint8_t[bufferSize];

    // Frame loop
    int frameCount = 0;
    auto intr = selection.get_stream(RS2_STREAM_COLOR)
        .as<rs2::video_stream_profile>()
        .get_intrinsics();

    std::cout << "fx=" << intr.fx
        << " fy=" << intr.fy
        << " ppx=" << intr.ppx
        << " ppy=" << intr.ppy
        << std::endl;

    auto startTime = std::chrono::steady_clock::now();

    for (;;) {
        // If the flag is false, go idle and wait, the pipeline does not want data for now
        if (!data.flagRunV) {
            cout << "(wait)" << endl;
            this_thread::sleep_for(chrono::milliseconds(10));
            continue;
        }

        // Wait for the next set of frames from the camera
        auto frames = rs_pipe.wait_for_frames();

        frames = thr_filter.process(frames);
        //frames = spat_filter.process(frames);
        //frames = temp_filter.process(frames);


        frames = align_to_color.process(frames);
        auto color = frames.get_color_frame(); 
        auto depth = frames.get_depth_frame();

        //Copier la profondeur dans un buffer modifiable
        memcpy(depth_raw, depth.get_data(), bufferSize * 2/3 * sizeof(uint16_t));

        encode_yuv420(depth_raw, depth_encoded, imW, imH);

        //decode_depth_z16(depth_encoded, depth_decoded, imW, imH);

        ////Debut Affichage des données envoyées
        //cv::Mat rawMat(imH, imW, CV_16UC1, depth_decoded);

        //// Convertir en 8 bits
        //cv::Mat rawMat8U;
        //double minVal, maxVal;
        //cv::minMaxLoc(rawMat, &minVal, &maxVal);

        //// Normalisation identique
        //rawMat.convertTo(rawMat8U, CV_8UC1, 255.0 / 4096);
        //
        //cv::imshow("framedebut", rawMat8U);
        //cv::waitKey(1);

        ////fin Affichage

        // New affichage pour verification de la compression 



        encodeRGBAtoI420((uint8_t*)color.get_data(), image_color_encoded_yuv, imW, imH);

        // Affichage couleur direct

        cv::Mat Matcouleur(imH, imW, CV_8UC4, (uint8_t*)color.get_data());
        cv::imshow("video_source", Matcouleur);
        cv::waitKey(1);
        concatI420Vertical(image_color_encoded_yuv, depth_encoded, (uint8_t*)frame_combined, imW, imH);


        /*decodeI420toRGBA(frame_combined, frame_combined_decoded, imW, imH*2);

        cv::Mat MatafficheRGB(imH * 2, imW, CV_8UC4, frame_combined_decoded);

        cv::imshow("framedebut", MatafficheRGB);
        cv::waitKey(1);*/

        // Create a GStreamer buffer and copy data to it via a map
 
        GstBuffer *buffer = gst_buffer_new_and_alloc(bufferSize * 2); // attention si concatene pas oublier le fois 2
        GstMapInfo m;
        gst_buffer_map(buffer, &m, GST_MAP_WRITE);
        memcpy(m.data, frame_combined, bufferSize * 2); // attention si concatene pas oublier le fois 2
        gst_buffer_unmap(buffer, &m);

        // Set up timestamp
        // This is not strictly required, but you need it for the correct 1x playback with sync=1 !
        buffer->pts = uint64_t(frameCount  / fps * GST_SECOND);

        // Send buffer to gstreamer
        GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(data.srcVideo), buffer);

        ++frameCount;
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime).count();

        if (elapsed >= 1) {  // toutes les secondes
            std::cout << "FPS: " << frameCount / elapsed << std::endl;
            frameCount = 0;
            startTime = currentTime;
        }
    }
    delete[] depth_raw;
    delete[] depth_encoded;
    delete[] depth_decoded;
    delete[] frame_combined_decoded;

    // Signal EOF to the pipeline
    gst_app_src_end_of_stream(GST_APP_SRC(data.srcVideo));
}

//======================================================================================================================
/// Callback called when the pipeline wants more data
static void startFeed(GstElement *source, guint size, GoblinData *data) {
    using namespace std;
    if (!data->flagRunV) {
        cout << "startFeed !" << endl;
        data->flagRunV = true;
    }
}

//======================================================================================================================
/// Callback called when the pipeline wants no more data for now
static void stopFeed(GstElement *source, GoblinData *data) {
    using namespace std;
    if (data->flagRunV) {
        cout << "stopFeed !" << endl;
        data->flagRunV = false;
    }
}

//======================================================================================================================
int main(int argc, char **argv) {
    using namespace std;

    // Init gstreamer
    gst_init(&argc, &argv);

    // Our global data
    GoblinData data;
    data.fileName = "video_z16";
    cout << "Live video from camera : " << data.fileName << endl;

    // Create GSTreamer pipeline
    // Note: we don't know image size or framerate yet !
    // We'll give preliminary caps only which we will replace later
    // format=time is not really needed for video, but audio appsrc will not work without it !
    //string pipeStr = "appsrc name=mysrc format=time caps=video/x-raw,format=I420 ! videoconvert !udpsink host=127.0.0.1 port=5000";
    string pipeStr = "appsrc name=mysrc format=time caps=video/x-raw,format=I420 ! queue ! videoconvert ! nvh264enc bitrate=20000 gop-size=30 ! video/x-h264, profile=high ! h264parse ! rtph264pay config-interval=1 pt=96 ! udpsink host=192.168.1.3 port=5000";
    //string pipeStr = "appsrc name=mysrc format=time caps=video/x-raw,format=I420 !queue !videoconvert !nvh265enc bitrate = 20000 gop-size=30 preset=hq !h265parse !rtph265pay config-interval=1 pt=96 !udpsink host=127.0.0.1 port = 5000";

    // ! \\ OPTIMALE : celle du dessous est testée et marche bien ne pas modifier
    //string pipeStr = "appsrc name=mysrc format=time is-live=true do-timestamp=true caps=video/x-raw,format=I420 ! queue ! videoconvert ! nvh264enc bitrate=20000 gop-size=30 preset=p1 tune=ultra-low-latency zerolatency=true ! video/x-h264,profile=main ! h264parse config-interval=1 ! rtph264pay pt=96 ! udpsink host=127.0.0.1 port=5000"; //192.168.1.2

    // TESTS

    // avec H265 
    // TO DO modify uri
    //std::string pipeStr = "appsrc name=mysrc format=time is-live=true do-timestamp=true caps=video/x-raw,format=I420 ! queue leaky=downstream ! videoconvert ! nvh265enc bitrate=5000 gop-size=30 preset=p7 tune=ultra-low-latency zerolatency=true ! video/x-h265,profile=main ! h265parse config-interval=1 ! rtph265pay pt=96 ! udpsink host=127.0.0.1 port=5000"; //192.168.1.2

    GError *err = nullptr;
    data.pipeline = gst_parse_launch(pipeStr.c_str(), &err);
    checkErr(err);
    MY_ASSERT(data.pipeline);
    // Find our appsrc by name
    data.srcVideo = gst_bin_get_by_name(GST_BIN (data.pipeline), "mysrc");
    MY_ASSERT(data.srcVideo);

    // Important ! We don't want to abuse the appsrc queue
    // Thus let the pipeline itself signal us when it wants data
    // This is based on GLib signals
    g_signal_connect(data.srcVideo, "need-data", G_CALLBACK(startFeed), &data);
    g_signal_connect(data.srcVideo, "enough-data", G_CALLBACK(stopFeed), &data);

    // Start the bus thread
    thread threadBus([&data]() -> void {
        codeThreadBus(data.pipeline, data, "ELF");
    });

    // Start the appsrc process thread
    thread threadSrcV([&data]() -> void {
        codeThreadSrcV(data);
    });

    // Wait for threads
    threadBus.join();
    threadSrcV.join();

    // Destroy the pipeline
    gst_element_set_state(data.pipeline, GST_STATE_NULL);
    gst_object_unref(data.pipeline);

    return 0;
}
