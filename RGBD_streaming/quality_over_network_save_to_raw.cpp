// Created by IT-JIM
// VIDEO1 : Send video to appsink, display with cv::imshow()

#include <iostream>
#include <string>
#include <thread>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <fstream>

#include "encode_z16.h"
#include "encodeRGBtoI420.h"

#include <opencv2/opencv.hpp>

//======================================================================================================================
/// A simple assertion function + macro
inline void myAssert(bool b, const std::string& s = "MYASSERT ERROR !") {
    if (!b)
        throw std::runtime_error(s);
}

#define MY_ASSERT(x) myAssert(x, "MYASSERT ERROR :" #x)

//======================================================================================================================
/// Check GStreamer error, exit on error
inline void checkErr(GError* err) {
    if (err) {
        std::cerr << "checkErr : " << err->message << std::endl;
        exit(0);
    }
}

//======================================================================================================================
/// Our global data, serious gstreamer apps should always have this !
struct GoblinData {
    GstElement* pipeline = nullptr;
    GstElement* sinkVideo = nullptr;
};

//======================================================================================================================
/// Process a single bus message, log messages, exit on error, return false on eof
static bool busProcessMsg(GstElement* pipeline, GstMessage* msg, const std::string& prefix) {
    using namespace std;

    GstMessageType mType = GST_MESSAGE_TYPE(msg);
    cout << "[" << prefix << "] : mType = " << mType << " ";
    switch (mType) {
    case (GST_MESSAGE_ERROR):
        // Parse error and exit program, hard exit
        GError* err;
        gchar* dbg;
        gst_message_parse_error(msg, &err, &dbg);
        cout << "ERR = " << err->message << " FROM " << GST_OBJECT_NAME(msg->src) << endl;
        cout << "DBG = " << dbg << endl;
        g_clear_error(&err);
        g_free(dbg);
        exit(1);
    case (GST_MESSAGE_EOS):
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

    default:
        cout << endl;
    }
    return true;
}

//======================================================================================================================
/// Run the message loop for one bus
void codeThreadBus(GstElement* pipeline, GoblinData& data, const std::string& prefix) {
    using namespace std;
    GstBus* bus = gst_element_get_bus(pipeline);

    int res;
    while (true) {
        GstMessage* msg = gst_bus_timed_pop(bus, GST_CLOCK_TIME_NONE);
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
/// Appsink process thread
void codeThreadProcessV(GoblinData& data) {
    using namespace std;
    initialize_delta_table();
    initialize_z16_tables();

    int imW = 1280;
    int imH = 720;
    uint8_t* color_rgba = new uint8_t[imH * imW * 4 * 2]; // attention si concatene pas oublier le fois 2
    uint8_t* color_yuv = new uint8_t[imH * imW * 3/2];
    uint8_t* depth_yuv = new uint8_t[imH * imW * 3/2];
    uint16_t* depth_z16 = new uint16_t[imH * imW];

    std::string filename_z16 = "z16_video1_received.raw";
    std::string filename_ARGB = "ARGB_video1_received.raw";

    // === Writting RAW files ===
    std::ofstream ofs_Z16(filename_z16, std::ios::binary);
    std::ofstream ofs_ARGB(filename_ARGB, std::ios::binary);

    for (;;) {
        // Exit on EOS
        if (gst_app_sink_is_eos(GST_APP_SINK(data.sinkVideo))) {
            cout << "EOS !" << endl;
            break;
        }

        // Pull the sample (synchronous, wait)
        GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(data.sinkVideo));
        if (sample == nullptr) {
            cout << "NO sample !" << endl;
            break;
        }

        GstBuffer* buffer = gst_sample_get_buffer(sample);
        GstMapInfo m;
        MY_ASSERT(gst_buffer_map(buffer, &m, GST_MAP_READ));
        MY_ASSERT(m.size == imW * imH * 1.5 * 2); 

        deconcatI420Vertical((uint8_t*)m.data, color_yuv, depth_yuv, imW, imH);

        decodeI420toRGBA_libyuv(color_yuv, color_rgba, imW, imH);
        decode_depth_z16_fast(depth_yuv, depth_z16, imW, imH);

        ofs_Z16.write((char*)depth_z16, imW * imH * 2);  // Z16 = 16 bits per pixel (2*8)
        ofs_ARGB.write((char*)color_rgba, imW * imH * 4); // RGBA8 = 4*8 bits per pixel

        cv::Mat colorMat(imH, imW, CV_8UC4, color_rgba);
        cv::Mat depthMat(imH, imW, CV_16U, depth_z16);

        cv::Mat depth8U;
        depthMat.convertTo(depth8U, CV_8U, 255.0 / 4096);

        cv::imshow("Depth frame received", depth8U);

        cv::imshow("Color frame received", colorMat);
        cv::waitKey(1);

        // Don't forget to unmap the buffer and unref the sample
        gst_buffer_unmap(buffer, &m);
        gst_sample_unref(sample);

    }
    // Why not good closing ?
    delete[] color_yuv;
    delete[] color_rgba;
    delete[] depth_yuv;
    delete[] depth_z16;

    ofs_Z16.close();
    ofs_ARGB.close();
}

//======================================================================================================================
int main(int argc, char** argv) {
    using namespace std;
    cout << "VIDEO1 : Send video to appsink, display with cv::imshow()" << endl;

    // Init gstreamer
    gst_init(&argc, &argv);

    GoblinData data;


    //gchar* pipeStr = g_strdup_printf(
    //    "udpsrc uri=udp://127.0.0.1:5000 caps=\"application/x-rtp, media=video, encoding-name=H265, payload=96\" ! "
    //    "rtpjitterbuffer latency=100 ! "
    //    "rtph265depay ! "
    //    "h265parse ! "
    //    "d3d11h265dec ! "
    //    "videoconvert ! "
    //    "video/x-raw,format=I420 ! "
    //    "appsink name=mysink max-buffers=0 drop=true sync=false"
    //);

    //gchar* pipeStr = g_strdup_printf(
    //    "udpsrc uri=udp://127.0.0.1:5000 caps=\"application/x-rtp, media=video, encoding-name=H265, payload=96\" ! "
    //    "rtpjitterbuffer latency=100 ! "
    //    "rtph265depay ! "
    //    "h265parse ! "
    //    "nvh265dec ! "
    //    "videoconvert ! "
    //    "video/x-raw,format=I420 ! "
    //    "appsink name=mysink max-buffers=0 drop=true sync=false"
    //);

    //gchar* pipeStr = g_strdup_printf(
    //    "udpsrc port=5000 caps=\"application/x-rtp, media=video, encoding-name=H265, clock-rate=90000, payload=96\" ! "
    //    "rtpjitterbuffer latency=50 drop-on-latency=true ! "
    //    "rtph265depay ! "
    //    "h265parse ! "
    //    "nvh265dec ! "
    //    "videoconvert ! "
    //    "video/x-raw,format=I420 ! "
    //    "appsink name=mysink max-buffers=1 drop=true sync=false"
    //);

    gchar* pipeStr = g_strdup_printf(
        "udpsrc uri=udp://127.0.0.1:5000 caps=\"application/x-rtp, media=video, encoding-name=H264, payload=96\" ! "
        "rtpjitterbuffer latency=10 ! "
        "rtph264depay ! "
        "h264parse ! "
        "d3d11h264dec ! "
        "videoconvert ! "
        "video/x-raw,format=I420 ! "
        "appsink name=mysink emit-signals=true max-buffers=0 drop=true sync=false"
    );

    GError* err = nullptr;
    data.pipeline = gst_parse_launch(pipeStr, &err);
    checkErr(err);
    MY_ASSERT(data.pipeline);

    // Find our appsink by name
    data.sinkVideo = gst_bin_get_by_name(GST_BIN(data.pipeline), "mysink");
    MY_ASSERT(data.sinkVideo);

    // Play the pipeline
    MY_ASSERT(gst_element_set_state(data.pipeline, GST_STATE_PLAYING));

    // Start the bus thread
    thread threadBus([&data]() -> void {
        codeThreadBus(data.pipeline, data, "GOBLIN");
        });

    // Start the appsink process thread
    thread threadProcess([&data]() -> void {
        codeThreadProcessV(data);
        });

    // Wait for threads
    threadBus.join();
    threadProcess.join();

    // Destroy the pipeline
    gst_element_set_state(data.pipeline, GST_STATE_NULL);
    gst_object_unref(data.pipeline);

    return 0;
}