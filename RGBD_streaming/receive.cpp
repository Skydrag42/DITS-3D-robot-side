// Created by IT-JIM
// VIDEO1 : Send video to appsink, display with cv::imshow()

#include <iostream>
#include <string>
#include <thread>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

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

        // You can add more stuff here if you want

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

    int imW = 1280;
    int imH = 720;
    uint8_t* image_dec = new uint8_t[imH * imW * 4 * 2]; // attention si concatene pas oublier le fois 2

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
        MY_ASSERT(m.size == imW * imH * 1.5 * 2); // attention si concatene pas oublier le fois 2


        //decode_depth_z16(m.data, image_dec, imW, imH);

        decodeI420toRGBA((uint8_t*)m.data, image_dec, imW, imH * 2); // attention si concatene pas oublier le fois 2

        cv::Mat decMat(imH * 2, imW, CV_8UC4, image_dec);

        //// Convertir en 8 bits
        //cv::Mat decMat8U;
        //double minVal, maxVal;
        //cv::minMaxLoc(decMat, &minVal, &maxVal);

        //// Normalisation identique
        //decMat.convertTo(decMat8U, CV_8UC1, 255.0 / 4096);

        cv::imshow("frame", decMat);
        cv::waitKey(1);

        // Don't forget to unmap the buffer and unref the sample
        gst_buffer_unmap(buffer, &m);
        gst_sample_unref(sample);

    }

    delete[] image_dec;
}

//======================================================================================================================
int main(int argc, char** argv) {
    using namespace std;
    cout << "VIDEO1 : Send video to appsink, display with cv::imshow()" << endl;

    // Init gstreamer
    gst_init(&argc, &argv);

    GoblinData data;
    
    //string pipeStr = "mfvideosrc device-index=0 ! videoconvert ! video/x-raw,format=BGR ! appsink name=mysink max-buffers=2 sync=1";
    // 
    // OPTIMAL H264 (for the moment)
    string pipeStr = "udpsrc port=5000 caps=\"application/x-rtp, media=video, encoding-name=H264, payload=96\" ! rtpjitterbuffer latency = 0 ! rtph264depay ! h264parse ! nvh264dec ! videoconvert ! video/x-raw,format=I420 ! appsink name=mysink max-buffers=1 sync=false drop=true";
    // 
    // TEST H265
    //string pipeStr = "udpsrc uri=udp://127.0.0.1:5000 caps=\"application/x-rtp, media=video, encoding-name=H265, payload=96\" ! rtpjitterbuffer latency = 0 ! rtph265depay ! h265parse ! nvh265dec ! videoconvert ! video/x-raw,format=I420 ! appsink name=mysink max-buffers=1 sync=false drop=true";
    GError* err = nullptr;
    data.pipeline = gst_parse_launch(pipeStr.c_str(), &err);
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