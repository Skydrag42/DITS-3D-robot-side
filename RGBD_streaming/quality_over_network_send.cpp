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
#include <cstring>
#include <fstream>
#include <string>

#include "encode_z16.h"
#include "encodeRGBtoI420.h"

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <librealsense2/rs.hpp> 
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
    /// pipeline
    GstElement* pipeline = nullptr;
    /// appsrc
    GstElement* srcVideo = nullptr;
    /// Video file name
    std::string fileName;
    /// Appsrc flag: when it's true, send the frames
    std::atomic_bool flagRunV{ false };

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
/// Take video from IntelRealsense SDK and send data to appsrc
void codeThreadSrcV(GoblinData& data) {
    using namespace std;

    // Find width, height, FPS
    int imW = 1280;
    int imH = 720;
    double fps = 30;

    // initialise tabulated values for encode_z16
    initialize_z16_tables();
    initialize_delta_table();

    ostringstream oss; // attention si concatene pas oublier le fois 2
    oss << "video/x-raw,format=I420,width=" << imW << ",height=" << imH * 2 << ",framerate=" << int(lround(fps)) << "/1";
    cout << "CAPS=" << oss.str() << endl;
    GstCaps* capsVideo = gst_caps_from_string(oss.str().c_str());
    g_object_set(data.srcVideo, "caps", capsVideo, nullptr);
    gst_caps_unref(capsVideo);

    // Play the pipeline AFTER we have set up the final caps
    MY_ASSERT(gst_element_set_state(data.pipeline, GST_STATE_PLAYING));


    int bufferSize = imH * imW * 3 / 2;
    uint8_t* image = new uint8_t[bufferSize];
    uint16_t* depth_raw = new uint16_t[bufferSize * 2 / 3];
    uint16_t* depth_decoded = new uint16_t[bufferSize * 2 / 3];
    uint8_t* depth_encoded = new uint8_t[bufferSize];
    uint8_t* frame_combined = new uint8_t[bufferSize * 2];
    uint8_t* frame_combined_decoded = new uint8_t[bufferSize * 2 / 3 * 4 * 2];
    uint8_t* image_color_encoded_yuv = new uint8_t[bufferSize];

    // Frame loop
    int frameCount = 0;

    auto startTime = std::chrono::steady_clock::now();

    std::string filename_z16 = "z16_video1.raw";
    std::string filename_ARGB = "ARGB_video1.raw";

    std::ifstream ifs_Z16(filename_z16, std::ios::binary);
    std::ifstream ifs_ARGB(filename_ARGB, std::ios::binary);

    MY_ASSERT(ifs_Z16.is_open() && ifs_ARGB.is_open());

    int frame_size_z16 = imW * imH * 2;  // uint16_t
    int frame_size_argb = imW * imH * 4; // uint8_t x4

    uint16_t* depth = new uint16_t[frame_size_z16];
    uint8_t* color = new uint8_t[frame_size_argb];

    for (;;) {

        // If the flag is false, go idle and wait, the pipeline does not want data for now
        if (!data.flagRunV) {
            cout << "(wait)" << endl;
            this_thread::sleep_for(chrono::milliseconds(10));
            continue;
        }

        this_thread::sleep_for(chrono::milliseconds(16)); // Expliquer pk

        ifs_Z16.read((char*)depth, frame_size_z16);
        if (!ifs_Z16) break; // check if end of file.

        ifs_ARGB.read((char*)color, frame_size_argb);
        if (!ifs_ARGB) break; // check if end of file.


        cv::Mat Matcouleur(imH, imW, CV_8UC4, (uint8_t*)color);
        cv::imshow("video_source", Matcouleur);
        cv::waitKey(1);

        //Copier la profondeur dans un buffer modifiable
        memcpy((uint16_t*)depth_raw, (uint16_t*)depth, bufferSize * 2 / 3 * sizeof(uint16_t));

        encode_yuv420_fast(depth_raw, depth_encoded, imW, imH);

        encodeRGBAtoI420_libyuv((uint8_t*)color, image_color_encoded_yuv, imW, imH);

        concatI420Vertical(image_color_encoded_yuv, depth_encoded, (uint8_t*)frame_combined, imW, imH);


        // Create a GStreamer buffer and copy data to it via a map

        GstBuffer* buffer = gst_buffer_new_and_alloc(bufferSize * 2); // attention si concatene pas oublier le fois 2
        GstMapInfo m;
        gst_buffer_map(buffer, &m, GST_MAP_WRITE);
        memcpy(m.data, frame_combined, bufferSize * 2); // attention si concatene pas oublier le fois 2
        gst_buffer_unmap(buffer, &m);

        // Set up timestamp
        // This is not strictly required, but you need it for the correct 1x playback with sync=1 !
        buffer->pts = uint64_t(frameCount / fps * GST_SECOND);

        // Send buffer to gstreamer
        GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(data.srcVideo), buffer);

        ++frameCount;
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime).count();

        if (elapsed >= 1) {  // Every second
            std::cout << "FPS: " << frameCount / elapsed << std::endl;
            frameCount = 0;
            startTime = currentTime;
        }
    }

    delete[] depth_raw;
    delete[] depth_encoded;
    delete[] depth_decoded;
    delete[] frame_combined_decoded;

    // Do not forget to close reading
    ifs_Z16.close();
    ifs_ARGB.close();
    // Signal EOF to the pipeline
    gst_app_src_end_of_stream(GST_APP_SRC(data.srcVideo));

}

//======================================================================================================================
/// Callback called when the pipeline wants more data
static void startFeed(GstElement* source, guint size, GoblinData* data) {
    using namespace std;
    if (!data->flagRunV) {
        cout << "startFeed !" << endl;
        data->flagRunV = true;
    }
}

//======================================================================================================================
/// Callback called when the pipeline wants no more data for now
static void stopFeed(GstElement* source, GoblinData* data) {
    using namespace std;
    if (data->flagRunV) {
        cout << "stopFeed !" << endl;
        data->flagRunV = false;
    }
}

//======================================================================================================================
int main(int argc, char** argv) {
    using namespace std;

    char host[20] = "127.0.0.1";

    if (argc < 2) {
        cout << "Missing one required argument: ip address of receiver --> will default to 127.0.0.1" << endl;
    }
    else {
        strcpy(host, argv[1]);
    }

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


    //   Quality enhancements :
    //    -`preset=hq` - High quality preset instead of default
    //    - `rc-mode = vbr` with `max-bitrate = 30000` - Variable bitrate for better quality
    //    - `bitrate=25000` - Increased base bitrate
    //    - `bframes=2` - B - frames for better compression efficiency
    //    - `cabac=true` - Better entropy coding(already default but explicit)
    //    - `spatial-aq = true aq - strength = 8` - Adaptive quantization(lowered from default 15 as requested)

    //    Quality control :
    //    -`qp-min - i = 18 qp - max - i = 28` - Tighter QP range for I - frames
    //    - `qp-min - p = 20 qp - max - p = 30` - Controlled P - frame quality
    //    - `qp-min - b = 22 qp - max - b = 32` - B - frame quality limits

    //    Structure improvements : 
    //    -`gop-size = 30` - Increased from 15 for better efficiency(still reasonable for streaming)

    // avec h265

    //gchar* pipeStr = g_strdup_printf(
    //    "appsrc is-live=true name=mysrc format=time "
    //    "caps=video/x-raw,format=I420 ! "
    //    "queue max-size-buffers=1 leaky=downstream ! "
    //    "videoconvert ! "
    //    "nvh265enc repeat-sequence-header=true preset=low-latency tune=ultra-low-latency zerolatency=true "
    //    "rc-mode=cbr bitrate=6000 gop-size=1 bframes=0 "
    //    "qp-min-i=20 qp-max-i=30 qp-min-p=20 qp-max-p=30 "
    //    "! video/x-h265 ! "
    //    "rtph265pay config-interval=1 pt=96 mtu=1200 ! "
    //    "queue max-size-buffers=1 leaky=downstream ! "
    //    "udpsink host=%s port=5000 sync=false async=false",
    //    host
    //);

    // test en coursssss

    //gchar* pipeStr = g_strdup_printf(
    //    "appsrc is-live=true name=mysrc format=time "
    //    "caps=video/x-raw,format=I420 ! "
    //    "queue max-size-buffers=1 leaky=downstream ! "
    //    "videoconvert ! "
    //    "nvh265enc repeat-sequence-header=true preset=low-latency tune=ultra-low-latency zerolatency=true "
    //    "rc-mode=cbr bitrate=6000 gop-size=1 bframes=0 "
    //    "! video/x-h265,profile=main ! "
    //    "rtph265pay config-interval=1 pt=96 mtu=1200 ! "
    //    "udpsink host=%s port=5000 sync=false async=false",
    //    host
    //);



    // ULTRA LOW LATENCY ----- TO KEEEEEEEEEPPPPPP H264
    gchar* pipeStr = g_strdup_printf(
        "appsrc is-live=true name=mysrc format=time "
        "caps=video/x-raw,format=I420 ! "
        "queue max-size-buffers=1 leaky=downstream ! "
        "videoconvert ! "
        "nvh264enc repeat-sequence-header=true preset=low-latency tune=ultra-low-latency zerolatency=true "
        "rc-mode=cbr bitrate=20000 gop-size=1 bframes=0 cabac=false "
        "qp-min-i=20 qp-max-i=30 qp-min-p=20 qp-max-p=30 "
        "! video/x-h264,profile=baseline ! "
        "rtph264pay config-interval=1 pt=96 mtu=1200 ! "
        "queue max-size-buffers=1 leaky=downstream ! "
        "udpsink host=%s port=5000 sync=false async=false",
        host
    );

    // BETTER QUALITY THEN ABOVE BUT NOT TESTED
    //    gchar* pipeStr = g_strdup_printf(
    //    "appsrc is-live=true name=mysrc format=time "
    //    "caps=video/x-raw,format=I420 ! "
    //    "queue max-size-buffers=1 leaky=downstream !"
    //    "videoconvert ! "
    //    "nvh264enc repeat-sequence-header=true preset=low-latency tune=ultra-low-latency zerolatency=true "
    //    "rc-mode=cbr bitrate=0 gop-size=5 bframes=0 cabac=false "
    //    "qp-min-i=20 qp-max-i=30 qp-min-p=20 qp-max-p=30 "
    //    "! video/x-h264,profile=high ! "
    //    "rtph264pay config-interval=1 pt=96 mtu=1200 ! "
    //    "queue max-size-buffers=1 leaky=downstream ! "
    //    "udpsink host=%s port=5000 sync=false async=false",
    //    host
    //);

    // ULTRA QUALITY but latency de fou

    //gchar* pipeStr = g_strdup_printf(
    //    "appsrc is-live=true name=mysrc format=time "
    //    "caps=video/x-raw,format=I420 ! "
    //    "queue max-size-buffers=1 leaky=downstream ! "
    //    "videoconvert ! "
    //    "nvh264enc repeat-sequence-header=true preset=p7 tune=high-quality "
    //    "rc-mode=vbr bitrate=35000 max-bitrate=40000 "
    //    "gop-size=60 bframes=2 cabac=true "
    //    "qp-min-i=0 qp-max-i=40 qp-min-p=0 qp-max-p=40 "
    //    "! video/x-h264,profile=high ! "
    //    "rtph264pay config-interval=1 pt=96 mtu=1200 ! "
    //    "queue max-size-buffers=1 leaky=downstream ! "
    //    "udpsink host=%s port=5000 sync=false async=false",
    //    host
    //);

    // FULL TEST 


        //gchar* pipeStr = g_strdup_printf(
        //    "appsrc is-live=true name=mysrc format=time "
        //    "caps=video/x-raw,format=I420 ! "
        //    "queue max-size-buffers=1 leaky=downstream !"
        //    "videoconvert ! "
        //    "nvh264enc repeat-sequence-header=true preset=low-latency tune=ultra-low-latency zerolatency=true "
        //    "rc-mode=cbr bitrate=0 gop-size=5 bframes=0 cabac=true "
        //    "qp-min-i=20 qp-max-i=30 qp-min-p=20 qp-max-p=30 "
        //    "! video/x-h264,profile=high ! "
        //    "rtph264pay config-interval=1 pt=96 mtu=1200 ! "
        //    "queue max-size-buffers=1 leaky=downstream ! "
        //    "udpsink host=%s port=5000 sync=false async=false",
        //    host
        //);

    // 
    //    gchar* pipeStr = g_strdup_printf(
    //    "appsrc is-live=true name=mysrc format=time "
    //    "caps=video/x-raw,format=I420,framerate=30/1,width=1280,height=720 ! "
    //    "queue max-size-buffers=1 leaky=downstream ! "
    //    "videoconvert ! "
    //    "nvh264enc repeat-sequence-header=true preset=low-latency tune=ultra-low-latency zerolatency=true "
    //    "rc-mode=cbr bitrate=0 gop-size=10 bframes=0 cabac=true "
    //    "qp-min-i=20 qp-max-i=30 qp-min-p=20 qp-max-p=30 "
    //    "! video/x-h264,profile=baseline ! "
    //    "rtph264pay config-interval=1 pt=96 mtu=1200 ! "
    //    "queue max-size-buffers=1 leaky=downstream ! "
    //    "udpsink host=%s port=5000 sync=false async=false",
    //    host
    //);

    //gchar* pipeStr = g_strdup_printf(
    //    "appsrc is-live=true name=mysrc format=time "
    //    "caps=video/x-raw,format=I420 ! "
    //    "queue max-size-buffers=1 leaky=downstream ! "
    //    "videoconvert ! "
    //    "nvh264enc preset=llhp tune=zerolatency rc-mode=cbr bitrate=6000 "
    //    "gop-size=1 bframes=0 cabac=false "
    //    "qp-min-i=20 qp-max-i=30 qp-min-p=20 qp-max-p=30 ! "
    //    "video/x-h264,profile=baseline ! "
    //    "rtph264pay config-interval=1 pt=96 mtu=1200 ! "
    //    "queue max-size-buffers=1 leaky=downstream ! "
    //    "udpsink host=%s port=5000 sync=false async=false",
    //    host
    //);



    //gchar* pipeStr = g_strdup_printf(
    //    "appsrc is-live=true format=time name=mysrc caps=video/x-raw,format=I420 !"
    //    "queue max-size-buffers=5 leaky=downstream !"
    //    "videoconvert !"
    //    "nvh264enc bitrate=20000 gop-size=10 repeat-sequence-header=true preset=hq rc-mode=cbr "
    //    "bframes=0 cabac=true spatial-aq=true aq-strength=8 qp-min-i=16 qp-max-i=26 qp-min-p=18 qp-max-p=28 !"
    //    "video/x-h264,profile=high !"
    //    "rtph264pay config-interval=1 pt=96 mtu=1200 !"
    //    "queue max-size-time=100000000 !" 
    //    "udpsink host=%s port=5000",
    //    host
    //);

    //gchar* pipeStr = g_strdup_printf(
    //    "appsrc is-live=true name=mysrc format=time caps=video/x-raw,format=I420 !"
    //    "queue max-size-buffers=5 leaky=downstream !"
    //    "videoconvert !"
    //    "nvh264enc bitrate=20000 gop-size=10 repeat-sequence-header=true preset=hq rc-mode=cbr "
    //    "bframes=0 cabac=true spatial-aq=true aq-strength=8 qp-min-i=16 qp-max-i=26 qp-min-p=18 qp-max-p=28 !"
    //    "video/x-h264,profile=high !"
    //    "rtph264pay config-interval=1 pt=96 mtu=1200 !"
    //    "queue max-size-time=100000000 !"
    //    "udpsink host=%s port=5000",
    //    host
    //);


    cout << pipeStr << endl;

    GError* err = nullptr;
    data.pipeline = gst_parse_launch(pipeStr, &err);
    checkErr(err);
    MY_ASSERT(data.pipeline);
    // Find our appsrc by name
    data.srcVideo = gst_bin_get_by_name(GST_BIN(data.pipeline), "mysrc");
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
    g_free(pipeStr);
    return 0;
}
