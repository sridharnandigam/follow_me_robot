#include <kinect_run/KinectRun.h>

using namespace std;


KinectRun::KinectRun(){
    _device = NULL;
    VERIFY(k4a_device_open(0, &_device), "Open K4A Device failed!");

    // Start camera. Make sure depth camera is enabled.
    k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    deviceConfig.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_1080P;
    //deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_OFF;
    VERIFY(k4a_device_start_cameras(_device, &deviceConfig), "Start K4A cameras failed!");
}
KinectRun::~KinectRun(){
    k4a_device_stop_cameras(_device);
    k4a_device_close(_device);
}

void KinectRun::update(){
    KinectFrame kf(this);
    kf.getSensorCapture();
    kf.bodyFrameCapture();
    kf.locateJoints();
}