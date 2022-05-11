#include <kinect_run/KinectRun.h>

using namespace std;


KinectRun::KinectRun(){
    _device = NULL;
    VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!");

    // Start camera. Make sure depth camera is enabled.
    k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    deviceConfig.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_1080P;
    //deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_OFF;
    VERIFY(k4a_device_start_cameras(_device, &deviceConfig), "Start K4A cameras failed!");
}

KinectRun::cv::Mat convertImage(k4a_image_t inputImage, unsigned char chan){
    uint8_t* imgBuffer = k4a_image_get_buffer(inputImage);
    int height = k4a_image_get_height_pixels(inputImage);
    int width = k4a_image_get_width_pixels(inputImage);

    cv::Mat retImage(height, width, chan, (void*) imgBuffer, cv::Mat::AUTO_STEP);

    cv::imshow("test", retImage);
    cvWaitKey(30);

    return retImage;
}

KinectRun::void update(){
    KinectFrame kf(this);
    kf.getSensorCapture();
}