#ifndef KINECTFRAME_H
#define KINECTFRAME_H

#include <k4a/k4a.hpp>
#include <k4abt.h>
#include <opencv2/opencv.hpp>

#include <Eigen/Eigen>

#include <string>
#include <vector>

class KinectRun;

class KinectFrame{
    public:
        KinectFrame(KinectRun *_kr);
        ~KinectFrame();
        void getSensorCapture();
        void bodyFrameCapture();
        void locateJoints();
        cv::Mat convertImage(k4a_image_t inputImage, unsigned char chan);
    protected:
        KinectRun *_kr;
        k4a_capture_t _capture;
        k4abt_frame_t _bodyFrame;
};

#endif