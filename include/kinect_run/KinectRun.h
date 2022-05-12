#ifndef KINECTRUN_H
#define KINECTRUN_H

#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <k4a/k4a.hpp>
#include <k4abt.h>
#include <kinect_run/KinectFrame.h>
#include <kinect_run/KinectFrameRecepient.h>

#include <opencv2/opencv.hpp>

#include <Eigen/Eigen>

#define VERIFY(result, error)                                                                            \
    if(result != K4A_RESULT_SUCCEEDED)                                                                   \
    {                                                                                                    \
        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
        exit(1);                                                                                         \
    }   

class KinectRun{
    public:
        k4a_capture_t _capture;
        k4a_device_t _device;
        k4a::transformation _transformation;

        KinectFrameRecipient &_kfr1;
        KinectFrameRecipient &_kfr2;

        k4abt_tracker_t tracker;

        KinectRun(KinectFrameRecipient &kfr1, KinectFrameRecipient &kfr2);
        ~KinectRun();

        void update();
};
#endif