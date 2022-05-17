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
        
        void computeDepthInfo();
        void getAllImages();
        
        cv::Mat getColorImage();
        cv::Mat drawJointImage();
        std::vector<k4a_float3_t> _chestXYZ;
        std::vector<k4a_float2_t> _jointsXY;
        k4abt_joint_t _landmarkJoint;

        k4a::image _colorImage, _depthImage, _xyzImage, _colorDepthImage;
        size_t _num_bodies;
    protected:
        KinectRun *_kr;
        k4a_capture_t _capture;
        k4abt_frame_t _bodyFrame;
        k4a_calibration_t _calibration;
};

#endif