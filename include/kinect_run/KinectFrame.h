#ifndef KINECTFRAME_H
#define KINECTFRAME_H

#include <KinectRun.h>

class KinectFrame(KinectRun *_kr){
    public:
        KinectFrame();
        ~KinectFrame();
        void getSensorCapture();
    protected:
        KinectRun *_kr;
        k4a_capture_t _capture;
        k4a_frame_t _bodyFrame;
}

#endif