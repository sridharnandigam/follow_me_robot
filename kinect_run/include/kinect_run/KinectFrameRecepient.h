#ifndef KINECT_FRAME_RECIPIENT_H
#define KINECT_FRAME_RECIPIENT_H

class KinectFrame;
class KinectFrameRecipient {
public:
    virtual void receiveFrame(KinectFrame *frame) = 0;
};

#endif