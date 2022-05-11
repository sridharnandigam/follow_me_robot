#ifndef KFR_IMAGE_PUBLISH_H
#define KFR_IMAGE_PUBLISH_H

#include <kinect_pkg/KinectFrameRecipient.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

class KFRImagePublish : public KinectFrameRecipient {
protected:
    image_transport::ImageTransport _it;
    image_transport::Publisher _pub;

public:
    KFRImagePublish(ros::NodeHandle &nh);
    ~KFRImagePublish();

    void receiveFrame(KinectFrame *frame);
};

#endif