#include <kinect_pkg/KFRImagePublish.h>
#include <kinect_pkg/KinectFrame.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


KFRImagePublish::KFRImagePublish(ros::NodeHandle &nh) :
    _it(nh), _pub(_it.advertise("/azure_kinect/bgr", 1)) {

}

KFRImagePublish::~KFRImagePublish() {}

void KFRImagePublish::receiveFrame(KinectFrame *frame) {
    cv::Mat *m = frame->_cvMats["bgr"];
    sensor_msgs::ImagePtr msg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", *m).toImageMsg();
    //ROS_INFO_STREAM("w: " << m->cols << "   h:  " << m->rows);
    _pub.publish(msg);
    //cv::imshow("WIND", *frame->_cvMats["bgra"]);
    //cv::waitKey(1);
}