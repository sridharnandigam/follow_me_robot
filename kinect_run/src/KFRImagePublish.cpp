#include <kinect_run/KFRImagePublish.h>
#include <kinect_run/KinectFrame.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


KFRImagePublish::KFRImagePublish(ros::NodeHandle &nh) :
    _it(nh), _pub(_it.advertise("joint_tracking", 1)) {

}

KFRImagePublish::~KFRImagePublish() {}

void KFRImagePublish::receiveFrame(KinectFrame *frame) {
    //cv::Mat *m = frame->_cvMats["bgr"];
    cv::Mat m = frame->drawJointImage();

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "8UC4", m).toImageMsg();
    //ROS_INFO_STREAM("w: " << m->cols << "   h:  " << m->rows);
    _pub.publish(msg);
    //cv::imshow("joint_track", m);
    //cv::waitKey(1);
}