#include <kinect_run/KinectFrame.h>
#include <kinect_run/KFRImagePublish.h>
#include <kinect_run/KinectRun.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "kinectApril");
    ros::NodeHandle nh("kinect_follower");

    KFRImagePublish kfrip(nh);
    
    KinectRun kr(kfrip);

    ros::Rate loop_rate(15);
    while(ros::ok()) {
        kr.update();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}