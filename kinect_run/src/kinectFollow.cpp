#include <kinect_run/KinectFrame.h>
#include <kinect_run/KFRImagePublish.h>
#include <kinect_run/KinectRun.h>
#include <kinect_run/KinectFollower.h>

//#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


int main(int argc, char** argv){
    ros::init(argc, argv, "kinectApril");
    ros::NodeHandle nh("kinect_follower");

    KFRImagePublish kfrip(nh);
    KinectFollower rf;
    
    KinectRun kr(kfrip, rf);

    ros::Rate loop_rate(15);
    while(ros::ok()) {
        kr.update();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}