#include <kinect_run/KinectFrame.h>
#include <kinect_run/KFRImagePublish.h>
#include <kinect_run/KinectRun.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
    ros::init(argc, argv, "kinectApril");
    ros::NodeHandle nh("kinect_follower");

    MoveBaseClient ac("move_base", true); //initialise client and start ros spin

    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for move_base action server....");
    }

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();

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