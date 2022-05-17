#include <Follower.h>

#include <ros/ros.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "navigateFollower");
    ros::NodeHandle nh("navigate_follow");

    Follower f(nh);

    /**
    ros::Rate loop_rate(1);
    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    **/
    ros::spinOnce();

    return 0;
}