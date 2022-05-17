#ifndef FOLLOWER_H
#define FOLLOWER_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_broadcaster.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Follower{
    protected:;
    ros::NodeHandle _nh;
    ros::Subscriber _sub;

    MoveBaseClient _ac;

    public:
    Follower(ros::NodeHandle &nh);
    ~Follower();

    void followCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
};

#endif