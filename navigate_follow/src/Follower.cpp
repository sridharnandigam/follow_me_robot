#include <Follower.h>

Follower::Follower(ros::NodeHandle &nh) : _nh(nh), _ac("move_base", true){
    _sub = nh.subscribe("/kinect_follower/computed", 1, &Follower::followCallback, this);

    while(!_ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for move base server");
    }
}

Follower::~Follower(){}

void Follower::followCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "base_footprint";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = msg->pose.position.x;
    goal.target_pose.pose.position.y = msg->pose.position.y;
    goal.target_pose.pose.position.z = msg->pose.position.z;

    goal.target_pose.pose.orientation.x = msg->pose.orientation.x;
    goal.target_pose.pose.orientation.y = msg->pose.orientation.y;
    goal.target_pose.pose.orientation.z = msg->pose.orientation.z;
    goal.target_pose.pose.orientation.w = msg->pose.orientation.w;

    ROS_INFO_STREAM("Sending goal");
    ROS_INFO_STREAM("Position XYZ: " << goal.target_pose.pose.position.x << ", " << goal.target_pose.pose.position.y << ", " << goal.target_pose.pose.position.z);
    ROS_INFO_STREAM("Orientation: " << goal.target_pose.pose.orientation.x << ", " << goal.target_pose.pose.orientation.y << ", " << goal.target_pose.pose.orientation.z << ", " << goal.target_pose.pose.orientation.w << "\n");
    _ac.sendGoal(goal);
    _ac.waitForResult();
}