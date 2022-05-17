#include <kinect_run/KinectFollower.h>
#include <kinect_run/KinectFrame.h>

#include <Eigen/Eigen>

#include <iostream>

using namespace Eigen;
using namespace std;

KinectFollower::KinectFollower(ros::NodeHandle &nh) : _nh(nh), _pub(_nh.advertise<geometry_msgs::PoseStamped>("computed", 1)) {
}

KinectFollower::~KinectFollower() {}

void KinectFollower::receiveFrame(KinectFrame *frame){
    _landmarkTransform.transform.translation.x = frame->_landmarkJoint.position.xyz.x;
    _landmarkTransform.transform.translation.y = frame->_landmarkJoint.position.xyz.y;
    _landmarkTransform.transform.translation.z = frame->_landmarkJoint.position.xyz.z;
    
    _landmarkTransform.transform.rotation.x = frame->_landmarkJoint.orientation.wxyz.x;
    _landmarkTransform.transform.rotation.y = frame->_landmarkJoint.orientation.wxyz.y;
    _landmarkTransform.transform.rotation.z = frame->_landmarkJoint.orientation.wxyz.z;
    _landmarkTransform.transform.rotation.w = frame->_landmarkJoint.orientation.wxyz.w;

     _num_bodies = frame->_num_bodies;
    follow();

    _pub.publish(_computedTransform);
}

void KinectFollower::follow() {
    /*
        You are going to want to know which axis is X, Y, and Z.
        You should render them by using the "add/TF" in rviz.
        *ROS Axis Colors*
            x - red
            y - green
            z - blue
    */

    /*
        Here's a variable.
        Into it you should put the x, y, and z coordinate of where ROS says
        the Alvar marker is.

        Use broadcast() --- which you will write --- to broadcast where that
        TF is on the map.

        Why? Because this will show you where the robot thinks the thing it
        should follow is.

        Once you've done that. Fire up rviz and take a look. Note that the tf
        jumps around. That's the correct behavior. The program alvar_to_tf
        sends ALL markers that it finds as the TF to "alvar_marker." The bottom
        line is, this is the thing that you want the robot to follow. Any Alvar
        marker in this program is attached to the robot. It's okay that it
        jumps around.

        Okay, now that you've done that. Set the z parameter to 0. What happens
        to the marker?

        You are going to move the robot "marvin" to follow the robot "ar_tag."
        Since Z is the vertical component, you do not need Z in your
        computations. The robots can't fly.

        Call the TF you send in broadcast() "incoming".
    */

    MatrixXd markerPos(3,1);
    markerPos(0,0) = _landmarkTransform.transform.translation.x;
    markerPos(1,0) = _landmarkTransform.transform.translation.y;
    markerPos(2,0) = 0;//transformStamped.transform.translation.z;
    //markerPos(2,0) = _landmarkTransform.transform.translation.z;
markerPos /= 16;
    /*
    broadcast(
        ros::Time::now(),
        markerPos(0,0), markerPos(1,0), markerPos(2,0),
        _landmarkTransform.transform.rotation.x,
        _landmarkTransform.transform.rotation.y,
        _landmarkTransform.transform.rotation.z,
        _landmarkTransform.transform.rotation.w,
        "chest_location"
    );
    */

    double norm = markerPos.norm();
    double angle = atan2(markerPos(1,0) / norm, markerPos(0,0) / norm);

    Eigen::AngleAxisd rotAngle(angle, Vector3d::UnitZ());
    Eigen::Quaterniond rotQuat(rotAngle);

    Eigen::MatrixXd xDir(3,1);
    xDir(0,0) = norm;
    xDir(1,0) = 0;
    xDir(2,0) = 0;

    Eigen::MatrixXd turned = rotQuat.toRotationMatrix() * xDir;

    broadcast(
        ros::Time::now(),
        turned(0,0), turned(1,0), 0,
        _landmarkTransform.transform.rotation.x,
        _landmarkTransform.transform.rotation.y,
        _landmarkTransform.transform.rotation.z,
        _landmarkTransform.transform.rotation.w,
        "chest_location"
    );

    
    //Okay, now we need to actually move the robot.
    _computedTransform.header.frame_id = "computedTransform";
    _computedTransform.header.stamp = ros::Time::now();

    if(norm > 1.0 && _num_bodies > 0){
        _computedTransform.pose.position.x = 0.25 * turned(0, 0);
        _computedTransform.pose.position.y = 0.25 * turned(1, 0);
        _computedTransform.pose.position.z = 0;
    } else{
        _computedTransform.pose.position.x = 0.0;
        _computedTransform.pose.position.y = 0.0;
        _computedTransform.pose.position.z = 0;
    }

    _computedTransform.pose.orientation.x = rotQuat.x();
    _computedTransform.pose.orientation.y = rotQuat.y();
    _computedTransform.pose.orientation.z = rotQuat.z();
    _computedTransform.pose.orientation.w = rotQuat.w();

    /*
    _computedTransform.pose.position.x = 1.0;
    _computedTransform.pose.position.y = 0.0;
    _computedTransform.pose.position.z = 0;

    _computedTransform.pose.orientation.x = 0.0;
    _computedTransform.pose.orientation.y = 0.0;
    _computedTransform.pose.orientation.z = 0.0;
    _computedTransform.pose.orientation.w = 1.0;
    */

    broadcast(
        ros::Time::now(),
        _computedTransform.pose.position.x,
        _computedTransform.pose.position.y,
        _computedTransform.pose.position.z,
        _computedTransform.pose.orientation.x,
        _computedTransform.pose.orientation.y,
        _computedTransform.pose.orientation.z,
        _computedTransform.pose.orientation.w,
        "computed");


    /*

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "marvin/base_footprint";
    goal.target_pose.header.stamp = transformStamped.header.stamp;

    if(norm > 1.0) {
        goal.target_pose.pose.position.x = 0.25 * turned(0,0);
        goal.target_pose.pose.position.y = 0.25 * turned(1,0);
        goal.target_pose.pose.position.z = 0;
    } else {
        goal.target_pose.pose.position.x = 0.0;
        goal.target_pose.pose.position.y = 0.0;
        goal.target_pose.pose.position.z = 0;
    }

    goal.target_pose.pose.orientation.w = rotQuat.w();
    goal.target_pose.pose.orientation.x = rotQuat.x();
    goal.target_pose.pose.orientation.y = rotQuat.y();
    goal.target_pose.pose.orientation.z = rotQuat.z();
            

    ROS_INFO("Sending goal");
    _ac.sendGoal(goal);
    _ac.waitForResult();
    **/
}

void KinectFollower::broadcast(
    ros::Time time,
    double tX, double tY, double tZ, double rX, double rY, double rZ, double rW,
    string name) {
    geometry_msgs::TransformStamped transformStamped;
    printf("begin broadcast\n");

    transformStamped.header.stamp = time;
    transformStamped.header.frame_id = "base_link";
    transformStamped.child_frame_id = name;//"go_here";
    transformStamped.transform.translation.x = tX;
    transformStamped.transform.translation.y = tY;
    transformStamped.transform.translation.z = tZ;
    transformStamped.transform.rotation.x = rX;
    transformStamped.transform.rotation.y = rY;
    transformStamped.transform.rotation.z = rZ;
    transformStamped.transform.rotation.w = rW;

    _br.sendTransform(transformStamped);
}