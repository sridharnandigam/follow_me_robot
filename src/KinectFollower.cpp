#include <kinect_run/KinectFollower.h>
#include <kinect_run/KinectFrame.h>

#include <Eigen/Eigen>

#include <iostream>

using namespace Eigen;
using namespace std;

KinectFollower::KinectFollower() : _ac("/marvin/move_base", true) {
    /*
    while(!_ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    */
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

    follow();
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

    broadcast(
        _landmarkTransform.header.stamp,
        markerPos(0,0), markerPos(1,0), markerPos(2,0),
        _landmarkTransform.transform.rotation.x,
        _landmarkTransform.transform.rotation.y,
        _landmarkTransform.transform.rotation.z,
        _landmarkTransform.transform.rotation.w,
        "chest location"
    );

    /*
        broadcast(
            );
    */

    /*
        Compute the norm of markerPos. This is the distance to the robot that
        you are following. Store it in a variable. Call it "norm."

        Compute the angle between facing straightforward and the angle that
        "marvin" could turn to to face "ar_tag." You can do this using atan2.
        Store the result in a variable called "angle."

        Use Eigen::AngleAxisd to compute a rotation such that, if the robot
        followed that rotation, it would be facing the ar_tag robot.
        
        Here, I've made you a variable. xDir. Put "norm" in xDir's x component
        and make the others 0, so that xDir is facing the x direction and is as
        long as the distance between the two robots.

        For navigation purposes, "x" is "forward."

        Using the rotation that you've just computed, rotate xDir. Call the 
        result "turned."
        
        Now, use broadcast() to broadcast the result of the above computations,
        the rotation that you computed is the rotation, and "turned" is the 
        translation. Call the TF "turned."

        If you have performed this computation correctly, and you are
        broadcasting both "incoming" and "turned," then "incoming" and "turned"
        should be on top of each other and facing the same direction.
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
        _landmarkTransform.header.stamp,
        turned(0,0), turned(1,0), turned(2,0),
        rotQuat.x(),
        rotQuat.y(),
        rotQuat.z(),
        rotQuat.w(),
        "chosen");

    
    //Okay, now we need to actually move the robot.

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
    transformStamped.header.frame_id = "marvin/base_link";
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