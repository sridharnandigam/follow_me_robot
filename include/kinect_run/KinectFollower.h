#ifndef ROBOT_FOLLOWER_H
#define ROBOT_FOLLOWER_H

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/TransformStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2_ros/transform_broadcaster.h>

#include <kinect_run/KinectFrameRecepient.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/*
  MoveBaseClient is defined in the tutorial:
  http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals

  You should follow that tutorial if you get lost on this part of the
  assignment.

*/

class RobotFollower : public KinectFrameRecepient{
public:
  RobotFollower();
  ~RobotFollower();
  void receiveFrame(KinectFrame *frame);
  void follow(geometry_msgs::TransformStamped transformStamped);

protected:
  void broadcast(
    ros::Time time,
    double tX, double tY, double tZ, double rX, double rY, double rZ, double rW,
    std::string name);

    geometry_msgs::TransformStamped _landmarkTransform;

  MoveBaseClient _ac;
  tf2_ros::TransformBroadcaster _br;
};

#endif