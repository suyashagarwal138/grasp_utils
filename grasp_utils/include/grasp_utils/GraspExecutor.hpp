#pragma once

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Temperature.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>

#include <vector>

// Message types
#include <moveit_msgs/Grasp.h>
#include <grasp_utils/GraspArray.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

const double tau = 2 * M_PI;


namespace grasp_utils
{

  // Class to handle the node that interfaces with ROS MoveIt and subscribes to the /grasps topic.
  class GraspExecutor
  {
  public:
    /*!
     * Constructor.
     * @param nodeHandle the ROS node handle.
     */
    GraspExecutor(ros::NodeHandle &nodeHandle);

    void openGripper(trajectory_msgs::JointTrajectory& posture)
    {
      /* Add both finger joints of panda robot. */
      posture.joint_names.resize(2);
      posture.joint_names[0] = "panda_finger_joint1";
      posture.joint_names[1] = "panda_finger_joint2";

      /* Set them as open. */
      posture.points.resize(1);
      posture.points[0].positions.resize(2);
      posture.points[0].positions[0] = 0.04;
      posture.points[0].positions[1] = 0.04;
      posture.points[0].time_from_start = ros::Duration(0.5);
    }

    void closedGripper(trajectory_msgs::JointTrajectory& posture)
    {
      /* Add both finger joints of panda robot. */
      posture.joint_names.resize(2);
      posture.joint_names[0] = "panda_finger_joint1";
      posture.joint_names[1] = "panda_finger_joint2";

      /* Set them as closed. */
      posture.points.resize(1);
      posture.points[0].positions.resize(2);
      posture.points[0].positions[0] = 0.00;
      posture.points[0].positions[1] = 0.00;
      posture.points[0].time_from_start = ros::Duration(0.5);
    }
    
    void pick(moveit::planning_interface::MoveGroupInterface& move_group)
    {
      // For now, just attempting 1 grasp to get the pipline running. 
      // Later, will change so that a GraspArray is sent to MoveIt. 

      // Setting grasp pose
      // ++++++++++++++++++++++
      // This is the pose of panda_link8. |br|
      // Make sure that when you set the grasp_pose, you are setting it to be the pose of the last link in
      // your manipulator which in this case would be `"panda_link8"` You will have to compensate for the
      // transform from `"panda_link8"` to the palm of the end effector.
      grasp_.grasp_pose.header.frame_id = "panda_link0";

      // Set the orientation of the chosen grasp (wasn't done in grasp_det_node.cpp)
      tf2::Quaternion orientation;
      orientation.setRPY(-tau / 4, -tau / 8, -tau / 4);
      grasp_.grasp_pose.pose.orientation = tf2::toMsg(orientation);

      // Setting pre-grasp approach
      // ++++++++++++++++++++++++++
      /* Defined with respect to frame_id */
      grasp_.pre_grasp_approach.direction.header.frame_id = "panda_link0";
      /* Direction is set as positive x axis */
      grasp_.pre_grasp_approach.direction.vector.x = 1.0;
      grasp_.pre_grasp_approach.min_distance = 0.095;
      grasp_.pre_grasp_approach.desired_distance = 0.115;

      // Setting post-grasp retreat
      // ++++++++++++++++++++++++++
      /* Defined with respect to frame_id */
      grasp_.post_grasp_retreat.direction.header.frame_id = "panda_link0";
      /* Direction is set as positive z axis */
      grasp_.post_grasp_retreat.direction.vector.z = 1.0;
      grasp_.post_grasp_retreat.min_distance = 0.1;
      grasp_.post_grasp_retreat.desired_distance = 0.25;

      // Setting posture of eef before grasp
      // +++++++++++++++++++++++++++++++++++
      openGripper(grasp_.pre_grasp_posture);

      // Setting posture of eef during grasp
      // +++++++++++++++++++++++++++++++++++
      closedGripper(grasp_.grasp_posture);

      // Set support surface as table1.
      move_group.setSupportSurfaceName("table1");
      // Call pick to pick up the object using the grasp(s) given
      move_group.pick("object", grasp_);
    }
    
    void place(moveit::planning_interface::MoveGroupInterface& group)
    {
      // Calling place function may lead to "All supplied place locations failed. Retrying last
      // location in verbose mode." This is a known issue.

      // Can create a vector of place locations to be attempted
      // Currently only attempting one
      std::vector<moveit_msgs::PlaceLocation> place_location;
      place_location.resize(1);

      // Setting place location pose
      // +++++++++++++++++++++++++++
      place_location[0].place_pose.header.frame_id = "panda_link0";
      tf2::Quaternion orientation;
      orientation.setRPY(0, 0, tau / 4);  // A quarter turn about the z-axis
      place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

      /* For place location, we set the value to the exact location of the center of the object. */
      place_location[0].place_pose.pose.position.x = 0;
      place_location[0].place_pose.pose.position.y = 0.5;
      place_location[0].place_pose.pose.position.z = 0.5;

      // Setting pre-place approach
      // ++++++++++++++++++++++++++
      /* Defined with respect to frame_id */
      place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
      /* Direction is set as negative z axis */
      place_location[0].pre_place_approach.direction.vector.z = -1.0;
      place_location[0].pre_place_approach.min_distance = 0.095;
      place_location[0].pre_place_approach.desired_distance = 0.115;

      // Setting post-grasp retreat
      // ++++++++++++++++++++++++++
      /* Defined with respect to frame_id */
      place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
      /* Direction is set as negative y axis */
      place_location[0].post_place_retreat.direction.vector.y = -1.0;
      place_location[0].post_place_retreat.min_distance = 0.1;
      place_location[0].post_place_retreat.desired_distance = 0.25;

      // Setting posture of eef after placing object
      // +++++++++++++++++++++++++++++++++++++++++++
      /* Similar to the pick case */
      openGripper(place_location[0].post_place_posture);

      // Set support surface as table2.
      group.setSupportSurfaceName("table2");
      // Call place to place the object using the place locations given.
      group.place("object", place_location);
    }


    /*!
     * Destructor.
     */
    virtual ~GraspExecutor();

  private:

    //! ROS node handle.
    ros::NodeHandle &nodeHandle_;

    //! ROS topic subscriber.
    ros::Subscriber subscriber_;

    //! ROS topic name to subscribe to.
    std::string subscriberTopic_;

    // Grasp to execute
    moveit_msgs::Grasp grasp_;
    
  };

} /* namespace */
