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
#include <visualization_msgs/Marker.h>
#include <moveit_msgs/GetPositionIK.h>
#include <sensor_msgs/JointState.h>

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
    
    // Old version of pick which took move_group as arg
    // void pick(moveit::planning_interface::MoveGroupInterface& move_group)
    // {
    //   // For now, just attempting 1 grasp to get the pipline running. 
    //   // Later, will change so that a GraspArray is sent to MoveIt. 
    //   // Setting grasp pose
    //   // ++++++++++++++++++++++
    //   // This is the pose of panda_link8. |br|
    //   // Make sure that when you set the grasp_pose, you are setting it to be the pose of the last link in
    //   // your manipulator which in this case would be `"panda_link8"` You will have to compensate for the
    //   // transform from `"panda_link8"` to the palm of the end effector.
    //   grasp_.grasp_pose.header.frame_id = "panda_link0";
    //   // Set the orientation of the chosen grasp (wasn't done in grasp_det_node.cpp)
    //   tf2::Quaternion orientation(1.0,0.0,0.0,0.0);                                   // chosen this as default orientation
    //   // orientation.setRPY(-tau / 4, -tau / 8, -tau / 4);
    //   grasp_.grasp_pose.pose.orientation = tf2::toMsg(orientation);
    //   // Setting pre-grasp approach
    //   // ++++++++++++++++++++++++++
    //   /* Defined with respect to frame_id */
    //   grasp_.pre_grasp_approach.direction.header.frame_id = "panda_link0";
    //   /* Direction is set as positive x axis */
    //   grasp_.pre_grasp_approach.direction.vector.x = 1.0;
    //   grasp_.pre_grasp_approach.min_distance = 0.095;
    //   grasp_.pre_grasp_approach.desired_distance = 0.115;
    //   // Setting post-grasp retreat
    //   // ++++++++++++++++++++++++++
    //   /* Defined with respect to frame_id */
    //   grasp_.post_grasp_retreat.direction.header.frame_id = "panda_link0";
    //   /* Direction is set as positive z axis */
    //   grasp_.post_grasp_retreat.direction.vector.z = 1.0;
    //   grasp_.post_grasp_retreat.min_distance = 0.1;
    //   grasp_.post_grasp_retreat.desired_distance = 0.25;
    //   // Setting posture of eef before grasp
    //   // +++++++++++++++++++++++++++++++++++
    //   openGripper(grasp_.pre_grasp_posture);
    //   // Setting posture of eef during grasp
    //   // +++++++++++++++++++++++++++++++++++
    //   closedGripper(grasp_.grasp_posture);
    //   // Set support surface as table1.
    //   move_group.setSupportSurfaceName("table1");
    //   // Call pick to pick up the object using the grasp(s) given
    //   move_group.pick("object", grasp_);
    // }
    
    // Using this version which takes grasp as input
    void pick(moveit_msgs::Grasp grasp_)
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
      group.setSupportSurfaceName("table1");
      // Call pick to pick up the object using the grasp(s) given
      group.pick("object", grasp_);
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

    // Add graspsCallback as a member functionn
    void graspsCallback(const grasp_utils::GraspArray::ConstPtr &msg);

    // Add another callback function for the joint state
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);

    // Add a function to perform IK checks
    bool IKchecker(moveit_msgs::Grasp grasp);

    /*!
     * Destructor.
     */
    virtual ~GraspExecutor();

    // assign movegroup here as member variable
    moveit::planning_interface::MoveGroupInterface group{"panda_arm"};

    // Need to store the joint state in the class
    sensor_msgs::JointState current_joint_state;

  private:

    // ROS node handle.
    ros::NodeHandle &nodeHandle_;

    // ROS topic subscriber for grasp arrays
    ros::Subscriber subscriber_;

    // ROS topic subscriber for joint  state
    ros::Subscriber js_subscriber;

    // ROS publisher for Rviz marker
    ros::Publisher marker_pub;

    // Service client for IK checker
    ros::ServiceClient ik_client  = nodeHandle_.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");

  };

  // Define the comparison function for sorting the grasp array
  bool compare(const moveit_msgs::Grasp& grasp1, const moveit_msgs::Grasp& grasp2){
    return grasp1.grasp_quality  > grasp2.grasp_quality;
  };

  // Define member function graspsCallback
  void GraspExecutor::graspsCallback(const grasp_utils::GraspArray::ConstPtr &msg){

    int no_of_elements = msg->array.size();

    // Initialise two variables that we will use in the loop to find the best grasp
    float best_grasp_quality = 0.0;
    int best_grasp_id = 0;

    // Loop through the array of grasps to find the highest grasp quality
    for(int i = 0; i < no_of_elements; i++){
      if (msg->array[i].grasp_quality > best_grasp_quality){
        best_grasp_quality = msg->array[i].grasp_quality;
        best_grasp_id = i;
      }
    };

    std::vector<moveit_msgs::Grasp> sorted_grasps;
    sorted_grasps.resize(no_of_elements);

    std::partial_sort_copy(msg->array.begin(),msg->array.end(),sorted_grasps.begin(),sorted_grasps.end(),compare);

    // debugging outputs to confirm sorting works
    ROS_INFO("%f",best_grasp_quality);
    ROS_INFO("%f",sorted_grasps[0].grasp_quality);

    // Output the best grasp.
    // For a 64-grasp array, this will be a number from 0  to 63. 
    std::string output_msg = "Best grasp was no. " + std::to_string(best_grasp_id);
    ROS_INFO("Best grasp was no. %d", best_grasp_id);

    // Set the best grasp so it can be executed
    moveit_msgs::Grasp best_grasp = msg->array[best_grasp_id];

    // Print result of IK check
    if(IKchecker(best_grasp)){
      ROS_INFO("successful IK check");
    }
    else{
      ROS_INFO("unsuccessful IK check");
    }

    // Set the maximum planning time
    group.setPlanningTime(45.0);

    // Debugging outputs - delete later
    ROS_INFO("%f",best_grasp.grasp_pose.pose.position.x);
    ROS_INFO("%f",best_grasp.grasp_pose.pose.position.y);
    ROS_INFO("%f",best_grasp.grasp_pose.pose.position.z);

    while (ros::ok())
    {
      visualization_msgs::Marker marker;
      // Set the frame ID and timestamp.  See the TF tutorials for information on these.
      marker.header.frame_id = "panda_link0";
      marker.header.stamp = ros::Time::now();

      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one
      marker.ns = "basic_shapes";
      marker.id = 0;

      // Set the marker type.
      marker.type = visualization_msgs::Marker::CUBE;

      // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
      marker.action = visualization_msgs::Marker::ADD;

      // Set the pose of the marker (should be at pose of attempted grasp)
      marker.pose.position.x = best_grasp.grasp_pose.pose.position.x;
      marker.pose.position.y = best_grasp.grasp_pose.pose.position.y;
      marker.pose.position.z = best_grasp.grasp_pose.pose.position.z;
      marker.pose.orientation.x = best_grasp.grasp_pose.pose.orientation.x;
      marker.pose.orientation.y = best_grasp.grasp_pose.pose.orientation.y;
      marker.pose.orientation.z = best_grasp.grasp_pose.pose.orientation.z;
      marker.pose.orientation.w = best_grasp.grasp_pose.pose.orientation.w;

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;

      // Set the color -- be sure to set alpha to something non-zero!
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;

      marker.lifetime = ros::Duration();

      marker_pub.publish(marker);      

      // call pick using the best grasp
      pick(best_grasp);
    }

    
  }

  // Checks if inverse kinematics was successful
  bool GraspExecutor::IKchecker(moveit_msgs::Grasp grasp){

    moveit_msgs::GetPositionIK::Response res;
    moveit_msgs::GetPositionIK::Request req;

    // Assigning the required information to the IK request
    req.ik_request.pose_stamped = grasp.grasp_pose;
    req.ik_request.group_name = "panda_arm";
    req.ik_request.robot_state.joint_state = current_joint_state;

    // call the IK service, checking that the request and response parameters were
    // autogenerated from the same srv file.
    if(ik_client.call(req,res)){
      if(res.error_code.val != res.error_code.SUCCESS){
        return false;
      }
      else{
        return true;
      }
    }
    else{
      ROS_INFO("call to service failed");     
      return false;
    }
    
  }

  // Another callback function is needed to process the joint_states for IK checking
  void GraspExecutor::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg){
    GraspExecutor::current_joint_state =  *msg;
  }

} /* namespace */


