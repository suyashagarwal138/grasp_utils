#include <ros/ros.h>
#include "grasp_utils/GraspExecutor.hpp"
#include "grasp_utils/GraspDetector.hpp"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "grasp_exec");

  ros::NodeHandle nodeHandle("grasp_executor");

  grasp_utils::GraspExecutor graspExecutor(nodeHandle);

  ros::Rate loop_rate(0.4);

  // while (ros::ok())
  // {
  //   ros::spinOnce();
  //   loop_rate.sleep();
  // }

  // Declare a MoveGroupInterface that we can use for picking and placing
  moveit::planning_interface::MoveGroupInterface group("panda_arm");

  // Set the maximum planning time
  group.setPlanningTime(45.0);

  // call the pick function
  graspExecutor.pick(group);

  ros::spin();
  return 0;
}
