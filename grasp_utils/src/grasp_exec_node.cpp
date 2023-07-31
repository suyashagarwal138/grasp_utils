#include <ros/ros.h>
#include "grasp_utils/GraspExecutor.hpp"
#include "grasp_utils/GraspDetector.hpp"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "grasp_exec");

  ros::NodeHandle nodeHandle("grasp_executor");

  grasp_utils::GraspExecutor graspExecutor(nodeHandle);

  ros::Rate loop_rate(0.4);

  // Declare the planning scene interface - not used for now
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  ros::spin();
  return 0;
}
