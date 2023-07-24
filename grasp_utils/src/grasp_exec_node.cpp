#include <ros/ros.h>
#include "grasp_utils/GraspExecutor.hpp"
#include "grasp_utils/GraspDetector.hpp"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "grasp_exec");

  //  topics will resolve to ros_package_template/ros_package_template_node/topic
  //  changed this to namespaced node handle
  ros::NodeHandle nodeHandle("grasp_executor");

  grasp_utils::GraspExecutor graspExecutor(nodeHandle);

  ros::Rate loop_rate(0.4);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin();
  return 0;
}
