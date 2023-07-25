#include "grasp_utils/GraspExecutor.hpp"
#include "grasp_utils/GraspDetector.hpp"

// STD
#include <string>

// Just print the id of the first grasp in the array
void graspsCallback(const grasp_utils::GraspArray::ConstPtr &msg){

  moveit_msgs::Grasp first_grasp = msg->array[0];
  ROS_INFO("I heard: [%s]", first_grasp.id.c_str());

}

// opens the namespace
namespace grasp_utils
{

  GraspExecutor::GraspExecutor(ros::NodeHandle &nodeHandle): nodeHandle_(nodeHandle)
  {
    // Grasp Executor shoud subscribe to the /grasps topic
    subscriber_ = nodeHandle_.subscribe("grasps",1000,graspsCallback);

    // Confirm the node launched.
    ROS_INFO("Successfully launched node.");

  }

  // Destructor for GraspExecutor class.
  GraspExecutor::~GraspExecutor(){}

} /* namespace */
