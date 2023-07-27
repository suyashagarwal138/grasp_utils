#include "grasp_utils/GraspExecutor.hpp"
#include "grasp_utils/GraspDetector.hpp"

// STD
#include <string>

moveit_msgs::Grasp best_grasp;

// Just print the id of the first grasp in the array
void graspsCallback(const grasp_utils::GraspArray::ConstPtr &msg){

  // No longer need this stuff
  // moveit_msgs::Grasp first_grasp = msg->array[0];
  // ROS_INFO("I heard: [%s]", first_grasp.id.c_str());

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

  // Output the best grasp.
  // For a 64-grasp array, this will be a number from 0  to 63. 
  std::string output_msg = "Best grasp was no. " + std::to_string(best_grasp_id);
  ROS_INFO("Best grasp was no. %d", best_grasp_id);

  // Set the best grasp so it can be executed
  best_grasp = msg->array[best_grasp_id];
}

// opens the namespace
namespace grasp_utils
{

  GraspExecutor::GraspExecutor(ros::NodeHandle &nodeHandle): nodeHandle_(nodeHandle)
  {
    // Grasp Executor shoud subscribe to the /grasps topic
    subscriber_ = nodeHandle_.subscribe("grasps",1000,graspsCallback);

    grasp_ = best_grasp;

    // Confirm the node launched.
    ROS_INFO("Successfully launched node.");

  }

  // Destructor for GraspExecutor class.
  GraspExecutor::~GraspExecutor(){}

} /* namespace */
