#include "grasp_utils/GraspExecutor.hpp"
#include "grasp_utils/GraspDetector.hpp"

// STD
#include <string>


namespace grasp_utils
{

  GraspExecutor::GraspExecutor(ros::NodeHandle &nodeHandle): nodeHandle_(nodeHandle)
  {
    // Grasp Executor shoud subscribe to the /grasps topic
    subscriber_ = nodeHandle_.subscribe("grasps",1000, &GraspExecutor::graspsCallback, this);

    // Publish marker at attempted grasp pose
    marker_pub = nodeHandle_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // Confirm the node launched.
    ROS_INFO("Successfully launched node.");

  }

  // Destructor for GraspExecutor class.
  GraspExecutor::~GraspExecutor(){}

} /* namespace */
