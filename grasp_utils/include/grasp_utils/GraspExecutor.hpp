#pragma once

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Temperature.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>

#include <vector>

#include <moveit_msgs/Grasp.h>
#include <grasp_utils/GraspArray.h>

// MoveIt integration
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>



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
    
  };

} /* namespace */
