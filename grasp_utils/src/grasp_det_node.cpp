#include <ros/ros.h>
#include "grasp_utils/GraspExecutor.hpp"
#include "grasp_utils/GraspDetector.hpp"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "grasp_det");

    ros::NodeHandle nodeHandle("grasp_detector");

    grasp_utils::GraspDetector graspDetector(nodeHandle);

    // define the publisher data member of the graspDetector object.
    // set it to advertise a message of type Grasp, which is from the moveit documentation.
    // graspDetector.set_pub(nodeHandle.advertise<moveit_msgs::Grasp>("grasps", 1000));

    graspDetector.set_pub(nodeHandle.advertise<grasp_utils::GraspArray>("grasps", 1000));


    ros::Rate loop_rate(0.4);

    while (ros::ok())
    {

        // declare a grasp message
        moveit_msgs::Grasp grasp_msg;

        grasp_msg.id = "This topic works!";

        // Also set the pose randomly
        grasp_msg.grasp_pose.pose.position.x = 1;
        grasp_msg.grasp_pose.pose.position.y = 0;
        grasp_msg.grasp_pose.pose.position.z = 1;

        grasp_msg.grasp_pose.pose.orientation.w = 0;
        grasp_msg.grasp_pose.pose.orientation.x = 1;
        grasp_msg.grasp_pose.pose.orientation.y = 0;
        grasp_msg.grasp_pose.pose.orientation.z = 1;

        grasp_utils::GraspArray grasp_array;

        for(int i = 0; i < 64; i++){
            // generate 64 random grasp poses
            srand(i);
            int x = rand()%10;
            int y = rand()%10;
            int z = rand()%10;
            moveit_msgs::Grasp grasp;
            grasp.grasp_pose.pose.position.x = x;
            grasp.grasp_pose.pose.position.y = y;
            grasp.grasp_pose.pose.position.z = z;
            grasp.id = "This is grasp pose no. " + std::to_string(i);
            grasp_array.array.push_back(grasp);
        }

        graspDetector.get_pub().publish(grasp_array);

        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::spin();
    return 0;
}