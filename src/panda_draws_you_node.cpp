
#include <ros/ros.h>
#include <robot_arm_tools/Robot.h>
#include <robot_arm_tools/RobotVisualTools.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "SketchToWaypoints.hpp"

int main(int argc, char **argv)
{
    //ROS node initialisation
    ros::init(argc, argv, "panda_draws_you_node");  
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::WallDuration(1.0).sleep();

    //Robot initialisation
    Robot robot;

    //Robot visual tools initialisation
    RobotVisualTools visualTools;

    //Move the robot to its initial configuration
    robot.init();

    //Create waypoints
    std::vector<geometry_msgs::Pose> waypoints;

    tf2::Quaternion quaternion;
    geometry_msgs::Quaternion quaternionMsg;
    quaternion.setRPY(M_PI,0,0);
    quaternionMsg = tf2::toMsg(quaternion);

    ROS_INFO("Starting sketch to waypoints conversion...");

    sketchToWaypoints("/home/student/Desktop/Picture3.png",waypoints,-0.01,quaternionMsg);

    //Perform approach motion
    geometry_msgs::Pose startingPose = waypoints[0];

    startingPose.position.z += 0.02;
    robot.goToTarget(startingPose);

    startingPose.position.z -= 0.02;
    robot.goToTarget(startingPose);

    //Perform trajectory
    ROS_INFO("Starting robot trajectory computation...");
    robot.runTrajectory(waypoints);

    //Perform retreat motion
    geometry_msgs::Pose endingPose = robot.getCurrentPose();
    endingPose.position.z += 0.02;
    robot.goToTarget(endingPose);

    ros::waitForShutdown();
    return 0;
}