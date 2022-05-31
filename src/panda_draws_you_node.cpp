
#include <ros/ros.h>
#include <robot_arm_tools/Robot.h>
#include <robot_arm_tools/RobotVisualTools.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "SketchToWaypoints.hpp"

#include "panda_draws_you/FileExplorer.h"

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
    visualTools.setupOptiTrack();

    //Move the robot to its initial configuration
    robot.init();
    robot.setAcceleration(0.1);
    robot.setVelocity(0.1);

    //Create waypoints
    std::vector<geometry_msgs::Pose> waypoints;

    tf2::Quaternion quaternion;
    geometry_msgs::Quaternion quaternionMsg;
    //quaternion.setRPY(M_PI,0,0);
    quaternion.setRPY(0,M_PI/2,0);
    quaternionMsg = tf2::toMsg(quaternion);
    
    //QApplication app(argc,argv);
    //FileExplorer explorer(ros::package::getPath("panda_draws_you")+"/config/ScienceDay");
    //explorer.openFile();
    //std::string path = explorer.getFilePath();

    //sketchToWaypoints(waypoints,-0.01,quaternionMsg);
    sketchToWaypoints(waypoints,1.03,quaternionMsg);

    //Perform approach motion
    geometry_msgs::Pose startingPose = waypoints[0];

    //startingPose.position.z += 0.02;
    startingPose.position.x -= 0.1;
    robot.goToTarget(startingPose);

    //startingPose.position.z -= 0.02;
    startingPose.position.x += 0.1;
    robot.goToTarget(startingPose);

    //Perform trajectory
    ROS_INFO("Starting robot trajectory computation...");
    robot.runTrajectory(waypoints);

    //Perform retreat motion
    geometry_msgs::Pose endingPose = robot.getCurrentPose();
    //endingPose.position.z += 0.02;
    endingPose.position.x -= 0.02;
    robot.goToTarget(endingPose);

    robot.init(false);

    ros::waitForShutdown();
    return 0;
}