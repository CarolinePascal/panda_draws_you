
#include <robot_arm_tools/Robot.h>
#include <robot_arm_tools/RobotVisualTools.h>
#include <robot_arm_tools/RobotTrajectories.h>

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

    //Initialisation + setting up velocity and acceleration
    robot.init();
    robot.setAcceleration(0.1);
    robot.setVelocity(0.1);

    //Create waypoints
    std::vector<geometry_msgs::Pose> waypoints;
    sketchToWaypoints(waypoints);

    //Retreive drawing plane specifications
    ros::NodeHandle nh;
    std::vector<double> xAxis,yAxis,zAxis,centerPoint;

    if(!nh.getParam("xAxis", xAxis))
    {
        ROS_ERROR("Unable to retrieve drawing plane data !");
        throw std::runtime_error("MISSING PARAMETER");
    }
    if(!nh.getParam("yAxis", yAxis))
    {
        ROS_ERROR("Unable to retrieve drawing plane data !");
        throw std::runtime_error("MISSING PARAMETER");
    }

    if(!nh.getParam("zAxis", zAxis))
    {
        ROS_ERROR("Unable to retrieve drawing plane data !");
        throw std::runtime_error("MISSING PARAMETER");
    }
    if(!nh.getParam("centerPoint", centerPoint))
    {
        ROS_ERROR("Unable to retrieve drawing plane data !");
        throw std::runtime_error("MISSING PARAMETER");
    }

    tf2::Matrix3x3 rotationMatrix = tf2::Matrix3x3(xAxis[0],yAxis[0],zAxis[0],xAxis[1],yAxis[1],zAxis[1],xAxis[2],yAxis[2],zAxis[2]);
    double roll,pitch,yaw;
    rotationMatrix.getRPY(roll,pitch,yaw);

    translateTrajectory(waypoints,-X_SIZE/2,-Y_SIZE/2,0.0);
    geometry_msgs::Point zero;
    zero.x = 0.0;
    zero.y = 0.0;
    zero.z = 0.0;
    rotateTrajectory(waypoints,zero,roll,pitch,yaw);
    translateTrajectory(waypoints,centerPoint[0],centerPoint[1],centerPoint[2]);

    //Perform approach motion
    geometry_msgs::Pose startingPose = waypoints[0];
    startingPose.position.x -= 0.1*zAxis[0];
    startingPose.position.y -= 0.1*zAxis[1];
    startingPose.position.z -= 0.1*zAxis[2];
    robot.goToTarget(startingPose,false);

    startingPose.position.x += 0.1*zAxis[0];
    startingPose.position.y += 0.1*zAxis[1];
    startingPose.position.z += 0.1*zAxis[2];
    robot.goToTarget(startingPose,false);

    //Perform trajectory
    ROS_INFO("Starting robot trajectory computation...");
    robot.runTrajectory(waypoints);

    //Perform retreat motion
    geometry_msgs::Pose endingPose = robot.getCurrentPose();
    endingPose.position.x -= 0.1*zAxis[0];
    endingPose.position.y -= 0.1*zAxis[1];
    endingPose.position.z -= 0.1*zAxis[2];
    robot.goToTarget(endingPose);

    robot.init(false);

    ros::waitForShutdown();
    return 0;
}