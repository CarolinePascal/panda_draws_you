#include <robot_arm_tools/Robot.h>
#include <robot_arm_tools/RobotVisualTools.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <yaml-cpp/yaml.h>

//std::vector<double> getPlaneEquation(std::vector<geometry_msgs::) 

int main(int argc, char **argv)
{ 
    //ROS node initialisation
    ros::init(argc, argv, "panda_draws_you_plane_calibration_node");  
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::WallDuration(1.0).sleep();

    //Robot initialisation
    Robot robot;

    //Robot visual tools initialisation
    RobotVisualTools visualTools;
    visualTools.setupOptiTrack();

    //Move the robot to its initial configuration
    //robot.init();
    robot.setAcceleration(0.1);
    robot.setVelocity(0.1);

    //Tf listenner initialisation
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    //Retreive end effector name
    ros::NodeHandle n;
    std::string endEffectorName;
    if(!n.getParam("endEffectorName",endEffectorName))
    {
        ROS_ERROR("Unable to retrieve measurement server name !");
        throw std::runtime_error("MISSING PARAMETER");
    }

    //Switch the robot to manual control mode (user action required)
    do 
    {
        ROS_INFO("Switch the robot to manual control mode, and move the tool to the central calibration position - Press enter to continue");
    } while (std::cin.get() != '\n');

    //Retreive central calibration point and orientation
    geometry_msgs::Transform transform;
    try
    {
        transform = tfBuffer.lookupTransform("world", endEffectorName, ros::Time(0), ros::Duration(5.0)).transform;
    } 
    catch (tf2::TransformException &ex) 
    {
        throw std::runtime_error("CANNOT RETRIVE SEEKED TRANSFORM !");
    }

    ROS_INFO("Central calibration point : \n X : %f \n Y : %f \n Z : %f",transform.translation.x,transform.translation.y,transform.translation.z);

    do 
    {
        ROS_INFO("Switch the robot to automatic control mode - Press enter to continue");
    } while (std::cin.get() != '\n');

    robot.triggerRobotStartup();

    std::vector<geometry_msgs::Pose> calibrationPoses;
    geometry_msgs::Pose initialPose,currentPose;
    initialPose.orientation = transform.rotation;

    tf2::Quaternion quaternion;
    tf2::fromMsg(transform.rotation,quaternion);
    tf2::Matrix3x3 rotationMatrix = tf2::Matrix3x3(quaternion);

    for(int i = 0; i < 4; i++)
    {
        initialPose.position.x = transform.translation.x + (2*(i%2) - 1)*0.2*rotationMatrix[0][i/2] - 0.05*rotationMatrix[0][2];
        initialPose.position.y = transform.translation.y + (2*(i%2) - 1)*0.2*rotationMatrix[1][i/2] - 0.05*rotationMatrix[1][2];
        initialPose.position.z = transform.translation.z + (2*(i%2) - 1)*0.2*rotationMatrix[2][i/2] - 0.05*rotationMatrix[2][2];

        ROS_INFO("Calibration point %d : \n X : %f \n Y : %f \n Z : %f",i+1,initialPose.position.x,initialPose.position.y,initialPose.position.z);
    
        robot.goToTarget(initialPose);

        double initNormalEffort = 0.0;
        for(int i = 0; i < 10; i++)
        {
            initNormalEffort += ((ros::topic::waitForMessage<geometry_msgs::WrenchStamped>("/wrench", ros::Duration(1.0))->wrench).force.z)/10;
        }

        double currentNormalEffort = initNormalEffort;
        currentPose = initialPose;
        
        while(abs(currentNormalEffort - initNormalEffort) < 1.0)
        {
            currentPose.position.x += 0.002*rotationMatrix[0][2];
            currentPose.position.y += 0.002*rotationMatrix[1][2];
            currentPose.position.z += 0.002*rotationMatrix[2][2];

            robot.goToTarget(currentPose,false,true);

            currentNormalEffort = 0.0;
            for(int i = 0; i < 10; i++)
            {
                currentNormalEffort += ((ros::topic::waitForMessage<geometry_msgs::WrenchStamped>("/wrench", ros::Duration(1.0))->wrench).force.z)/10;
            }
        }

        robot.goToTarget(initialPose,false,true);
        calibrationPoses.push_back(currentPose);
    }

    //Save reference position
    /*std::string yamlFile = ros::package::getPath("panda_draws_you")+"/config/PlaneCalibration.yaml";

    YAML::Node config = YAML::LoadFile(yamlFile);
    std::ofstream fout(yamlFile);

    if (config["poseReference"]) 
    {
        config.remove("poseReference");
        config["poseReference"].push_back(transform.translation.x);
        config["poseReference"].push_back(transform.translation.y);
        config["poseReference"].push_back(transform.translation.z);
        config["poseReference"].push_back(roll);
        config["poseReference"].push_back(pitch);
        config["poseReference"].push_back(yaw);
    }

    fout << config;*/

    ros::waitForShutdown();
    return 0;
}


