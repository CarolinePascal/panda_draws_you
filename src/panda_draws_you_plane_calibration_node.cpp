#include <robot_arm_tools/Robot.h>
#include <robot_arm_tools/RobotVisualTools.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <yaml-cpp/yaml.h>
#include <fstream>

#include <Eigen/QR>

#include <geometry_msgs/TwistStamped.h>

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
    //Move the robot to its initial configuration
    //robot.init();

    //Robot visual tools initialisation
    RobotVisualTools visualTools;
    
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
    ros::WallDuration(1.0).sleep();

    geometry_msgs::Pose centralPose, calibrationPose;
    centralPose.orientation = transform.rotation;
    centralPose.position.x = transform.translation.x;
    centralPose.position.y = transform.translation.y;
    centralPose.position.z = transform.translation.z;
    calibrationPose = centralPose;

    tf2::Quaternion quaternion;
    tf2::fromMsg(transform.rotation,quaternion);
    tf2::Matrix3x3 rotationMatrix = tf2::Matrix3x3(quaternion);

    Eigen::MatrixXd A(4,3);

    ros::Publisher twistCommandPublisher = n.advertise<geometry_msgs::TwistStamped>("/servo_server/delta_twist_cmds", 10);
    geometry_msgs::TwistStamped twistCommand;

    double initNormalEffort, currentNormalEffort;

    for(int i = 0; i < 4; i++)
    {
        calibrationPose.position.x = centralPose.position.x - (2*(i%2) - 1)*0.2*rotationMatrix[0][i/2];
        calibrationPose.position.y = centralPose.position.y - (2*(i%2) - 1)*0.2*rotationMatrix[1][i/2];
        calibrationPose.position.z = centralPose.position.z - (2*(i%2) - 1)*0.2*rotationMatrix[2][i/2];

        ROS_INFO("Calibration point %d : \n X : %f \n Y : %f \n Z : %f",i+1,calibrationPose.position.x,calibrationPose.position.y,calibrationPose.position.z);
    
        robot.goToTarget(calibrationPose,false);

        initNormalEffort = (ros::topic::waitForMessage<geometry_msgs::WrenchStamped>("/wrench", ros::Duration(1.0))->wrench).force.z;
        currentNormalEffort = initNormalEffort;

        //Switch to position controller instead of trajectory controller
        robot.switchController({"joint_group_pos_controller"},{"scaled_pos_joint_traj_controller"});
        ros::WallDuration(1.0).sleep();

        twistCommand.header.stamp = ros::Time::now();
        twistCommand.twist.linear.x = 0.075*rotationMatrix[0][2];
        twistCommand.twist.linear.y = 0.075*rotationMatrix[1][2];
        twistCommand.twist.linear.z = 0.075*rotationMatrix[2][2];
        twistCommandPublisher.publish(twistCommand);
    
        while(abs(currentNormalEffort - initNormalEffort) < 2.0)
        {
            twistCommand.header.stamp = ros::Time::now();
            twistCommandPublisher.publish(twistCommand);
            currentNormalEffort = (ros::topic::waitForMessage<geometry_msgs::WrenchStamped>("/wrench", ros::Duration(1.0))->wrench).force.z;
        }

        twistCommand.header.stamp = ros::Time::now();
        twistCommand.twist.linear.x = 0.0;
        twistCommand.twist.linear.y = 0.0;
        twistCommand.twist.linear.z = 0.0;
        twistCommandPublisher.publish(twistCommand);

        try
        {
            transform = tfBuffer.lookupTransform("world", endEffectorName, ros::Time(0), ros::Duration(5.0)).transform;
        } 
        catch (tf2::TransformException &ex) 
        {
            throw std::runtime_error("CANNOT RETRIVE SEEKED TRANSFORM !");
        }

        //Switch to position controller instead of trajectory controller
        robot.switchController({"scaled_pos_joint_traj_controller"},{"joint_group_pos_controller"});
        ros::WallDuration(1.0).sleep();

        robot.goToTarget(calibrationPose,false);
        A.row(i) << transform.translation.x,transform.translation.y,transform.translation.z;
    }

    robot.goToTarget(centralPose,false);

    Eigen::MatrixXd pinvA = A.completeOrthogonalDecomposition().pseudoInverse();

    Eigen::Vector4d v(-1.0,-1.0,-1.0,-1.0);
    Eigen::Vector4d planeEquation;
    planeEquation  << pinvA*v, 1;

    planeEquation /= sqrt(planeEquation.head(3).squaredNorm());
    Eigen::Vector3d planeNormal = planeEquation.head(3);

    Eigen::Vector3d planePoint(0.0,0.0,0.0);
    int argMaxPlaneNormal = planeNormal.array().abs().maxCoeff();
    planePoint(argMaxPlaneNormal) = -planeEquation(3)/planeNormal(argMaxPlaneNormal);

    Eigen::Vector3d planeCenter(transform.translation.x,transform.translation.y,transform.translation.z);
    
    planeCenter = planeCenter - ((planeCenter - planePoint).transpose()*planeNormal)*planeNormal;

    Eigen::Vector3d xAxis = Eigen::Vector3d(planeEquation(1),-planeEquation(0),0);
    xAxis /= sqrt(xAxis.squaredNorm());

    Eigen::Vector3d yAxis = planeNormal.cross(xAxis);

    //Save plane equation
    std::string yamlFile = ros::package::getPath("panda_draws_you")+"/config/PlaneCalibration.yaml";
    YAML::Node config;

    try
    {
        config = YAML::LoadFile(yamlFile);
    }
    catch(const std::exception& e)
    {
        std::ofstream {yamlFile};
        config = YAML::LoadFile(yamlFile);
    } 

    if (config["xAxis"]) 
    {
        config.remove("xAxis");
    }
    config["xAxis"].push_back(xAxis(0));
    config["xAxis"].push_back(xAxis(1));
    config["xAxis"].push_back(xAxis(2));  

    if (config["yAxis"]) 
    {
        config.remove("yAxis");
    }
    config["yAxis"].push_back(yAxis(0));
    config["yAxis"].push_back(yAxis(1));
    config["yAxis"].push_back(yAxis(2));

    if (config["zAxis"]) 
    {
        config.remove("zAxis");
    }
    config["zAxis"].push_back(planeNormal(0));
    config["zAxis"].push_back(planeNormal(1));
    config["zAxis"].push_back(planeNormal(2));

    if (config["centerPoint"]) 
    {
        config.remove("centerPoint");
    }
    config["centerPoint"].push_back(planeCenter(0));
    config["centerPoint"].push_back(planeCenter(1));
    config["centerPoint"].push_back(planeCenter(2));

    std::ofstream fout(yamlFile); 
    fout << config;

    ros::waitForShutdown();
    return 0;
}


