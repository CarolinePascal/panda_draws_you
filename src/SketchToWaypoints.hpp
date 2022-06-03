#pragma once

#define PATCH_HALF_SIZE 2
#define X_SIZE 0.20
#define Y_SIZE 0.30

#define PIXEL_DISTANCE 0.0015
#define JUMP_DISTANCE 0.005

#define ELLIPSE_X_RATIO 0.3
#define ELLIPSE_Y_RATIO 0.7
#define CROP_RATIO 0.0

#include <vector>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <math.h> 

#include <iostream>
#include <algorithm>
#include <numeric>

#include <ctime> 

#include "CannyDeriche.hpp"

#include "panda_draws_you/matplotlibcpp.h"
namespace plt = matplotlibcpp;

void CannyDericheFilter(int, void* pointer)
{
    CANNY_DERICHE_PARAMETERS* parameters = (CANNY_DERICHE_PARAMETERS*)pointer;

    //Pre-processing : gaussian blur
    GaussianBlur(parameters->input,parameters->output,Size(2*parameters->gaussianBlur+1,2*parameters->gaussianBlur+1),0.0);

    //Pre-processing : edges-preserving filter
    edgePreservingFilter(parameters->output,parameters->output,1,(float)parameters->sigma_s,(float)parameters->sigma_r/100.0);
    //stylization(parameters->output,parameters->output,(float)parameters->sigma_s,(float)parameters->sigma_r/100.0);
    
    cvtColor(parameters->output, parameters->output, COLOR_BGR2GRAY);
    equalizeHist(parameters->output,parameters->output);

    //Computing Canny-Deriche filter
    double d=parameters->alDerive/100.0,m=parameters->alMean/100.0;

    UMat img;
    parameters->output.copyTo(img);

    UMat rx=GradientDericheX(img,d,m);
    UMat ry=GradientDericheY(img,d,m);

    double minv,maxv;
    minMaxLoc(rx,&minv,&maxv);
    minMaxLoc(ry,&minv,&maxv);

    Mat mm;
    Mat sobel_x, sobel_y;
    mm=abs(rx.getMat(ACCESS_READ));
    rx.getMat(ACCESS_READ).convertTo(sobel_x,CV_16S,1);
    mm=abs(ry.getMat(ACCESS_READ));
    ry.getMat(ACCESS_READ).convertTo(sobel_y,CV_16S,1);

    minMaxLoc(sobel_x,&minv,&maxv);
    minMaxLoc(sobel_y,&minv,&maxv);

    CannyBis(img, parameters->output, parameters->lowThreshold, parameters->maxThreshold, 5, true, sobel_x,sobel_y);

    /*
    int morph_size = 0;
    Mat element = getStructuringElement(
        MORPH_RECT,
        Size(2 * morph_size + 1,
             2 * morph_size + 1),
        Point(morph_size,
              morph_size));
    morphologyEx(parameters->output,parameters->output,MORPH_CLOSE,element,Point(-1,-1),2);
    */

    //Updating display
    imshow("CannyDericheFilter", parameters->output);
}

void sketchToWaypoints(std::vector<geometry_msgs::Pose> &waypoints)
{
    VideoCapture capture(0);
    if (!capture.isOpened()) 
    {
        ROS_ERROR("Cannot open video capture device !");
        throw std::runtime_error("Cannot open video capture device !");   
    }

    capture.set(CV_CAP_PROP_FRAME_WIDTH,1920);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT,1080);

    // Prepare an image where to store the video frames
    Mat image, display_image;

    // Main loop
    while (capture.read(image) && !image.empty()) 
    {
        // Show the captured image and the detected features
        display_image = image.clone();
        ellipse(display_image, Point(image.cols/2,image.rows/2), Size(image.cols*ELLIPSE_X_RATIO/2, image.rows*ELLIPSE_Y_RATIO/2), 0, 0, 360, Scalar(0, 0, 255), 2, LINE_AA);
        imshow("FaceDetection", display_image);

        int cropX = (int)floor(floor((image.cols*ELLIPSE_X_RATIO + image.cols*CROP_RATIO)/X_SIZE)*X_SIZE);
        int cropY = (int)floor(floor((image.rows*ELLIPSE_Y_RATIO + image.rows*CROP_RATIO)/Y_SIZE)*Y_SIZE);
        
        Rect crop(image.cols/2 - cropX/2, image.rows/2 - cropY/2, cropX, cropY);

        // Wait for input or process the next frame      
        if((int)waitKey(10) == 10)
        {
            std::time_t t = std::time(0);
            std::tm* now = std::localtime(&t);
            image = image(crop);
            imwrite(ros::package::getPath("panda_draws_you") + "/config/ScienceDay/Picture_"+std::to_string(now->tm_mday)+"_"+std::to_string(now->tm_hour)+"_"+std::to_string(now->tm_min)+".png", image);
            break;
        }
    }

    capture.release();
    destroyAllWindows();

    Mat output = image.clone();

    //[DEBUG]
    output = imread(ros::package::getPath("panda_draws_you") + "/config/Picture1.png",IMREAD_COLOR);

    //[DISPLAY]
    //imshow("Output",output);
    //waitKey();

    //Edges detection : Canny-Deriche filter

    //Setting initial filter parameters
    CANNY_DERICHE_PARAMETERS parameters;

    parameters.lowThreshold=0;
    parameters.maxThreshold=20;
    parameters.alDerive=100;
    parameters.alMean=50;

    parameters.gaussianBlur=1;

    parameters.sigma_s = 60;
    parameters.sigma_r = 45;

    parameters.input = output.clone();
    parameters.output = output.clone();

    //Displaying first output
    CannyDericheFilter(0,&parameters);
           
    //Tweaking parameters...
    Mat dummy = Mat::zeros(Size(1000,1),CV_8UC1);
    imshow("Parameters",dummy);

    createTrackbar("Gaussian Blur:","Parameters", &parameters.gaussianBlur, 10, CannyDericheFilter, (void*)&parameters);

    createTrackbar("Sigma S:","Parameters", &parameters.sigma_s, 200, CannyDericheFilter, (void*)&parameters);
    createTrackbar("Sigma R:","Parameters", &parameters.sigma_r, 100, CannyDericheFilter, (void*)&parameters);

    createTrackbar("Min Threshold:","Parameters", &parameters.lowThreshold, 500, CannyDericheFilter, (void*)&parameters);
    createTrackbar("Max Threshold:", "Parameters", &parameters.maxThreshold, 500, CannyDericheFilter, (void*)&parameters);
    createTrackbar("Derive:","Parameters", &parameters.alDerive, 400, CannyDericheFilter, (void*)&parameters);
    createTrackbar("Mean:", "Parameters", &parameters.alMean, 400, CannyDericheFilter, (void*)&parameters);

    waitKey(); 
    destroyAllWindows();

    output = parameters.output.clone();

    //[DISPLAY]
    //imshow("Output",output);
    //waitKey();

    //Removing detected edges on the picture borders
    output.row(0) = Scalar(0);
    output.row(output.rows - 1) = Scalar(0);
    output.col(0) = Scalar(0);
    output.col(output.cols - 1) = Scalar(0);

    //Thresholding
    threshold(output,output,127,255,0);

    //Cropping image to fit the input sketch
    Mat rowSum, columnSum;
    double minVal, maxVal;

    reduce(output,columnSum,1,CV_REDUCE_SUM,CV_32S);
    minMaxLoc(columnSum, &minVal, &maxVal);
    columnSum.convertTo(columnSum,CV_8UC1,255.0/(maxVal-minVal),-255.0*minVal/(maxVal-minVal));

    reduce(output,rowSum,0,CV_REDUCE_SUM,CV_32S);
    minMaxLoc(rowSum, &minVal, &maxVal);
    rowSum.convertTo(rowSum,CV_8UC1,255.0/(maxVal-minVal),-255.0*minVal/(maxVal-minVal));

    std::vector<Point> rowId, columnId;
    findNonZero(columnSum,columnId);
    findNonZero(rowSum,rowId);

    int minCol = max(0, rowId.front().x - 1), maxCol = min(output.cols - 1, rowId.back().x + 1);
    int minRow = max(0, columnId.front().y - 1), maxRow = min(output.rows - 1, columnId.back().y + 1);

    Rect crop(minCol, minRow, maxCol-minCol, maxRow-minRow);
    output = output(crop); 

    double ratio = min(X_SIZE/(maxCol-minCol),Y_SIZE/(maxRow-minRow));

    //[DISPLAY]
    //imshow("Output",output);
    //waitKey();

    /*
    //Finding breakpoints in edge lines using hit-or-miss morphological transform
    Mat kernel1 = (Mat_<int>(3, 3) <<
        0, 0, 0,
        1, -1, 1,
        1, 1, 1);

    Mat kernel2 = (Mat_<int>(3, 3) <<
        0, 1, 1,
        0, -1, 1,
        0, 1, 1);

    Mat kernel3 = (Mat_<int>(3, 3) <<
        1, 1, 1,
        1, -1, 1,
        0, 0, 0);

    Mat kernel4 = (Mat_<int>(3, 3) <<
        1, 1, 0,
        1, -1, 0,
        1, 1, 0);

    Mat output_image, output_image1, output_image2, output_image3, output_image4;

    //Mat element = getStructuringElement(MORPH_RECT, Size(3,3), Point(1,1));
    //erode(output, output, element);
    //erode(output, output, element);
    //dilate(output, output, element);
    //dilate(output, output, element);

    morphologyEx(output, output_image1, MORPH_HITMISS, kernel1);
    morphologyEx(output, output_image2, MORPH_HITMISS, kernel2);
    morphologyEx(output, output_image3, MORPH_HITMISS, kernel3);
    morphologyEx(output, output_image4, MORPH_HITMISS, kernel4);

    bitwise_or(output_image1,output_image2,output_image);
    bitwise_or(output_image,output_image3,output_image);
    bitwise_or(output_image,output_image4,output_image);

    imshow("Breakpoints",output_image);
    imshow("Original",output);
    waitKey();
    */

    //Sampling pixels at a given distance from each other
    int size = (int)(PIXEL_DISTANCE/ratio);
    Mat patch(2*size+1, 2*size+1, CV_8UC1, Scalar(0)); 
    patch.at<uchar>(size, size) = 255;

    copyMakeBorder(output, output, size, size, size, size, BORDER_CONSTANT, Scalar(0));

    for(int i = size; i < output.rows - size; i++)
    {
        for(int j = size; j < output.cols - size; j++)
        {
            if(output.at<uchar>(i,j) == 255)
            {
                patch.copyTo(output(Rect(j-size, i-size, 2*size+1, 2*size+1)));
            } 
        }
    }

    //[DISPLAY]
    imshow("Output",output);
    waitKey(); 
    destroyAllWindows();
    
    //Converting pixels into waypoints
    std::vector<double> X,Y,localDistances;
    std::vector<std::vector<double>> distances;
    double distance;

    std::vector<Point> pixelId;
    findNonZero(output,pixelId);

    for(int i = 0; i < pixelId.size(); i++)
    {
        X.push_back(ratio*pixelId[i].x);
        Y.push_back(ratio*(output.rows - pixelId[i].y));

        localDistances.clear();

        for(int k = 0; k < X.size() - 1; k++)
        {
            distance = (X[k]-X.back())*(X[k]-X.back()) + (Y[k]-Y.back())*(Y[k]-Y.back());
            localDistances.push_back(distance);
            distances[k].push_back(distance);
        }

        localDistances.push_back(0.0);
        distances.push_back(localDistances);
    }

    //Re-organising the waypoints order to reduce the overall pencil trajectory => OPTIMISATION
    geometry_msgs::Pose currentPose, intermediatePose;

    //Setting up waypoints height and orientation
    currentPose.position.z = 0.0;
    tf2::Quaternion quaternion;
    quaternion.setRPY(M_PI,0,0);
    currentPose.orientation = tf2::toMsg(quaternion);

    currentPose.position.x = X.back();
    currentPose.position.y = Y.back();
    waypoints.push_back(currentPose);

    int currentIndex = X.size()-1;

    std::vector<size_t> indices(X.size());
    std::iota(indices.begin(), indices.end(), 0);

    int jumpCounter = 0;

    while(indices.size() > 1)
    {
        //Finding the next closest waypoint
        localDistances = distances[currentIndex];
        localDistances.erase(localDistances.begin() + currentIndex);

        std::vector<size_t> idx(localDistances.size());
        std::iota(idx.begin(), idx.end(), 0);
        
        std::stable_sort(idx.begin(), idx.end(),
            [&localDistances](size_t i1, size_t i2) {return localDistances[i1] < localDistances[i2];});

        //Deleting the current waypoint in the distance matrix and indices vector
        distances.erase(distances.begin() + currentIndex);
        for(auto& row:distances)
        {
            row.erase(row.begin()+currentIndex);
        }

        indices.erase(indices.begin() + currentIndex);

        //Switching to the closest waypoint
        currentIndex = idx[0];
        currentPose.position.x = X[indices[currentIndex]];
        currentPose.position.y = Y[indices[currentIndex]];

        if(localDistances[currentIndex] > JUMP_DISTANCE*JUMP_DISTANCE)
        {
            intermediatePose = waypoints.back();
            intermediatePose.position.z += 0.02;
            waypoints.push_back(intermediatePose);

            intermediatePose = currentPose;
            intermediatePose.position.z += 0.02;
            waypoints.push_back(intermediatePose);

            jumpCounter++;
        }

        waypoints.push_back(currentPose);
    }

    ROS_INFO("Number of jumps : %i", jumpCounter);

    //imshow("Output",output);
    //waitKey();

    /*
    std::ofstream myfile("/tmp/Positions.csv");

    for(int i = 0; i < waypoints.size(); i++)
    {
        myfile << waypoints[i].position.x;
        myfile << ";";
        myfile << waypoints[i].position.y;
        myfile << ";";
        myfile << waypoints[i].position.z;
        myfile << "\n";
    }
    myfile.close();
    */

    std::map<std::string, std::string> keywords;
    keywords.insert(std::pair<std::string, std::string>("ms", "2") );
    keywords.insert(std::pair<std::string, std::string>("c", "blue") );
    keywords.insert(std::pair<std::string, std::string>("marker", "o") );
    keywords.insert(std::pair<std::string, std::string>("lw", "0") );

    for(int i = 0; i < waypoints.size(); i++)
    {
        plt::plot(std::vector<double> {waypoints[i].position.x},std::vector<double> {waypoints[i].position.y}, keywords);
    }

    plt::xlabel("y");
    plt::ylabel("x");
    plt::axis("equal");
    plt::show();   
}
