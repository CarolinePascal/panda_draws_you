#pragma once

#define PATCH_HALF_SIZE 2
#define X_SIZE 0.3
#define Y_SIZE 0.2

#define X_OFFSET 0.35
#define Y_OFFSET -0.1

#define PIXEL_DISTANCE 0.002
#define JUMP_DISTANCE 0.01

#include <vector>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <math.h> 

#include <iostream>
#include <algorithm>
#include <numeric>

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
    cvtColor(parameters->output, parameters->output, COLOR_BGR2GRAY);

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

    //Updating display
    imshow("CannyDericheFilter", parameters->output);
}

void sketchToWaypoints(std::string sketchFileName, std::vector<geometry_msgs::Pose> &waypoints, double height, geometry_msgs::Quaternion orientation)
{
    //Loading the input sketch
    //Mat output = imread(sketchFileName,IMREAD_GRAYSCALE);
    Mat output = imread(sketchFileName,IMREAD_COLOR);

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

    namedWindow("TrackBar", WINDOW_AUTOSIZE);
    resizeWindow("TrackBar", 1000, 0);
    imshow("CannyDericheFilter", parameters.output);

    //Tweaking parameters...
    createTrackbar( "Gaussian Blur:","TrackBar", &parameters.gaussianBlur, 10, CannyDericheFilter, (void*)&parameters);

    createTrackbar( "Sigma S:","TrackBar", &parameters.sigma_s, 200, CannyDericheFilter, (void*)&parameters);
    createTrackbar( "Sigma R:","TrackBar", &parameters.sigma_r, 100, CannyDericheFilter, (void*)&parameters);

    createTrackbar( "Min Threshold:","TrackBar", &parameters.lowThreshold, 500, CannyDericheFilter, (void*)&parameters);
    createTrackbar( "Max Threshold:", "TrackBar", &parameters.maxThreshold, 500, CannyDericheFilter, (void*)&parameters);
    createTrackbar( "Derive:","TrackBar", &parameters.alDerive, 400, CannyDericheFilter, (void*)&parameters);
    createTrackbar( "Mean:", "TrackBar", &parameters.alMean, 400, CannyDericheFilter, (void*)&parameters);

    waitKey();   

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
    double ratio = min(X_SIZE/(maxRow-minRow),Y_SIZE/(maxCol-minCol));

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
    
    //Converting pixels into waypoints
    std::vector<double> X,Y,localDistances;
    std::vector<std::vector<double>> distances;
    double distance;

    std::vector<Point> pixelId;
    findNonZero(output,pixelId);

    for(int i = 0; i < pixelId.size(); i++)
    {
        X.push_back(X_OFFSET + ratio*(output.rows - pixelId[i].y));
        Y.push_back(Y_OFFSET + ratio*pixelId[i].x);

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
    currentPose.position.z = height;
    currentPose.orientation = orientation;

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
        plt::plot(std::vector<double> {waypoints[i].position.y},std::vector<double> {waypoints[i].position.x}, keywords);
    }

    plt::xlabel("y");
    plt::ylabel("x");
    plt::axis("equal");
    plt::show();
    
}
