#pragma once

#define PATCH_HALF_SIZE 2
#define X_SIZE 0.3
#define Y_SIZE 0.2

#define X_OFFSET 0.35
#define Y_OFFSET -0.1

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

void sketchToWaypoints(std::string sketchFileName, std::vector<geometry_msgs::Pose> &waypoints, double height, geometry_msgs::Quaternion orientation)
{
    Mat output = imread(sketchFileName,IMREAD_GRAYSCALE);

    //imshow("Output",output);
    //waitKey();

    //Pre-processing : gaussian blur and bilateral filter
    GaussianBlur(output,output,Size(3,3),0.0);

    Mat dst = output.clone();
    bilateralFilter(output,dst,9,200,200);
    output = dst.clone();

    //imshow("Output",output);
    //waitKey();
    
    //Apply Canny Deriche filter

    //Filter parameters
    int lowThreshold=0;
    int maxThreshold=15;
    int const max_lowThreshold = 500;
    int alDerive=100;
    int alMean=100;

    //Compute filter
    double d=alDerive/100.0,m=alMean/100.0;

    UMat img;
    output.copyTo(img);

    UMat rx=  GradientDericheX(img,d,m);
    UMat ry=  GradientDericheY(img,d,m);

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

    CannyBis(img, output, lowThreshold, maxThreshold, 5, true, sobel_x,sobel_y);

    //imshow("Output",output);
    //waitKey();

    //Removing detected edges on the picture borders
    output.row(0) = Scalar(0,0,0);
    output.row(output.rows - 1) = Scalar(0,0,0);
    output.col(0) = Scalar(0,0,0);
    output.col(output.cols - 1) = Scalar(0,0,0);

    //Thresholding and inversion
    threshold(output,output,127,255,0);
    bitwise_not(output,output);

    //Crop image to fit the sketch
    Mat rowSum, columnSum;
    reduce(output,columnSum,1,CV_REDUCE_SUM,CV_32S);
    reduce(output,rowSum,0,CV_REDUCE_SUM,CV_32S);

    int minCol = output.cols, minRow = output.rows, maxCol = 0, maxRow = 0;

    for(int i = 0; i < output.rows; i++)
    {
        if(columnSum.at<int>(i) != 255*output.cols && minRow == output.rows)
        {
            minRow = i;
        }

        if(columnSum.at<int>(output.rows - 1 - i) != 255*output.cols && maxRow == 0)
        {
            maxRow = output.rows - i - 1;
        }

        if(minRow != output.rows && maxRow != 0)
        {
            break;
        }
    }

    for(int i = 0; i < output.cols; i++)
    {
        if(rowSum.at<int>(i) != 255*output.rows && minCol == output.cols)
        {
            minCol = i;
        }

        if(rowSum.at<int>(output.cols - 1 - i) != 255*output.rows && maxCol == 0)
        {
            maxCol = output.cols - i - 1;
        }

        if(minCol != output.cols && maxCol != 0)
        {
            break;
        }
    }

    Rect crop(minCol, minRow, maxCol-minCol, maxRow-minRow);
    output = output(crop);  

    //imshow("Output",output);
    //waitKey(); 

    //Convert pixels into waypoints => OPTIMISATION
    std::vector<double> X,Y,rowDistances;
    std::vector<std::vector<double>> distances;
    double distance;

    for(int i = 0; i < output.rows; i++)
    {
        for(int j = 0; j < output.cols; j++)
        {
            if(output.at<uchar>(i,j) == 0)
            {
                X.push_back(output.rows - i);
                Y.push_back(j);

                rowDistances.clear();

                for(int k = 0; k < X.size() - 1; k++)
                {
                    distance = sqrt((X[k]-X.back())*(X[k]-X.back()) + (Y[k]-Y.back())*(Y[k]-Y.back()));
                    rowDistances.push_back(distance);
                    distances[k].push_back(distance);
                }

                rowDistances.push_back(0.0);
                distances.push_back(rowDistances);
            }
        }
    }

    //imshow("Output",output);
    //waitKey();

    //Re-organise waypoints order to reduce the overall pencil trajectory => OPTIMISATION
    geometry_msgs::Pose currentPose, intermediatePose;
    currentPose.position.z = height;
    currentPose.orientation = orientation;

    std::vector<int> visitedIndex;
    std::vector<size_t> idx(X.size());
    int currentIndex = X.size()-1;

    double ratio = min(X_SIZE/(maxRow-minRow),Y_SIZE/(maxCol-minCol));

    while(visitedIndex.size() < X.size())
    {
        currentPose.position.x = X_OFFSET + ratio*X[currentIndex];
        currentPose.position.y = Y_OFFSET + ratio*Y[currentIndex];

        //Skip waypoint if too close from the previous one
        if(visitedIndex.empty() || sqrt((waypoints.back().position.x-currentPose.position.x)*(waypoints.back().position.x-currentPose.position.x) + (waypoints.back().position.y-currentPose.position.y)*(waypoints.back().position.y-currentPose.position.y)) > 0.003)
        {
            if(!waypoints.empty() && sqrt((waypoints.back().position.x-currentPose.position.x)*(waypoints.back().position.x-currentPose.position.x) + (waypoints.back().position.y-currentPose.position.y)*(waypoints.back().position.y-currentPose.position.y)) > 0.02)
            {
                intermediatePose = waypoints.back();
                intermediatePose.position.z += 0.02;
                waypoints.push_back(intermediatePose);
                intermediatePose = currentPose;
                intermediatePose.position.z += 0.02;
                waypoints.push_back(intermediatePose);
            }

            waypoints.push_back(currentPose);
        }

        visitedIndex.push_back(currentIndex);

        //Find the next closest waypoint... 
        iota(idx.begin(), idx.end(), 0);
        std::vector<double> localDistances = distances[currentIndex];
        stable_sort(idx.begin(), idx.end(),
            [&localDistances](size_t i1, size_t i2) {return localDistances[i1] < localDistances[i2];});

        //... Which has not been visited yet
        currentIndex = (int)idx[0];
        int counter = 0;
        while(std::find(visitedIndex.begin(), visitedIndex.end(), currentIndex) != visitedIndex.end())
        {
            counter++;
            currentIndex = (int)idx[counter];
        }
    }

    //imshow("Output",output);
    //waitKey();

    /*
    std::ofstream myfile("/tmp/Positions.csv");

    for(int i = 0; i < waypoints.size(); i++)
    {
        myfile << waypoints[i].position.x;
        myfile << ";";
        myfile << waypoints[i].position.y;
        myfile << "\n";
    }
    myfile.close();
    */

    /*
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
    */
}
