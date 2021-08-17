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
    int maxThreshold=10;
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
            minRow = max(0,i-1);
        }

        if(columnSum.at<int>(output.rows - 1 - i) != 255*output.cols && maxRow == 0)
        {
            maxRow = min(output.rows - i, output.rows - 1);
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
            minCol = max(0,i-1);
        }

        if(rowSum.at<int>(output.cols - 1 - i) != 255*output.rows && maxCol == 0)
        {
            maxCol = min(output.cols - i, output.rows - 1);
        }

        if(minCol != output.cols && maxCol != 0)
        {
            break;
        }
    }

    Rect crop(minCol, minRow, maxCol-minCol, maxRow-minRow);
    output = output(crop);  
    double ratio = min(X_SIZE/(maxRow-minRow),Y_SIZE/(maxCol-minCol));

    //imshow("Output",output);
    //waitKey(); 

    /*
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

    Mat element = getStructuringElement(MORPH_RECT,
                       Size(3,3),
                       Point(1,1));
    erode(output, output, element);
    erode(output, output, element);
    dilate(output, output, element);
    dilate(output, output, element);

    morphologyEx(output, output_image1, MORPH_HITMISS, kernel1);
    morphologyEx(output, output_image2, MORPH_HITMISS, kernel2);
    morphologyEx(output, output_image3, MORPH_HITMISS, kernel3);
    morphologyEx(output, output_image4, MORPH_HITMISS, kernel4);

    bitwise_or(output_image1,output_image2,output_image);
    bitwise_or(output_image,output_image3,output_image);
    bitwise_or(output_image,output_image4,output_image);
    */
    

    //Convert pixels into waypoints => OPTIMISATION
    std::vector<double> X,Y,localDistances;
    std::vector<std::vector<double>> distances;
    double distance;

    for(int i = 0; i < output.rows; i++)
    {
        for(int j = 0; j < output.cols; j++)
        {
            if(output.at<uchar>(i,j) == 0)
            {
                X.push_back(X_OFFSET + ratio*(output.rows - i));
                Y.push_back(Y_OFFSET + ratio*j);

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
        }
    }

    //Re-organise waypoints order to reduce the overall pencil trajectory => OPTIMISATION
    geometry_msgs::Pose currentPose, intermediatePose;
    currentPose.position.z = height;
    currentPose.orientation = orientation;

    currentPose.position.x = X.back();
    currentPose.position.y = Y.back();
    waypoints.push_back(currentPose);

    int currentIndex = X.size()-1;
    int newIndex;
    std::vector<int> visitedIndex {currentIndex};

    std::vector<size_t> indices(X.size());
    std::iota(indices.begin(), indices.end(), 0);

    while(visitedIndex.size() < X.size())
    {
        //Sort the distances to the current waypoint
        localDistances = distances[currentIndex];
        localDistances.erase(localDistances.begin() + currentIndex);

        std::vector<size_t> idx(localDistances.size());
        std::iota(idx.begin(), idx.end(), 0);
        
        std::stable_sort(idx.begin(), idx.end(),
            [&localDistances](size_t i1, size_t i2) {return localDistances[i1] < localDistances[i2];});

        //Delete the current waypoint in the distance matrix and indices vector
        distances.erase(distances.begin() + currentIndex);
        for(auto& row:distances)
        {
            row.erase(row.begin() + currentIndex);
        }

        indices.erase(indices.begin() + currentIndex);

        //Skip waypoints which are too close from the current waypoint
        newIndex = 0;   
        while(localDistances[newIndex] < 0.003*0.003)
        {
            visitedIndex.push_back(indices[newIndex]);
            newIndex++;
        }

        //Switch to the closest (but not too close) waypoint
        currentIndex = idx[newIndex];
        currentPose.position.x = X[indices[currentIndex]];
        currentPose.position.y = Y[indices[currentIndex]];
        visitedIndex.push_back(indices[currentIndex]);

        //Delete the too close waypoints
        for(int i = 0; i < newIndex; i++)
        {
            distances.erase(distances.begin() + idx[i]);
            for(auto& row:distances)
            {
                row.erase(row.begin() + idx[i]);
            }

            indices.erase(indices.begin() + idx[i]);   
        }

        if(localDistances[newIndex] > 0.01*0.01)
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
