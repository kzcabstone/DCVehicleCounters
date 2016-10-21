//
//  EdgeDetector.cpp
//  DCVehicleCounter
//
//  Created by hohe on 7/31/14.
//  Copyright (c) 2014 DongCheng. All rights reserved.
//

#include "EdgeDetector.h"

double EdgeDetector::setLowThreshold(double d)
{
    low_threshold = d;
    return low_threshold;
}

double EdgeDetector::setHighThreshold(double d)
{
    high_threshold = d;
    return high_threshold;
}

int EdgeDetector::setApertureSize(int i)
{
    apertureSize = i;
    return apertureSize;
}

bool EdgeDetector::setL2gradientFlag(bool f)
{
    L2gradient = f;
    return L2gradient;
}
static bool wayToSort(cv::Vec2i &i, cv::Vec2i &j) { return i[1] > j[1]; }

void EdgeDetector::countContourChildren(std::vector<cv::Vec4i> &h, std::vector<cv::Vec2i> &childrens)
{
	/*
    int i;
    childrens.resize(h.size());
    for (i = 0; i < h.size(); i++)
    {
        childrens[i] = cv::Vec2i(i, 0);
    }
    
    for (i = 0; i < h.size(); i++)
    {
        cv::Vec4i v = h[i];
        do
        {
            int parent = v[3];
            if (parent == -1)
            {
                break;
            }
            v = h[parent];
            childrens[parent][1]++;
        } while (1);
    }
    
    // Sort
    std::sort(childrens.begin(), childrens.end(), wayToSort);
    
#if 0
    for (i = 0; i < childrens.size(); i++)
    {
        std::cout << "Contour " << childrens[i][0] << " -- " << childrens[i][1] << std::endl;
    }
#endif
*/
}

static cv::Scalar colors[] = {
    cv::Scalar(255, 0, 0),
    cv::Scalar(160, 0, 0),
    cv::Scalar(80, 0, 0),
    cv::Scalar(0, 255, 0),
    cv::Scalar(0, 160, 0),
    cv::Scalar(0, 80, 0),
    cv::Scalar(0, 0, 255),
    cv::Scalar(0, 0, 160),
    cv::Scalar(0, 0, 80),
    cv::Scalar(255, 255),
    cv::Scalar(160, 160, 0),
    cv::Scalar(80, 80, 0),
    cv::Scalar(0, 255, 255),
    cv::Scalar(0, 160, 160),
    cv::Scalar(0, 80, 80)
};

static double max_xy_ratio = 0;
bool EdgeDetector::isVerticalLine(std::vector<cv::Point> *pContour)
{
    if (pContour == nullptr)
    {
        return false;
    }
    
    int xmin, xmax, ymin, ymax;
    xmin = ymin = std::numeric_limits<int>::max();
    xmax = ymax = 0;
    
    for (int i = 0; i < pContour->size(); i++)
    {
        int x, y;
        x = pContour->at(i).x;
        y = pContour->at(i).y;
        if (x < xmin)
        {
            xmin = x;
        }
        if (x > xmax)
        {
            xmax = x;
        }
        if (y < ymin)
        {
            ymin = y;
        }
        if (y > ymax)
        {
            ymax = y;
        }
    }
    
    double xoffset, yoffset;
    xoffset = xmax - xmin;
    yoffset = ymax - ymin;
    if (xoffset == 0)
    {
        if ((yoffset > 0))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    
    
    double ratio = yoffset/xoffset;
    if (ratio > max_xy_ratio)
    {
        max_xy_ratio = ratio;
        DBOUT(std::cout << "Max Ratio: " << max_xy_ratio << std::endl);
    }
    if (ratio > 5)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool EdgeDetector::process(cv::InputArray frame, cv::OutputArray edgeMask, uint64 fn, int thickness)
{
    cv::Mat canny_output;
    cv::Mat temp = frame.getMat();
    //cv::GaussianBlur(frame, temp, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT );
    cv::Canny(temp, canny_output, low_threshold, high_threshold, apertureSize, L2gradient);
    DBSHOW("filtered", temp);
    DBSHOW("canny", edgeMask);
    
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<cv::Vec2i> children;
    
    /// Find contours
    findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    //countContourChildren(hierarchy, children);
    
    /// Draw contours
    cv::Mat drawing = cv::Mat::zeros( canny_output.size(), CV_8UC1);// edgeMask.type());
    for( int i = 0; i < contours.size(); i++ )
    {
        if (!isVerticalLine(&(contours[i])))
        {
            drawContours(drawing, contours, i, cv::Scalar(255),
                         thickness, //CV_FILLED,
                         8, hierarchy, 0, cv::Point());
        }
        /*
        else
        {
            drawContours(drawing, contours, i, cv::Scalar(127),
                         2, //CV_FILLED,
                         8, hierarchy, 0, cv::Point());
        }
         */
    }
    /*
    {
        cv::Canny(drawing, canny_output, low_threshold, high_threshold, apertureSize, L2gradient);
        /// Find contours
        findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
        countContourChildren(hierarchy, children);
        
        for( int i = 0; i < contours.size(); i++ )
        {
            if (isContourConvex(contours[i]))
            {
            drawContours(drawing, contours, i, cv::Scalar(255),
                         CV_FILLED,
                         8, hierarchy, 0, cv::Point());
            }
            else
            {
                drawContours(drawing, contours, i, cv::Scalar(150),
                             2,//CV_FILLED,
                             8, hierarchy, 0, cv::Point());
            }
        }
    }
*/
    /// Show in a window
    DBSHOW("Contours", drawing);
    drawing.copyTo(edgeMask);

    return true;
}
