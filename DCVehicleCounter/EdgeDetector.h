//
//  EdgeDetector.h
//  DCVehicleCounter
//
//  Created by hohe on 7/31/14.
//  Copyright (c) 2014 DongCheng. All rights reserved.
//

#ifndef __DCVehicleCounter__EdgeDetector__
#define __DCVehicleCounter__EdgeDetector__

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cvaux.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "debug.h"

class EdgeDetector
{
protected:
    // The higher the threshold, the stronger an edge needs to be to get detected.
    // high_threshold should be roughly 3 x low_threshold
    double low_threshold;
    double high_threshold;
    int apertureSize;
    bool L2gradient;
    void countContourChildren(std::vector<cv::Vec4i> &h, std::vector<cv::Vec2i> &childrens);
    bool isVerticalLine(std::vector<cv::Point> *pContour);
    
public:
    EdgeDetector() : EdgeDetector(300, 900)
    {
    }
    EdgeDetector(double low, double high) : low_threshold(low), high_threshold(high), apertureSize(3), L2gradient(false)
    {
    }
    
    double setLowThreshold(double);
    double setHighThreshold(double);
    int setApertureSize(int);
    bool setL2gradientFlag(bool);
    bool process(cv::InputArray frame, cv::OutputArray edgeMask, uint64 fn, int thickness = 2);
};

#endif /* defined(__DCVehicleCounter__EdgeDetector__) */
