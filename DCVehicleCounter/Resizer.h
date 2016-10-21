//
//  Resizer.h
//  DCVehicleCounter
//
//  Created by hohe on 7/16/14.
//  Copyright (c) 2014 DongCheng. All rights reserved.
//

#ifndef __DCVehicleCounter__Resizer__
#define __DCVehicleCounter__Resizer__

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cvaux.hpp>
#include "ErrorCodes.h"
#include "debug.h"

class Resizer
{
private:
    double resize_factor;
    
public:
    Resizer(double d) : resize_factor(d)
    {
    }
    bool resize(cv::InputArray frame, cv::OutputArray resized);
    void setResizeFactor(double d)
    {
        resize_factor = d;
    }
    double getResizeFactor()
    {
        return resize_factor;
    }
    
};

#endif /* defined(__DCVehicleCounter__Resizer__) */
