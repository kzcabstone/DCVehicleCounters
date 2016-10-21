//
//  Resizer.cpp
//  DCVehicleCounter
//
//  Created by hohe on 7/16/14.
//  Copyright (c) 2014 DongCheng. All rights reserved.
//

#include "Resizer.h"

bool Resizer::resize(cv::InputArray frame, cv::OutputArray resized)
{
    if (frame.empty())
    {
        ERR(std::cout << "Passed in frame empty" << std::endl);
        return false;
    }
    cv::Mat im = frame.getMat();
    cv::Size new_size = cv::Size(frame.size().width * resize_factor, frame.size().height * resize_factor);
    cv::resize(frame, resized, new_size);
    return true;
}