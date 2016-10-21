//
//  ImageEnhancer.h
//  DCVehicleCounter
//
//  Created by hohe on 7/14/14.
//  Copyright (c) 2014 DongCheng. All rights reserved.
//

#ifndef __DCVehicleCounter__ImageEnhancer__
#define __DCVehicleCounter__ImageEnhancer__

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cvaux.hpp>

class ImageEnhancer
{
private:
    constexpr const static double CONTRIBUTION_RATIO = 0.1;
    double min_luma, max_luma, avg_luma;
    void haarReduceNoise(cv::InputOutputArray frame);
    void equalize(cv::InputOutputArray frame);
    
public:
    ImageEnhancer();
    ~ImageEnhancer();
    
    bool enhance(cv::InputArray frame, uint64 fn, cv::OutputArray enhanced_gray, cv::OutputArray enhanced_color = cv::noArray());
};
#endif /* defined(__DCVehicleCounter__ImageEnhancer__) */
