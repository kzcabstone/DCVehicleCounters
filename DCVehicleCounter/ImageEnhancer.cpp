//
//  ImageEnhancer.cpp
//  DCVehicleCounter
//
//  Created by hohe on 7/14/14.
//  Copyright (c) 2014 DongCheng. All rights reserved.
//

#include "ImageEnhancer.h"

ImageEnhancer::ImageEnhancer()
{
    min_luma = max_luma = avg_luma = 0.0;
}

ImageEnhancer::~ImageEnhancer()
{
    
}

// Expecting frame to be 3 channel RGB format
// Output: single channel grayscale of enhanced image
bool ImageEnhancer::enhance(cv::InputArray frame, uint64 fn, cv::OutputArray enhanced_gray, cv::OutputArray enhanced_color)
{
    CV_Assert(frame.channels() == 3);
    
    cv::Mat im = frame.getMat();
    enhanced_gray.create(frame.size(), CV_8UC1); // TODO: need 16bit?
    cv::Mat egray = enhanced_gray.getMat();
    cv::Mat ec;
    bool ec_needed = enhanced_color.needed();
    
    // Convert RGB to Grayscale
    //cv::cvtColor(frame, egray, CV_BGR2GRAY);
    std::vector<cv::Mat> channels;
    cv::Mat f1;
    cvtColor(frame, f1, CV_BGR2YCrCb);
    split(f1, channels);
    
    channels[0].copyTo(enhanced_gray);
    
    // Adaptive equalize
    //equalize(egray);
    
    if (ec_needed)
    {
        cv::Mat channels[] = {egray, egray, egray};
        cv::merge(channels, 3, enhanced_color);
    }
    return true;
}

void ImageEnhancer::equalize(cv::InputOutputArray frame)
{
    CV_Assert(frame.channels() == 1);
    
    double min, max, avg;
    cv::minMaxLoc(frame, &min, &max);
    cv::Scalar m = cv::mean(frame);
    avg = m[0];
    if (min_luma == 0)
    {
        min_luma = min;
    }
    else
    {
        min_luma = min_luma * (1 - CONTRIBUTION_RATIO) + min * CONTRIBUTION_RATIO;
    }
    
    if (max_luma == 0)
    {
        max_luma = max;
    }
    else
    {
        max_luma = max_luma * (1 - CONTRIBUTION_RATIO) + max * CONTRIBUTION_RATIO;
    }
    
    if (avg_luma == 0)
    {
        avg_luma = avg;
    }
    else
    {
        avg_luma = avg_luma * (1 - CONTRIBUTION_RATIO) + avg * CONTRIBUTION_RATIO;
    }
    
    cv::Size s = frame.size();
    cv::Mat im = frame.getMat();
    
    // The goal is to move avg to 128, then scale the upper half and lower half accordingly
    double ratio1 = (avg_luma - min_luma) / 128;
    double ratio2 = (max_luma - avg_luma) / 128;
    for (int i = 0; i < s.width; i++)
    {
        for (int j = 0; j < s.height; j++)
        {
            uchar val = im.at<uchar>(cv::Point(i, j));
            double t;
            if (val < avg_luma)
            {
                t = ratio1 * val + 0;
            }
            else
            {
                t = ratio2 * val + avg_luma;
            }
            im.at<uchar>(cv::Point(i, j)) = (t > 255) ? 255 : (uchar)t;
        }
    }
}

void ImageEnhancer::haarReduceNoise(cv::InputOutputArray frame)
{
    
}
