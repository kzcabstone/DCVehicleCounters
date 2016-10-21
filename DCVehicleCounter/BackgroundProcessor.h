//
//  BackgroundProcessor.h
//  DCVehicleCounter
//
//  Created by hohe on 7/14/14.
//  Copyright (c) 2014 DongCheng. All rights reserved.
//

#ifndef __DCVehicleCounter__BackgroundProcessor__
#define __DCVehicleCounter__BackgroundProcessor__

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cvaux.hpp>
#include "EdgeDetector.h"
#include "debug.h"

using namespace cv;

class BackgroundProcessor
{
protected:
    static const int MOG2_HISTORY = 200;
    static const int MOG2_THRESH = 8;
    static const int EROSION_SIZE = 6;
    static const int DILATION_SIZE = 4;
    static const int BLUR_KERNEL_SIZE = 11; // Must be odd
    constexpr static const double ACCUMULATE_WEIGHT = 0.1;
    
    typedef struct _CONFIG
    {
        int bg_history;
        int bg_threshold;
        bool bg_shadow;
        
        int erosion_size;
        int dilation_size;
        int blur_size;
    } CONFIG, *PCONFIG;
    
    CONFIG cfg;
    bool first_run;
    cv::BackgroundSubtractorMOG2 *pbg;
    uint64 frame_number;
    Mat *background;
    Mat *smask;
    Mat temp, accu;
    Mat last_frame, last_fore;
    bool initialize(Size s, int type);
    void calcStillMask(InputArray fore, uint64 fn);
    bool isCameraMoving(InputArray frame, uint64 fn);
    
public:
    BackgroundProcessor();
    ~BackgroundProcessor();
    void setConfigParams(int bh, int bt, bool bs, int es, int ds, int bls);
    virtual bool process(InputArray frame, uint64 fn, OutputArray fore, OutputArray back);
};

class BackgroundProcessorNight : public BackgroundProcessor
{
protected:
    static const int MOG2_HISTORY = 100;
    static const int MOG2_THRESH = 10;
    static const int BLUR_KERNEL_SIZE = 17; // Must be odd
    cv::Mat foreground;
    cv::Mat temp_3c;
    cv::Mat temp_1c;
    cv::Mat edge;
    cv::Mat last_edge;
    cv::Mat lastlast_edge;
    cv::Mat edge_back;
    EdgeDetector *ped;
    EdgeDetector *ped_back;
    cv::BackgroundSubtractorMOG2 *pbg_edge;
public:
    BackgroundProcessorNight();
    ~BackgroundProcessorNight();
    virtual bool process(InputArray frame, uint64 fn, OutputArray fore, OutputArray back);
};

#endif /* defined(__DCVehicleCounter__BackgroundProcessor__) */
