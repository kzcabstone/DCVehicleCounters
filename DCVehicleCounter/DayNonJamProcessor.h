//
//  DayNonJamProcessor.h
//  DCVehicleCounter
//
//  Created by hohe on 8/26/14.
//  Copyright (c) 2014 DongCheng. All rights reserved.
//

#ifndef __DCVehicleCounter__DayNonJamProcessor__
#define __DCVehicleCounter__DayNonJamProcessor__

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cvaux.hpp>
#include "SaveLoadConfig.h"
#include "VehicleTracking.h"
#include "ObjectSplitterVehicleCounter.h"
#include "BackgroundProcessor.h"
#include "debug.h"

#define _USE_PBAS_ 0
// Handles daylight(no headlight), non traffic jam case.
class DayNonJamProcessor : public SaveLoadConfig
{
protected:
    typedef struct _CONFIG
    {
        int bg_history;
        int bg_threshold;
        bool bg_shadow;
        
        bool roi_defined;
        cv::Mat clh; // Array index is x, array element is y, could have same y but 2 different x
        int x11, x12, y11, y12, x21, x22, y21, y22; // Four vertex of detect area
        
        int min_area;
        int max_area;
        int erosion_size;
        int dilation_size;
        int blur_size;
    } CONFIG, *PCONFIG;
    
    CONFIG cfg;
    bool first_run;
    bool startDraw;
    bool drawn;
    
    enum DRAW_STATE
    {
        INIT = 0,
        LINE,
        P11,
        P12,
        P21,
        P22,
        DONE,
        RUNNING
    };
    DRAW_STATE dstate;
    cv::Mat *pframe;
    
#if _USE_PBAS_
    PixelBasedAdaptiveSegmenter pbas;
#else
    BackgroundProcessor *pbg;
#endif
    VehicleTracking *pvt;
    ObjectSplitterVehicleCounter *splitter;
    
    void initialize(cv::InputArray frame, uint64 fn);
    void saveConfig();
    void loadConfig();
    void setupROI(cv::InputArray frame);
    void drawROI(cv::InputOutputArray frame);
    void drawROI();
    void drawCountedVehicles(cv::InputOutputArray frame, std::vector<cv::Rect> *pAB, std::vector<cv::Rect> *pBA);
    
public:
    DayNonJamProcessor();
    ~DayNonJamProcessor();
    void onMouse(int evt, int x, int y, int flag);
    static void mouser(int evt, int x, int y, int flag, void* this_);
    void process(cv::InputArray frame, cv::OutputArray oframe, uint64 fn, unsigned int *cntAB, unsigned int *cntBA);
};

#endif /* defined(__DCVehicleCounter__DayNonJamProcessor__) */
