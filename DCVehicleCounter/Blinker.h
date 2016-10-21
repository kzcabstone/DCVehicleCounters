//
//  Blinker.h
//  DCVehicleCounter
//
//  Created by hohe on 8/5/14.
//  Copyright (c) 2014 DongCheng. All rights reserved.
//

#ifndef __DCVehicleCounter__Blinker__
#define __DCVehicleCounter__Blinker__

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cvaux.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "EdgeDetector.h"
#include "pt/PixelBasedAdaptiveSegmenter.h"
#include "ObjectSplitterVehicleCounter.h"
#include "VehicleTracking.h"
#include "VehicleMerger.h"
#include "SaveLoadConfig.h"
#include "debug.h"

class LBlinker : public SaveLoadConfig
{
protected:
    const static int MIN_BLOB_AREA = 10;
    const static int MAX_BLOB_AREA = 20000;
    std::deque<cv::Mat *> pl;
    int cache_line_type;
    // Since our processing introduces delay, we buffer the frames to show them in corresponding delay
    std::deque<cv::Mat *> framebuffer;
    cv::Mat *pframe;
    uint64 frame_num;
    EdgeDetector *ped;
    cv::BackgroundSubtractorMOG2 *pbg;
    cv::Mat ttemp, temp_3c;
    /*
    PixelBasedAdaptiveSegmenter pbas;
    */
    enum DRAW_STATE
    {
        INIT = 0,
        Y,
        X1,
        X2,
        Y1,
        Y2,
        X11,
        X12,
        X21,
        X22,
        LDIV,
        DONE,
        RUNNING
    };
    
    typedef struct _CONFIG
    {
        int BUF_LEN;
        bool line_defined;
        int x1, x2, y, y1, y2, x11, x12, x21, x22;
        int ed_thresh1, ed_thresh2;
        int bg_history, bg_thresh;
        bool bg_shadow;
        std::vector<int> lane_devider;
        int lines_per_frame;
        
        int min_area;
        int max_area;
        
        float box_variation_ratio_h, box_variation_ratio_w;
        float rrate, aspeed, vwidth, vlen, lwidth;
        
        int contour_thickness;
    } CONFIG, *PCONFIG;
    
    CONFIG cfg;
    
    DRAW_STATE dstate;
    bool startDraw;
    bool first_run;
    int element_size, imgw, imgh, line_len, line_offset;
    int narrowest_lane_width;
    
    VehicleMerger vm;
    ObjectSplitterVehicleCounter os;
    VehicleTracking *pvt;
    
    void setupLine();
    void drawLine();
    void drawBlinker(std::vector<cv::Rect> *crossed_line_objs);
    virtual void cacheLine();
    virtual void cacheLine(cv::InputArray frame);
    void cacheLine(cv::Mat *pLine);
    void cacheFrame(cv::InputArray frame, int delay);
    
    void initialize();
    void saveConfig();
    void loadConfig();
    
    int getNarrowestLane();
    
public:
    LBlinker();
    LBlinker(MODE m);
    ~LBlinker();
    
    virtual void process(cv::InputArray frame, uint *count, uint64 fn);
    void onMouse(int evt, int x, int y, int flag);
    static void mouser(int evt, int x, int y, int flag, void* this_);

    void getPicture(cv::OutputArray pic);
    void setBufLen(int len)
    {
        cfg.BUF_LEN = len;
    }

};

class LBlinkerHeadLight : public LBlinker
{
protected:
    virtual void cacheLine(cv::InputArray frame);
    
public:
    LBlinkerHeadLight(MODE m) : LBlinker(m)
    {
        cfg.lines_per_frame = 3;
    }
    virtual void process(cv::InputArray frame, uint *count, uint64 fn);
};

class LBlinkerDay : public LBlinker
{
protected:
    void initialize();
    double average_gray_value;
    uint average_gray_value_samples;
    uint moving_average_window_len;
    // This is for Day use only for now. Since we use the road color (gray) to stabilize the frame
    void cacheStabilizeLine(cv::InputArray frame);    
public:
    virtual void process(cv::InputArray frame, uint *count, uint64 fn);
};

#endif /* defined(__DCVehicleCounter__Blinker__) */
