//
//  VehicleMerger.h
//  DCVehicleCounter
//
//  Created by hohe on 8/25/14.
//  Copyright (c) 2014 DongCheng. All rights reserved.
//

#ifndef __DCVehicleCounter__VehicleMerger__
#define __DCVehicleCounter__VehicleMerger__

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cvaux.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "EdgeDetector.h"
#include "pt/PixelBasedAdaptiveSegmenter.h"
#include "ObjectSplitterVehicleCounter.h"
#include "VehicleTracking.h"
#include "SaveLoadConfig.h"
#include "debug.h"

// This class operates on foremask, connects blobs that it believes belong to same vehicle
class VehicleMerger
{
protected:
    typedef struct _BLOB
    {
        cv::Rect box;
        cv::Point2i seed;
        int area;
    } BLOB, *PBLOB;
    
    typedef struct _CONFIG
    {
        int min_area;
        int max_area;
        
        float box_variation_ratio_h, box_variation_ratio_w;
        float rrate, aspeed, vwidth, vlen, lwidth;
        std::vector<int> lane_devider;
    } CONFIG, *PCONFIG;
    
    CONFIG cfg;
    int lwidth_pixels, cached_lines_per_frame;
    int box_width, box_height, imgw, imgh;
    virtual void calculateParams();
    virtual bool isOverlapping(cv::Rect &r1, cv::Rect &r2);
    void merge(cv::InputArray fore, cv::OutputArray foreout, uint64 fn, std::vector<BLOB> &blobs, cv::InputArray frame = cv::noArray());
    void getAllBlobs(cv::InputArray fore, cv::OutputArray foreout, uint64 fn, std::vector<BLOB> &blobs);
    
    void saveConfig();
    void loadConfig();
    
    float countOccupiedLanes(cv::Rect &rr);
    float getLaneWidth(int idx);
    
    void splitBlob(BLOB *pb, cv::InputOutputArray fore, int cnt, cv::InputArray frame = cv::noArray());
    void split(cv::InputArray fore, cv::OutputArray foreout, uint64 fn, std::vector<BLOB> &blobs, cv::InputArray frame = noArray());
public:
    VehicleMerger() : VehicleMerger(20, 20000)
    {
        
    }
    VehicleMerger(int min, int max)
    {
        cfg.min_area = min;
        cfg.max_area = max;
        cfg.rrate = 30; // fps
        cfg.aspeed = 60; // km/h
        cfg.vwidth = 2; // m
        cfg.vlen = 4.5; // m
        cfg.lwidth = 3.75; // m
        lwidth_pixels = 0; // we can't have a default value for this one.
        box_width = 0;
        box_height = 0;
        cached_lines_per_frame = 1;
        cfg.box_variation_ratio_h = 1.5; // Allow the box to be up to 1.5x of vehicle size
        cfg.box_variation_ratio_w = 1.2;
        
        //loadConfig();
    }
    
    ~VehicleMerger()
    {
        //saveConfig();
    }
    
    // set params so that we can calculate the approximate box to decide if it's one vehicle
    void setVehicleParams(float refresh_rate, float avg_speed, float vehicle_width, float vehicle_length, float lane_width, int lane_width_pixels,
                          float box_vr_h, float box_vr_w, int lines_per_frame = 1);
    void setLaneDevider(std::vector<int> &lane_devider, int x1);
    void setMinMaxArea(int min, int max)
    {
        cfg.min_area = min;
        cfg.max_area = max;
    }
    
    void process(cv::InputArray fore, cv::OutputArray foreout, uint64 fn, cv::InputArray frame = cv::noArray());
};
#endif /* defined(__DCVehicleCounter__VehicleMerger__) */
