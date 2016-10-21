//
//  VehicleTracking.h
//  DCVehicleCounter
//
//  Created by hohe on 7/19/14.
//  Copyright (c) 2014 DongCheng. All rights reserved.
//

#ifndef __DCVehicleCounter__VehicleTracking__
#define __DCVehicleCounter__VehicleTracking__

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cvaux.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "ObjectSplitter.h"
#include "SaveLoadConfig.h"
#include "debug.h"


class VehicleTracking
{
protected:
    static const uint CLEANUP_THRESHOLD = 24; // If an object hasn't been updated for this many frames, delete it
    
    enum VehiclePosition
    {
        VP_NONE = 0,
        VP_A  = 1,
        VP_B  = 2
    };

    class Vehicle
    {
    public:
        Vehicle() : obj_identifier(0), counted(false), active(false), last_update_frame(0)
        {
            
        }

        uint64 obj_identifier;
        std::map<uint64, cv::Point2i> centeroids; // Keyed with frame number
        std::map<uint64, cv::Rect> boxes; // Bounding boxes keyed with frame number
        bool counted;
        bool active;
        uint64 last_update_frame;
    };
    
    cv::Mat clh;
    bool roi_defined;
    uint64 frame_num;
    uint countAB, countBA;
    
    bool startDraw;
    bool drawn;
    bool first_run;
    uint img_w;
    uint img_h;
    std::map<uint64, Vehicle*> vehicles;
    
    cv::Mat *pFrame;

    VehiclePosition getVehiclePosition(const cv::Point2i& centroid);
    void saveConfig();
    void loadConfig();
    //void drawTrack(cv::InputOutputArray im, list<cv::Point2i> *pCenter);
    void updateVehicle(const DetectedObject *p);
    bool matchVehicle(const DetectedObject *p);
    void newVehicle(const DetectedObject *p);
    void countVehicles(std::vector<cv::Rect> *crossed_line_objs_AB, std::vector<cv::Rect> *crossed_line_objs_BA);
    void cleanupVehicles();
    void drawROI(); // Draw ROI on *pFrame
    void connectROI(); // Connect all mouser points so that ROI is a connected line
    
public:
    VehicleTracking(int w, int h);
    ~VehicleTracking();
    
    void setCountingLine(std::vector<cv::Point> &cl);
    void setCountingLine(cv::Point s1, cv::Point s2);
    void setCountingLine(cv::InputArray ll);
    bool process(cv::InputOutputArray im, std::list<DetectedObject> *pObjs, uint *cntAB, uint *cntBA, uint64 fn,
                 std::vector<cv::Rect> *crossed_line_objs_AB, std::vector<cv::Rect> *crossed_line_objs_BA);
    
};

#endif /* defined(__DCVehicleCounter__VehicleTracking__) */
