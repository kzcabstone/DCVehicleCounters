//
//  ObjectSplitter.h
//  OpenCV_learner
//
//  Created by hohe on 7/1/14.
//  Copyright (c) 2014 hohe. All rights reserved.
//

#ifndef __OpenCV_learner__ObjectSplitter__
#define __OpenCV_learner__ObjectSplitter__

#include "debug.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cvaux.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

class DetectedObject
{
public:
    DetectedObject();
    DetectedObject(uint8_t iden);
    ~DetectedObject();
    bool updatePoints(std::vector<Point2i> &&points, uint64 fn);
    bool updateCenteroid(cv::Point2i &p, uint64 fn);
    bool updateBoundingBox(cv::Rect &r, uint64 fn);
    bool updateArea(uint a, uint64 fn);
    
    // this is to detect if the object is active or not
    void frameTicker(uint64 fn);
    
    bool operator==(const DetectedObject &other)
    {
        return (obj_identifier == other.obj_identifier);
    }
    
    uint64 getIdentifier() const
    {
        return obj_identifier;
    }
    
    bool isActive() const
    {
        return ((stop_frame == 0) && (start_frame != 0));
    }
    
    bool isDisappeared(uint64 fn) const
    {
        return ((stop_frame < fn) && (stop_frame != 0));
    }
    
    const Rect& getLatestBoundingBox() const
    {
        return bounding_boxes.back();
    }
    
    uint getLatestArea() const
    {
        return areas.back();
    }
    
    const cv::Point2i& getLatestCenteroid() const
    {
        return centeroids.back();
    }
    
    uint64 getLastUpdateFrame() const
    {
        return last_update_frame;
    }
    
    uint8_t getFillVal() const
    {
        return fill_val;
    }
    
    uint64 getStartFrame() const
    {
        return start_frame;
    }
    
    // idx: 0 for current frame
    //      -1 for last frame
    //      and so on
    std::vector<cv::Point2i> *getPointsHistory(int idx)
    {
        if (abs(idx) >= HISTORY_LEN)
        {
            std::cerr << "getPointsHistory: Out of range. " << idx << std::endl;
            return nullptr;
        }
        int i = 0;
        std::list<std::vector<cv::Point2i>>::reverse_iterator it;
        for (it = history.rbegin(); (it != history.rend()) && (i > idx); it++, i--)
        {
        }
        return &(*it);
    }
    
    cv::Rect *getBoxesHistory(int idx)
    {
        if (abs(idx) >= HISTORY_LEN)
        {
            std::cerr << "getBoxesHistory: Out of range. " << idx << std::endl;
            return nullptr;
        }
        int i = 0;
        std::list<cv::Rect>::reverse_iterator it;
        for (it = bounding_boxes.rbegin(); (it != bounding_boxes.rend()) && (i > idx); it++, i--)
        {
        }
        return &(*it);
    }
    
    cv::Point2i *getCenteroidsHistory(int idx)
    {
        if (abs(idx) >= HISTORY_LEN)
        {
            std::cerr << "getCenteroidsHistory: Out of range. " << idx << std::endl;
            return nullptr;
        }
        int i = 0;
        std::list<cv::Point2i>::reverse_iterator it;
        for (it = centeroids.rbegin(); (it != centeroids.rend()) && (i > idx); it++, i--)
        {
        }
        return &(*it);
    }
    
    uint getAreaHistory(int idx)
    {
        if (abs(idx) >= HISTORY_LEN)
        {
            std::cerr << "getAreaHistory: Out of range. " << idx << std::endl;
            return 0;
        }
        int i = 0;
        std::list<uint>::reverse_iterator it;
        for (it = areas.rbegin(); (it != areas.rend()) && (i > idx); it++, i--)
        {
        }
        return (*it);
    }

protected:
    uint64 obj_identifier; // unique obj serial number
    uint8_t fill_val; // Value that used for floodFill for this object, this also is the unique obj SN when the obj is active
    static const uint HISTORY_LEN = 32; // 32 frames of history
    list<vector<Point2i>> history; // history of object points
    std::list<cv::Point2i> centeroids; // centeroid history
    list<cv::Rect> bounding_boxes; // bounding box history
    list<uint> areas; // Area history
    uint64 start_frame; // The frame # when the object was first detected
    uint64 stop_frame; // The frame # when the object last appears in
    uint64 last_update_frame; // Frame when this object was last updated
    
    static uint64 SN;
    // TODO: make this thread safe
    static uint64 getSN()
    {
        return ++SN;
    }
};

class ObjectSplitter
{
protected:
    uint max_area;
    uint min_area; // Area out of this range is not tracked

    list<DetectedObject> objs;
    uint64 frame_num;
    uint width, height;
    Mat objmask;
    bool fill_val_used[256];
    virtual bool fill(InputArray mask, uint64 fn);
    uchar findFirstUnusedFillVal();
    bool cleanupObjects(uint64 fn);
    uint getOverlapArea(Rect *r1, Rect *r2);
    
    // For subclass to implement to take whatever informatino in their interest
    virtual bool registerObject(InputArray mask, uchar val, Rect &rect, uint area, uint64 fn) = 0;
    
public:
    ObjectSplitter();
    ObjectSplitter(uint _min_area, uint _max_area);
    ~ObjectSplitter();
    bool process(InputArray mask, uint64 fn);
    list<DetectedObject>* getAllObjects();
    size_t getNumOfObjects();
    DetectedObject* getObject(uint64 sn);
    bool deleteObject(uint64_t sn);
    // For subclass to decide how to draw their objects
    virtual void drawObjects(InputOutputArray img, uint64 fn) = 0;
    void setMinMaxArea(int min, int max);
};
#endif /* defined(__OpenCV_learner__ObjectSplitter__) */
