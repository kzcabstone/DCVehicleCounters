//
//  ObjectSplitterVehicleCounter.cpp
//  DCVehicleCounter
//
//  Created by hohe on 7/18/14.
//  Copyright (c) 2014 DongCheng. All rights reserved.
//

#include "ObjectSplitterVehicleCounter.h"

bool ObjectSplitterVehicleCounter::registerObject(InputArray mask, uchar val, Rect &rect, uint area, uint64 fn)
{
    bool ret = false;
    // 0. Get a vector of the points in this object
    std::vector<Point2i> points;
    Mat im = mask.getMat();
    
    // 1. Do we have this active object already?
    for (std::list<DetectedObject>::iterator it = objs.begin(); it != objs.end(); ++it)
    {
        if (it->getIdentifier() == val)
        {
            // 2. Update this object with new set of points
            ret = it->updateArea(area, fn);
            ret = it->updateBoundingBox(rect, fn) && ret;
            Point2i center(rect.x + rect.width/2, rect.y + rect.height/2);
            ret = it->updateCenteroid(center, fn) && ret;
            
            DBOUT(std::cout << "Old object, K:" << (uint)val << " Box:" << rect << " Area:" << area << std::endl);
            return ret;
        }
    }
    
    // 3. Create a new object
    DetectedObject new_object(val);
    ret = new_object.updateArea(area, fn);
    ret = new_object.updateBoundingBox(rect, fn) && ret;
    Point2i center(rect.x + rect.width/2, rect.y + rect.height/2);
    ret = new_object.updateCenteroid(center, fn) && ret;
    
    DBOUT(std::cout << "New object, K:" << (uint)val << " Box:" << rect << " Area:" << area << std::endl);
    if (ret)
    {
        objs.push_back(std::move(new_object));
        return true;
    }
    return false;
}

void ObjectSplitterVehicleCounter::drawObjects(InputOutputArray img, uint64 fn)
{
#if 1
    cv::Mat im = img.getMat();
    // Draw bounding boxes for all objects that're active for current frame
    for (std::list<DetectedObject>::iterator it = objs.begin(); it != objs.end(); ++it)
    {
        cv::rectangle(im, it->getLatestBoundingBox(), cv::Scalar(255, 0, 0), 2);
    }
#endif
}