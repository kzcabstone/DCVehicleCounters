//
//  ObjectSplitter.cpp
//  OpenCV_learner
//
//  Created by hohe on 7/1/14.
//  Copyright (c) 2014 hohe. All rights reserved.
//

#include "ObjectSplitter.h"

DetectedObject::DetectedObject(uint8_t iden)
{
    obj_identifier = iden;
    fill_val = iden;
    start_frame = 0;
    stop_frame = 0;
    last_update_frame = 0;
}

DetectedObject::DetectedObject() : DetectedObject(0)
{
}

DetectedObject::~DetectedObject()
{
    obj_identifier = 0;
    start_frame = 0;
    stop_frame = 0;
    last_update_frame = 0;
    history.clear();
    areas.clear();
    centeroids.clear();
    bounding_boxes.clear();
}

uint64 DetectedObject::SN = 257; // Starting from above 8-bit range

bool DetectedObject::updatePoints(std::vector<Point2i> &&points, uint64 fn)
{
    if (start_frame == 0)
    {
        // New object & first update
        start_frame = fn;
    }
    else if (fn - start_frame > HISTORY_LEN)
    {
        // Need to delect the oldest history to make space for new one
        history.pop_front();
    }
    history.push_back(std::move(points)); // reference of rvalue is lvalue, so we need to use std::move here
    last_update_frame = fn;
    return true;
}

bool DetectedObject::updateArea(uint a, uint64 fn)
{
    if (start_frame == 0)
    {
        start_frame = fn;
    }
    else if (fn - start_frame > HISTORY_LEN)
    {
        areas.pop_front();
    }
    areas.push_back(a);
    last_update_frame = fn;
    return true;
}

bool DetectedObject::updateCenteroid(cv::Point2i &p, uint64 fn)
{
    if (start_frame == 0)
    {
        start_frame = fn;
    }
    else if (fn - start_frame > HISTORY_LEN)
    {
        centeroids.pop_front();
    }
    centeroids.push_back(p);
    last_update_frame = fn;
    return true;
}

bool DetectedObject::updateBoundingBox(cv::Rect &r, uint64 fn)
{
    if (start_frame == 0)
    {
        start_frame = fn;
    }
    else if (fn - start_frame > HISTORY_LEN)
    {
        bounding_boxes.pop_front();
    }
    bounding_boxes.push_back(r);
    last_update_frame = fn;
    return true;
}

// Called on every object every frame
void DetectedObject::frameTicker(uint64 fn)
{
    if (fn > last_update_frame)
    {
        // This object has disappeared from the frame
        stop_frame = last_update_frame;
        
        uint64 tmp = obj_identifier;
        
        // Now the object is inactive, change the obj_identifier to more than 8-bit to free up fill_val for active objects
        obj_identifier = DetectedObject::getSN();

        DBOUT(std::cout << "Change Object K:" << this->obj_identifier << " active -> inactive" << " ID " << tmp << "->" << obj_identifier << std::endl);
    }
}

ObjectSplitter::ObjectSplitter() : ObjectSplitter(std::numeric_limits<uint>::lowest(), std::numeric_limits<uint>::max())
{
}

ObjectSplitter::ObjectSplitter(uint _min_area, uint _max_area)
{
    frame_num = 0;
    width = 0;
    height = 0;
    memset(fill_val_used, false, 256);
    fill_val_used[0] = fill_val_used[255] = true;
    min_area = _min_area;
    max_area = _max_area;
}

ObjectSplitter::~ObjectSplitter()
{
    objs.clear();
    frame_num = 0;
    width = 0;
    height = 0;
}

list<DetectedObject>* ObjectSplitter::getAllObjects()
{
    return &objs;
}

DetectedObject* ObjectSplitter::getObject(uint64_t sn)
{
    for (list<DetectedObject>::iterator it = objs.begin(); it != objs.end(); ++it)
    {
        if (it->getIdentifier() == sn)
        {
            return &(*it);
        }
    }
    
    return nullptr;
}

bool ObjectSplitter::deleteObject(uint64_t sn)
{
    bool ret = false;
    
    for (list<DetectedObject>::iterator it = objs.begin(); it != objs.end();)
    {
        if (it->getIdentifier() == sn)
        {
            DBOUT(std::cout << "Delete object, K:" << it->getIdentifier() << " Box:" << it->getLatestBoundingBox() << " Area:" << it->getLatestArea() << std::endl);

            it = objs.erase(it);
            ret = true;
        }
        else
        {
            ++it;
        }
    }
    return ret;
}

bool ObjectSplitter::process(InputArray mask, uint64 fn)
{
    bool ret = false;
    uint w, h;
    w = mask.size().width;
    h = mask.size().height;
    Mat im = mask.getMat();
    
    if ((width == 0) && (height == 0))
    {
        width = w;
        height = h;
        objmask = im.clone();
    }
    
    if ((width != w) || (height != h))
    {
        cerr << "Invalid size. Expecting (" << width << ", " << height << ") got (" << w << ", " << h << ")" << endl;
        return false;
    }
    
    if (fn <= frame_num)
    {
        cerr << "Invalid frame number. Expecting greater than " << frame_num << ", got " << fn << endl;
        return false;
    }
    
    // Cleanup discontinued objects to have a fresh start every frame
    // This must be here instead of in the end of process(),
    // because other class(like vehicle tracker) may need to look at those discontinued objects
    cleanupObjects(fn);
    
    frame_num = fn;

    // Do the job to find old and new objects
    ret = fill(mask, fn);
    
    // Call ticker on every object
    for (std::list<DetectedObject>::iterator it = objs.begin(); it != objs.end(); ++it)
    {
        it->frameTicker(fn);
    }
    
    return ret;
}

// Cleanup disappeared objects
bool ObjectSplitter::cleanupObjects(uint64 fn)
{
    for (std::list<DetectedObject>::iterator it = objs.begin(); it != objs.end(); )
    {
        if (it->isDisappeared(fn))
        {
            DBOUT(std::cout << "Delete object, K:" << it->getIdentifier() << " Box:" << it->getLatestBoundingBox() << " Area:" << it->getLatestArea() << std::endl);
            
            uint8_t f = it->getFillVal();
            it = objs.erase(it);
            fill_val_used[f] = false;
        }
        else
        {
            ++it;
        }
    }
    return true;
}

uint ObjectSplitter::getOverlapArea(Rect *r1, Rect *r2)
{
    uint xa1, ya1, xb1, yb1;
    uint xa2, ya2, xb2, yb2;
    
    xa1 = r1->x;
    ya1 = r1->y;
    xb1 = r1->x + r1->width;
    yb1 = r1->y + r1->height;
    
    xa2 = r2->x;
    ya2 = r2->y;
    xb2 = r2->x + r2->width;
    yb2 = r2->y + r2->height;
    
    uint xao, yao, xbo, ybo;
    xao = std::max(xa1, xa2);
    yao = std::max(ya1, ya2);
    xbo = std::min(xb1, xb2);
    ybo = std::min(yb1, yb2);
    
    return (xbo - xao) * (ybo - yao);
}

/*
 Here's the procedure. 
 For every pixel in the image
 1. check mask first, if it's white (255)
 2. check objmask, if it has a value, go to 3, else go to 4
 3. use that value to flood fill mask
 4. go to next pixel, go to 1
 5. for any leftover white pixel in mask, flood fill mask with a unused value (avoid 0 and 255 as well), and register the new object
 */
bool ObjectSplitter::fill(InputArray mask, uint64 fn)
{
    Mat im = mask.getMat();
    int i, j;
    typedef struct _NewObject
    {
        uchar fill_val;
        cv::Point2i seed;
        cv::Rect box;
        uint area;
    } NewObject;
    typedef std::map<uchar, std::list<NewObject *>*> NewObjectMap;
    NewObjectMap temp_objs;
    
    for (j = 0; j < height; j++)
    {
        for (i = 0; i < width; i++)
        {
            uchar val = im.at<uchar>(Point(i, j)); // NOTE: this is same as im.at<uchar>(j, i)
            if (val == 255)
            {
                // Check objmask
                uchar fill_val = objmask.at<uchar>(Point(i, j));
                
                if ((fill_val != 0) && (fill_val != 255))
                {                    
                    Rect rect0;
                    uchar new_fill_val = this->findFirstUnusedFillVal();
                    uint area0 = cv::floodFill(im, Point(i, j), cv::Scalar(new_fill_val), &rect0, cv::Scalar(0), cv::Scalar(0), 8 | FLOODFILL_FIXED_RANGE | (new_fill_val << 8));
                    if ((area0 < min_area) || (area0 > max_area))
                    {
                        // Too small or too big, fill it to black and continue
                        cv::floodFill(im, Point(i, j), cv::Scalar(0), &rect0, cv::Scalar(0), cv::Scalar(0), 8 | FLOODFILL_FIXED_RANGE | (0 << 8));
                        fill_val_used[new_fill_val] = false;
                        continue;
                    }
                    
                    //fill_val_used[fill_val] = true;
                    if (temp_objs.find(fill_val) == temp_objs.end())
                    {
                        temp_objs[fill_val] = new std::list<NewObject *>;
                    }
                    
                    NewObject *po = new NewObject;
                    po->fill_val = new_fill_val;
                    po->box = rect0;
                    po->seed = Point2i(i, j);
                    po->area = area0;

                    DBOUT(
                    {
                        DetectedObject *pOldObject = this->getObject(fill_val);
                        CV_Assert(pOldObject != nullptr);
                        
                        {
                            // It's possible that all objects with this fill_val are too small or big to be tracked
                            Rect r1 = pOldObject->getLatestBoundingBox();
                            std::cout << "K:" << (uint)fill_val << " Area:" << area0 << " box:" << rect0 << " seed:" << po->seed << " New fill:" << (uint)new_fill_val << " List size:" << temp_objs[fill_val]->size() << " OBox:" << r1 << std::endl;
                        }
                    }
                    );

                    // Save this object to the map
                    temp_objs[fill_val]->push_back(po);
                }
            }
            else
            {
                // Either it's Black or it's already flood filled, do nothing
                continue;
            }
        }
    }
    
    // Now we have all potential same objects in the map, let's go through it and choose the most overlapped as the same objects
    for (NewObjectMap::iterator it = temp_objs.begin(); it != temp_objs.end(); ++it)
    {
        uchar k = it->first;
        std::list<NewObject *> *pList = it->second;
        DetectedObject *pOldObject = this->getObject(k);
        if (pOldObject)
        {
            // It's possible that all objects with this fill_val are too small or big to be tracked
            Rect r1 = pOldObject->getLatestBoundingBox();
            uint max_overlap = 0;
            NewObject *pMax = nullptr;
            for (std::list<NewObject *>::iterator lit = pList->begin(); lit != pList->end(); ++lit)
            {
                uint area = getOverlapArea(&r1, &((*lit)->box));
                CV_Assert(area > 0);
                if (area > max_overlap)
                {
                    max_overlap = area;
                    pMax = *lit;
                }
            }
            
            CV_Assert((pList->size() > 0) && (pMax != nullptr));
            for (std::list<NewObject *>::iterator lit = pList->begin(); lit != pList->end(); ++lit)
            {
                if (*lit != pMax)
                {
                    // New Object, register it
                    registerObject(im, (*lit)->fill_val, (*lit)->box, (*lit)->area, fn);
                }
                else
                {
                    // Free the old fill_val
                    fill_val_used[(*lit)->fill_val] = false;
                    
                    // This is the continue of an old object, flood fill with old fill_val, and register it
                    Rect rect;
                    uint area0 = floodFill(im, (*lit)->seed, Scalar(k), &rect, cv::Scalar(0), cv::Scalar(0), 8 | FLOODFILL_FIXED_RANGE | (k << 8));
                    registerObject(im, k, rect, area0, fn);
                }
            }
        }
    }
    
    // Don't forget to free the memory
    for (NewObjectMap::iterator it = temp_objs.begin(); it != temp_objs.end(); ++it)
    {
        std::list<NewObject *> *pList = it->second;
        for (std::list<NewObject *>::iterator lit = pList->begin(); lit != pList->end(); ++lit)
        {
            delete (*lit);
        }
        delete pList;
    }
    
    // For any left over 255 pixels, must be new objects, flood fill with unused value.
    for (j = 0; j < height; j++)
    {
        for (i = 0; i < width; i++)
        {
            uchar val = im.at<uchar>(Point(i, j));
            if (val == 255)
            {
                // Find the first unused fill_val
                uchar fill_val = findFirstUnusedFillVal();
                CV_Assert(fill_val != 255);
                if (fill_val == 255)
                {
                    // we ran out of fill_val
                    cerr << "Ran out of fill_val" << endl;
                    return false;
                }
                
                Rect rect;
                uint area0 = floodFill(im, Point(i, j), cv::Scalar(fill_val), &rect, cv::Scalar(0), cv::Scalar(0), 8 | FLOODFILL_FIXED_RANGE | (fill_val << 8));
                if ((area0 < min_area) || (area0 > max_area))
                {
                    // unmark the object to black
                    floodFill(im, Point(i, j), cv::Scalar(0), &rect, cv::Scalar(0), cv::Scalar(0), 8 | FLOODFILL_FIXED_RANGE | (0 << 8));
                    fill_val_used[fill_val] = false;
                    continue;
                }

                DBOUT(cout << "RECT3: " << rect << endl);
                // This mean a new object, BTW
                registerObject(im, fill_val, rect, area0, fn);
            }
        }
    }
    
    // Save im as objmask for next frame
    im.copyTo(objmask);
    return true;
}

// TODO: change the behavior to more randomize
uchar ObjectSplitter::findFirstUnusedFillVal()
{
    for (int i = 0; i < 256; i++)
    {
        if (fill_val_used[i] == false)
        {
            fill_val_used[i] = true;
            return i;
        }
    }
    return 255;
}

size_t ObjectSplitter::getNumOfObjects()
{
    return objs.size();
}

void ObjectSplitter::setMinMaxArea(int min, int max)
{
    min_area = min;
    max_area = max;
}
