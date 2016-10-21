//
//  VehicleTracking.cpp
//  DCVehicleCounter
//
//  Created by hohe on 7/19/14.
//  Copyright (c) 2014 DongCheng. All rights reserved.
//

#include "VehicleTracking.h"

VehicleTracking::VehicleTracking(int w, int h) : startDraw(false),
        countAB(0), countBA(0), first_run(true), drawn(false), img_w(w), img_h(h)
{
}

VehicleTracking::~VehicleTracking()
{
    for (std::map<uint64, Vehicle*>::iterator it = vehicles.begin(); it != vehicles.end(); ++it)
    {
        Vehicle *p = it->second;
        if (p)
        {
            delete p;
        }
    }
}

void VehicleTracking::connectROI()
{
    int last_non_zero_x = -1;
    int last_non_zero_y = -1;
    for (int x = 0; x < clh.cols; x++)
    {
        int y = clh.at<int>(0, x);
        if (y > 0)
        {
            if (last_non_zero_x >= 0)
            {
                double dx = x - last_non_zero_x;
                double dy = ((double)(y - last_non_zero_y))/dx;
                if (dx > 1)
                {
                    // Need to fill the gap between last_non_zero_x and x
                    for (int i = last_non_zero_x + 1; i < x; i++)
                    {
                        clh.at<int>(0, i) = dy * (i - last_non_zero_x) + last_non_zero_y;
                    }
                }
            }
            
            last_non_zero_x = x;
            last_non_zero_y = y;
        }
    }
    roi_defined = true;
}

#define IN_LEGAL_RANGE(x, y) ((x >= 0) && (x < img_w) && (y >= 0) && (y < img_h))
void VehicleTracking::setCountingLine(cv::Point s1, cv::Point s2)
{
    if (IN_LEGAL_RANGE(s1.x, s1.y) && IN_LEGAL_RANGE(s2.x, s2.y))
    {
        // only supports horizontal
        clh = cv::Mat(1, img_w, CV_32S, Scalar(-1));
        clh.at<int>(s1.x) = s1.y;
        clh.at<int>(s2.x) = s2.y;
        connectROI();
    }
    else
    {
        std::cerr << "Invalid points passed in. " << s1 << ", " << s2 << std::endl;
    }
}

void VehicleTracking::setCountingLine(std::vector<cv::Point> &cl)
{
    int minx, miny, maxx, maxy;
    
    minx = miny = std::numeric_limits<int>::max();
    maxx = maxy = 0;
    
    for (int i = 0; i < cl.size(); i++)
    {
        if (cl[i].x < minx)
        {
            minx = cl[i].x;
        }
        if (cl[i].x > maxx)
        {
            maxx = cl[i].x;
        }
        if (cl[i].y < miny)
        {
            miny = cl[i].y;
        }
        if (cl[i].y > maxy)
        {
            maxy = cl[i].y;
        }
    }
    
    if (IN_LEGAL_RANGE(minx, miny) && IN_LEGAL_RANGE(maxx, maxy))
    {
        clh = cv::Mat(1, img_w, CV_32S, Scalar(-1));
        // horizontal
        for (int i = 0; i < cl.size(); i++)
        {
            if ((cl[i].x < 0) || (cl[i].x >= img_w))
            {
                std::cerr << "Invalid point passed in. Ignore it" << std::endl;
                continue;
            }
            clh.at<int>(cl[i].x) = cl[i].y;
        }
        connectROI();
    }
    else
    {
        std::cerr << "Invalid points passed in." << std::endl;
    }
}


void VehicleTracking::setCountingLine(cv::InputArray ll)
{
    cv::Mat llm = ll.getMat();
    llm.copyTo(clh);
    connectROI();
}

VehicleTracking::VehiclePosition VehicleTracking::getVehiclePosition(const cv::Point2i& centroid)
{
    VehiclePosition vp = VP_NONE;
    
    if (clh.at<int>(/*0, */centroid.x) < 0)
    {
        vp = VP_NONE;
    }
    if (centroid.y < clh.at<int>(/*0, */centroid.x))
    {
        vp = VP_B;
    }
    else
    {
        vp = VP_A;
    }
    
    return vp;
}

/*
void VehicleTracking::saveConfig()
{
    std::string config_file = getConfigFileName();
    FileStorage fs(config_file, FileStorage::WRITE);
    if (fs.isOpened() == false)
    {
        cerr << "Failed to open " << config_file << " file to save config" << endl;
        return;
    }
    
    fs << "ROI" << cfg.roi_defined;
    fs << "LO" << cfg.counting_line_orientation;
    fs << "CLV" << cfg.clv;
    fs << "CLH" << cfg.clh;
    fs.release();
}

void VehicleTracking::loadConfig()
{
    if (cfg.roi_defined)
    {
        return;
    }
    std::string config_file = getConfigFileName();
    FileStorage fs(config_file, FileStorage::READ);
    if (fs.isOpened() == false)
    {
        cerr << "Failed to open " << config_file << " file to load config" << endl;
        return;
    }
    cfg.roi_defined = (int)fs["ROI"];
    cfg.counting_line_orientation = (VehicleTracking::LaneOrientation)(int)fs["LO"];
    fs["CLV"] >> cfg.clv;
    fs["CLH"] >> cfg.clh;
    fs.release();
}
 */
/*
void VehicleTracking::drawTrack(cv::InputOutputArray im, list<cv::Point2i> *pCenter)
{
    Mat imat = im.getMat();
}
*/
/*
void VehicleTracking::drawROI()
{
    if (cfg.counting_line_orientation != LO_HORIZONTAL)
    {
        for (int i = 0; i < cfg.clv.cols; i++)
        {
            cv::circle(*pFrame, cv::Point2i(cfg.clv.at<int>(0, i), i), 1, CV_RGB(255, 0, 255));
        }
    }
    else
    {
        for (int i = 0; i < cfg.clh.cols; i++)
        {
            cv::circle(*pFrame, cv::Point2i(i, cfg.clh.at<int>(0, i)), 1, CV_RGB(255, 0, 255));
        }
    }
}
 */

// crossed_line_objs: returns a vector of all objects that crossed the counting line in this frame
void VehicleTracking::countVehicles(std::vector<cv::Rect> *crossed_line_objs_AB, std::vector<cv::Rect> *crossed_line_objs_BA)
{
    for (std::map<uint64, Vehicle*>::iterator it = vehicles.begin(); it != vehicles.end(); )
    {
        Vehicle *p = it->second;
        if ((p == nullptr) || (p->counted == true))
        {
            ++it;
            continue;
        }
        
        VehiclePosition vpos = VP_NONE;
        VehiclePosition last_vpos = VP_NONE;
        for (std::map<uint64, cv::Point2i>::iterator cit = p->centeroids.begin();
             cit != p->centeroids.end(); ++cit)
        {
            // std::map is sorted by key(frame number), so begin() always gives you the smallest key
            vpos = getVehiclePosition(cit->second);
            if ((vpos == VP_A) && (last_vpos == VP_B))
            {
                countBA++;
                p->counted = true;
                if (crossed_line_objs_BA)
                {
                    crossed_line_objs_BA->push_back(p->boxes[cit->first]);
                }
            }
            else if ((vpos == VP_B) && (last_vpos == VP_A))
            {
                countAB++;
                p->counted = true;
                if (crossed_line_objs_AB)
                {
                    crossed_line_objs_AB->push_back(p->boxes[cit->first]);
                }
            }
            
            //DBOUT(cout << "countVehicles: K:" << it->first << " Center:" << cit->second << " POS:" << vpos << " LPOS:" << last_vpos << endl);

            cv::circle(*pFrame, cit->second, 2, cv::Scalar(0, 0, 255));

            last_vpos = vpos;
        }
        
        ++it;
    }
}

void VehicleTracking::cleanupVehicles()
{
    for (std::map<uint64, Vehicle*>::iterator it = vehicles.begin(); it != vehicles.end(); )
    {
        Vehicle *p = it->second;
        if (p == nullptr)
        {
            ++it;
            continue;
        }
        if (frame_num - p->last_update_frame > CLEANUP_THRESHOLD)
        {
            delete p;
            it = vehicles.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

// crossed_line_objs: returns a vector of all objects that crossed the counting line in this frame 
bool VehicleTracking::process(cv::InputOutputArray im, std::list<DetectedObject> *pObjs, uint *cntAB, uint *cntBA, uint64 fn,
                              std::vector<cv::Rect> *crossed_line_objs_AB, std::vector<cv::Rect> *crossed_line_objs_BA)
{
    frame_num = fn;
    if (im.empty())
    {
        return false;
    }
    
    if (first_run)
    {
        img_w = im.size().width;
        img_h = im.size().height;
    }
    CV_Assert(img_w == im.size().width);
    CV_Assert(img_h == im.size().height);
    
    //--------------------------------------------------------------------------
    cv::Mat imat = im.getMat();
    pFrame = &imat;
    
    //--------------------------------------------------------------------------
    
    for (std::list<DetectedObject>::iterator it = pObjs->begin() ; it != pObjs->end(); ++it)
    {
        DBOUT(std::cout << "VehicleTracking: K:" << it->getIdentifier() << " Fill:" << (uint)it->getFillVal() << " Active:" << it->isActive() << " Disappeared:" << it->isDisappeared(fn) << std::endl);
        updateVehicle(&(*it));
    }
    
    countVehicles(crossed_line_objs_AB, crossed_line_objs_BA);
    *cntAB = countAB;
    *cntBA = countBA;
    
    std::stringstream msg;
    msg << "UP:" << countAB << " DOWN:" << countBA;
    cv::putText(imat, msg.str(), cv::Point(10, 15), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,255));
    
    cleanupVehicles();
    
    first_run = false;
    
    return true;
}

// Update a still actively moving vehicle or a vehicle just became inactive in this frame
// This function NEEDS to be called EVERY frame
void VehicleTracking::updateVehicle(const DetectedObject *p)
{

    if (p->isActive())
    {
        // 1. See if the object is already there
        std::map<uint64, Vehicle*>::iterator fit = vehicles.find(p->getIdentifier());
        if (fit != vehicles.end())
        {
            // Active objects should have identifier and fillval the same
            Vehicle *pV = fit->second;
            CV_Assert(pV != nullptr);
            CV_Assert(pV->obj_identifier == p->getIdentifier());
            // both vehicles in this object and DetectedObject should be updated per frame, DetectedObject update first
            CV_Assert(pV->last_update_frame == p->getLastUpdateFrame() - 1);
            
            pV->centeroids[pV->last_update_frame + 1] = p->getLatestCenteroid();
            pV->boxes[pV->last_update_frame + 1] = p->getLatestBoundingBox();
            pV->last_update_frame++;
            pV->active = true;
        }
        else
        {
            // 2. New car? Not necessarily, try match the track see if it's old car that lost tracking
            if (!matchVehicle(p))
            {
                // 3. I treat this as new car at this point
                newVehicle(p);
            }
        }
    }
    else
    {
        // Because ObjectSplitter only keep inactive frame till the beginning of next frame, this object must have just became inactive in this frame. Identifier is now 64-bit, use fill_val which is our key
        uint64 id = p->getIdentifier();
        uint8_t old_id = p->getFillVal();
        Vehicle *pV = vehicles[old_id];
        CV_Assert(pV->obj_identifier == old_id);
        CV_Assert(pV != nullptr);
        CV_Assert(pV->last_update_frame == p->getLastUpdateFrame());
        
        pV->active = false;
        pV->obj_identifier = id;
        
        // We need to move the node of old_id to id, to do this we create a new node and erase the old node
        vehicles[id] = pV;
        vehicles.erase(old_id);
    }
}

// Try to match a new vehicle we never seen before to see if it might be same vehicle just lost track for sometime
bool VehicleTracking::matchVehicle(const DetectedObject *p)
{
    // TODO: do nothing for now
    return false;
}

void VehicleTracking::newVehicle(const DetectedObject *p)
{
    Vehicle *pV = new Vehicle;
    CV_Assert(pV != nullptr);
    pV->obj_identifier = p->getIdentifier();
    pV->active = true; // Must be active
    pV->last_update_frame = p->getLastUpdateFrame();
    pV->counted = false;
    pV->centeroids[pV->last_update_frame] = p->getLatestCenteroid();
    pV->boxes[pV->last_update_frame] = p->getLatestBoundingBox();
    vehicles[pV->obj_identifier] = pV;
}


