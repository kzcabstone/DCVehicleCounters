//
//  VehicleMerger.cpp
//  DCVehicleCounter
//
//  Created by hohe on 8/25/14.
//  Copyright (c) 2014 DongCheng. All rights reserved.
//

#include "VehicleMerger.h"

void VehicleMerger::merge(cv::InputArray fore, cv::OutputArray foreout, uint64 fn, std::vector<BLOB> &blobs, cv::InputArray frame)
{
    int i, j;
    
    cv::Mat im = fore.getMat();
    
    if (foreout.needed())
    {
        foreout.create(fore.size(), fore.type());
    }
    cv::Mat om = foreout.getMat();
    // then check for ones that are same vehicle and connect them
    for (i = 0; i < blobs.size(); i++)
    {
        for (j = i+1; j < blobs.size(); j++)
        {
            if (isOverlapping(blobs[i].box, blobs[j].box))
            {
                DBOUT(std::cout << "Merging blob[" << i << "] with blob[" << j << "]" << std::endl);
                cv::line(im, blobs[i].seed, blobs[j].seed, cv::Scalar(254), 2);
            }
        }
    }
    DBSHOW("VM", fore);

    // we filled 254, needs to be 255
    cv::threshold(fore, om, 253, 255, cv::THRESH_BINARY);
}

void VehicleMerger::getAllBlobs(cv::InputArray fore, cv::OutputArray foreout, uint64 fn, std::vector<BLOB> &blobs)
{
    int i, j;
    
    cv::Mat im = fore.getMat();
    int width = im.cols;
    int height = im.rows;

    // first get all blobs in the frame
    for (j = 0; j < height; j++)
    {
        for (i = 0; i < width; i++)
        {
            uchar val = im.at<uchar>(cv::Point(i, j));
            if (val == 255)
            {
                BLOB bb;
                bb.seed = cv::Point(i, j);
                bb.area = cv::floodFill(im, cv::Point(i, j), cv::Scalar(254), &bb.box,
                                        cv::Scalar(0), cv::Scalar(0), 8 | cv::FLOODFILL_FIXED_RANGE | (254 << 8));
                if ((bb.area < cfg.min_area) || (bb.area > cfg.max_area))
                {
                    // Too small or too big, fill it to black and continue
                    cv::floodFill(im, cv::Point(i, j), cv::Scalar(0), &bb.box, cv::Scalar(0), cv::Scalar(0), 8 | cv::FLOODFILL_FIXED_RANGE | (0 << 8));
                    continue;
                }
                blobs.push_back(bb);
            }
        }
    }
}

void VehicleMerger::process(cv::InputArray fore, cv::OutputArray foreout, uint64 fn, cv::InputArray frame)
{
    imgw = fore.size().width;
    imgh = fore.size().height;
    
    std::vector<BLOB> blobs;
    getAllBlobs(fore, foreout, fn, blobs);
    merge(fore, foreout, fn, blobs);
    
    cv::Mat om = foreout.getMat();
    split(om, foreout, fn, blobs);
}

void VehicleMerger::calculateParams()
{
    if (lwidth_pixels == 0)
    {
        std::cerr << "Please set lane width in pixels first." << std::endl;
        return;
    }
    float aspeed_ms = cfg.aspeed / 3.6;
    int vh_pixels = cached_lines_per_frame * cfg.rrate * cfg.vlen/aspeed_ms;
    int vw_pixels = lwidth_pixels * cfg.vwidth/cfg.lwidth;
    box_height = vh_pixels * cfg.box_variation_ratio_h;
    box_width = vw_pixels * cfg.box_variation_ratio_w;
    DBOUT(std::cout << "Box: " << box_width << "x" << box_height << std::endl);
}

bool VehicleMerger::isOverlapping(cv::Rect &r1, cv::Rect &r2)
{
    // Get the bounding box of both
    int x0, y0, x1, y1;
    x0 = std::min(r1.x, r2.x);
    y0 = std::min(r1.y, r2.y);
    x1 = std::max(r1.x + r1.width, r2.x + r2.width);
    y1 = std::max(r1.y + r1.height, r2.y + r2.height);
    
    cv::Rect rr(x0, y0, x1 - x0, y1 - y0);
    float cntLane = countOccupiedLanes(rr);
    bool bOverlapW, bOverlapH;
    bOverlapW = (cntLane <= cfg.box_variation_ratio_w);
    bOverlapH = (rr.height <= box_height);
    
    return (bOverlapH && bOverlapW);
    
    /*
    // Now compare it with the vehicle box
    int new_width, new_height;
    new_width = x1 - x0;
    new_height = y1 - y0;
    if ((new_width < box_width)
        && (new_height < box_height))
    {
        return true;
    }
    else
    {
        return false;
    }
     */
}

// set params so that we can calculate the approximate box to decide if it's one vehicle
void VehicleMerger::setVehicleParams(float refresh_rate, float avg_speed, float vehicle_width, float vehicle_length, float lane_width, int lane_width_pixels, float box_vr_h, float box_vr_w, int lines_per_frame)
{
    cfg.rrate = refresh_rate;
    cfg.aspeed = avg_speed;
    cfg.vwidth = vehicle_width;
    cfg.vlen = vehicle_length;
    cfg.lwidth = lane_width;
    cfg.box_variation_ratio_h = box_vr_h;
    cfg.box_variation_ratio_w = box_vr_w;
    lwidth_pixels = lane_width_pixels;
    cached_lines_per_frame = lines_per_frame;
    calculateParams();
}

void VehicleMerger::setLaneDevider(std::vector<int> &lane_devider, int x1)
{
    cfg.lane_devider = lane_devider;
    // Then we need to substract the offset -- x1
    for (int i = 0; i < cfg.lane_devider.size(); i++)
    {
        cfg.lane_devider[i] -= x1;
    }
}

float VehicleMerger::getLaneWidth(int idx)
{
    if (idx == 0)
    {
        return cfg.lane_devider[0];
    }
    else if (idx == cfg.lane_devider.size())
    {
        return imgw - cfg.lane_devider[idx - 1];
    }
    else
    {
        return cfg.lane_devider[idx] - cfg.lane_devider[idx - 1];
    }
}

#define IS_IN_BLOB(xx, rr) ((xx >= rr.x) && (xx < rr.x + rr.width))
float VehicleMerger::countOccupiedLanes(cv::Rect &rr)
{
    bool *flagLane = new bool[cfg.lane_devider.size() + 1];
    bool *flagDevider = new bool[cfg.lane_devider.size() + 2];
    std::memset(flagLane, false, cfg.lane_devider.size() + 1);
    std::memset(flagDevider, false, cfg.lane_devider.size() + 2);
    
    int cntLane = 0;
    int cntDevider = 0;
    int i;
    for (i = 0; i < cfg.lane_devider.size() + 1; i++)
    {
        if (i < cfg.lane_devider.size())
        {
            if (IS_IN_BLOB(cfg.lane_devider[i], rr))
            {
                cntDevider++;
                flagDevider[i+1] = true;
            }
        }
        
        if (i == 0)
        {
            if (IS_IN_BLOB(0, rr) && IS_IN_BLOB(cfg.lane_devider[i], rr))
            {
                cntLane++;
                flagLane[i] = true;
            }
        }
        else if (i == cfg.lane_devider.size())
        {
            if (IS_IN_BLOB(imgw, rr) && IS_IN_BLOB(cfg.lane_devider[i], rr))
            {
                cntLane++;
                flagLane[i] = true;
            }
        }
        else
        {
            if (IS_IN_BLOB(cfg.lane_devider[i], rr) && IS_IN_BLOB(cfg.lane_devider[i - 1], rr))
            {
                cntLane++;
                flagLane[i] = true;
            }
        }
    }
    
    if (IS_IN_BLOB(0, rr))
    {
        cntDevider++;
        flagDevider[0] = true;
    }
    
    if (IS_IN_BLOB(imgw, rr))
    {
        cntDevider++;
        flagDevider[cfg.lane_devider.size() + 1] = true;
    }
    
    float lanew = 0;
    // If there are multiple lanes inside the blob, then average them to get the lane_width for this blob
    if (cntLane >= 1)
    {
        for (i = 0; i < cfg.lane_devider.size() + 1; i++)
        {
            if (flagLane[i])
            {
                lanew += getLaneWidth(i);
            }
        }
        lanew /= cntLane;
    }
    // If there are no lanes inside the blob, then there're 2 cases
    // 1. there's 1 lane devider in the blob
    else if (cntDevider > 0)
    {
        for (i = 0; i < cfg.lane_devider.size() + 2; i++)
        {
            if (flagDevider[i])
            {
                if (i == 0)
                {
                    lanew = getLaneWidth(i);
                }
                else if (i == cfg.lane_devider.size() + 1)
                {
                    lanew = getLaneWidth(i);
                }
                else
                {
                    float a1, a2;
                    a1 = getLaneWidth(i - 1);
                    a2 = getLaneWidth(i);
                    float r1, r2;
                    r1 = (float)(cfg.lane_devider[i - 1] - rr.x)/rr.width;
                    r2 = 1 - r1;
                    lanew = a1 * r1 + a2 * r2;
                }
            }
        }
    }
    // 2. the blob falls in 1 lane completely
    else
    {
        delete[] flagLane;
        delete[] flagDevider;
        return 1;
    }
    
    float ratio = ((float)rr.width) / lanew;
    
    delete[] flagLane;
    delete[] flagDevider;
    
    return ratio;
}

void VehicleMerger::split(cv::InputArray fore, cv::OutputArray foreout, uint64 fn, std::vector<BLOB> &blobs, cv::InputArray frame)
{
    int i;
    
    cv::Mat im = fore.getMat();
    
    if (foreout.needed())
    {
        foreout.create(fore.size(), fore.type());
    }
    cv::Mat om = foreout.getMat();
    im.copyTo(om);
    // then check for ones that are same vehicle and connect them
    for (i = 0; i < blobs.size(); i++)
    {
        float fcntLane = countOccupiedLanes(blobs[i].box);
        int cntLane = round(fcntLane);
        if (fcntLane > 1.5) // TODO: make this a config param
        {
            DBOUT(std::cout << "Splitting blob " << i << " into " << cntLane << "parts. " << std::endl);
            splitBlob(&blobs[i], om, cntLane, frame);
        }
    }
    DBSHOW("splitted", om);
}

void VehicleMerger::splitBlob(BLOB *pb, cv::InputOutputArray fore, int cnt, cv::InputArray frame)
{
    // TODO: look at frame to better determine if the blob needs to be splitted, if yes, how many sub blobs
    
    cv::Mat im = fore.getMat();
    // For now: equally split the blob into cnt pieces
    cv::Rect rr = pb->box;
    int width = rr.width / cnt;
    for (int i = 1; i < cnt; i++)
    {
        cv::line(im, cv::Point(rr.x + i * width, rr.y),
                 cv::Point(rr.x + i * width, rr.y + rr.height), cv::Scalar(0), 2);
    }
}
