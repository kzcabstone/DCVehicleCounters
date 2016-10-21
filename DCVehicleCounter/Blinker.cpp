//
//  Blinker.cpp
//  DCVehicleCounter
//
//  Created by hohe on 8/5/14.
//  Copyright (c) 2014 DongCheng. All rights reserved.
//

#include "Blinker.h"

LBlinker::LBlinker(MODE m) : SaveLoadConfig(IDS_BLINKER, m), dstate(INIT), pframe(nullptr), ped(nullptr), pbg(nullptr), imgw(0), imgh(0), frame_num(0), narrowest_lane_width(0)
{
    cfg.x1 = 0;
    cfg.x2 = 0;
    cfg.y = 0;
    cfg.x11 = 0;
    cfg.x12 = 0;
    cfg.x21 = 0;
    cfg.x22 = 0;
    cfg.y1 = 0;
    cfg.y2 = 0;
    cfg.line_defined = false;
    cfg.BUF_LEN = 100;
    cfg.ed_thresh1 = 200;
    cfg.ed_thresh2 = 600;
    cfg.bg_history = 400;
    cfg.bg_thresh = 1;
    cfg.lines_per_frame = 1;
    first_run = true;
    
    //loadConfig();
    
    ped = new EdgeDetector(cfg.ed_thresh1, cfg.ed_thresh2);
    pbg = new cv::BackgroundSubtractorMOG2(cfg.bg_history, cfg.bg_thresh, false);
}

LBlinker::LBlinker() : LBlinker(MODE_DEFAULT)
{
   
}

LBlinker::~LBlinker()
{
    saveConfig();
    while (!pl.empty())
    {
        cv::Mat *ptmp = pl.back();
        if (ptmp)
        {
            delete ptmp;
        }
        pl.pop_back();
    }
    
    while (!framebuffer.empty())
    {
        cv::Mat *ptmp = framebuffer.back();
        if (ptmp)
        {
            delete  ptmp;
        }
        framebuffer.pop_back();
    }
    
    if (ped)
    {
        delete ped;
        ped = nullptr;
    }
    if (pbg)
    {
        delete pbg;
        pbg = nullptr;
    }
    if (pvt)
    {
        delete pvt;
    }
}

void LBlinker::saveConfig()
{
    std::string config_file = getConfigFileName();
    cv::FileStorage fs(config_file, cv::FileStorage::WRITE);
    if (fs.isOpened() == false)
    {
        std::cerr << "Failed to open " << config_file << " to save config" << std::endl;
        return;
    }
    
    cv::Mat temp = cv::Mat(cfg.lane_devider);
    
    fs << "BL" << cfg.BUF_LEN;
    fs << "LD" << cfg.line_defined;
    fs << "x1" << cfg.x1;
    fs << "x2" << cfg.x2;
    fs << "y" << cfg.y;
    fs << "y1" << cfg.y1;
    fs << "y2" << cfg.y2;
    fs << "x11" << cfg.x11;
    fs << "x12" << cfg.x12;
    fs << "x21" << cfg.x21;
    fs << "x22" << cfg.x22;
    fs << "EDT1" << cfg.ed_thresh1;
    fs << "EDT2" << cfg.ed_thresh2;
    fs << "BGH" << cfg.bg_history;
    fs << "BGT" << cfg.bg_thresh;
    fs << "BGS" << cfg.bg_shadow;
    fs << "LPF" << cfg.lines_per_frame;
    fs << "CTT" << cfg.contour_thickness;
    fs << "LDIV" << temp;
    
    fs << "MIA" << cfg.min_area;
    fs << "MAA" << cfg.max_area;
    fs << "BVRH" << cfg.box_variation_ratio_h;
    fs << "BVRW" << cfg.box_variation_ratio_w;
    fs << "RR" << cfg.rrate;
    fs << "AS" << cfg.aspeed;
    fs << "VW" << cfg.vwidth;
    fs << "VL" << cfg.vlen;
    fs << "LW" << cfg.lwidth;
    
    fs.release();
}

void LBlinker::loadConfig()
{
    std::string config_file = getConfigFileName();
    cv::FileStorage fs(config_file, cv::FileStorage::READ);
    if (fs.isOpened() == false)
    {
        std::cerr << "Failed to open " << config_file << " file to load config" << std::endl;
        return;
    }
    cv::Mat temp;
    
    cfg.BUF_LEN = fs["BL"];
    cfg.line_defined = (int)fs["LD"];
    cfg.x1 = fs["x1"];
    cfg.x2 = fs["x2"];
    cfg.y = fs["y"];
    cfg.y1 = fs["y1"];
    cfg.y2 = fs["y2"];
    cfg.x11 = fs["x11"];
    cfg.x12 = fs["x12"];
    cfg.x21 = fs["x21"];
    cfg.x22 = fs["x22"];
    cfg.ed_thresh1 = fs["EDT1"];
    cfg.ed_thresh2 = fs["EDT2"];
    cfg.bg_history = fs["BGH"];
    cfg.bg_thresh = fs["BGT"];
    cfg.bg_shadow = (int)fs["BGS"];
    cfg.lines_per_frame = fs["LPF"];
    cfg.contour_thickness = fs["CTT"];
    
    cfg.min_area = fs["MIA"];
    cfg.max_area = fs["MAA"];
    cfg.box_variation_ratio_h = fs["BVRH"];
    cfg.box_variation_ratio_w = fs["BVRW"];
    cfg.rrate = fs["RR"];
    cfg.aspeed = fs["AS"];
    cfg.vwidth = fs["VW"];
    cfg.vlen = fs["VL"];
    cfg.lwidth = fs["LW"];
    
    fs["LDIV"] >> temp;
    CV_Assert(temp.cols == 1);
    for (int i = 0; i < temp.rows; i++)
    {
        cfg.lane_devider.push_back(temp.at<int>(i, 0));
    }
    
    fs.release();
    
    config_loaded = true;
}

// Mouser
void LBlinker::mouser(int evt, int x, int y, int flag, void* this_)
{
    //std::cout << "Mouser " << x << ", " << y << std::endl;
    static_cast<LBlinker*>(this_)->onMouse(evt, x, y, flag);
}

void LBlinker::onMouse(int evt, int xx, int yy, int flag)
{
    //std::cout << "onMouse " << dstate << ", " << evt << " y:" << y << " x1:" << x1 << " x2:" << x2 << std::endl;
    switch (dstate)
    {
        case INIT:
        case Y:
            if (evt == CV_EVENT_LBUTTONDOWN)
            {
                cfg.y = yy;
                dstate = X1;
                DBOUT(std::cout << "Y: " << cfg.y << std::endl);
            }
            break;
        case X1:
            if (evt == CV_EVENT_LBUTTONDOWN)
            {
                cfg.x1 = xx;
                dstate = X2;
                DBOUT(std::cout << "X1: " << cfg.x1 << std::endl);
            }
            break;
        case X2:
            if (evt == CV_EVENT_LBUTTONDOWN)
            {
                cfg.x2 = xx;
                dstate = Y1;
                DBOUT(std::cout << "X2: " << cfg.x2 << std::endl);
            }
            break;
        case Y1:
            if (evt == CV_EVENT_LBUTTONDOWN)
            {
                cfg.y1 = yy;
                dstate = Y2;
                DBOUT(std::cout << "Y1: " << cfg.y1 << std::endl);
            }
            break;
        case Y2:
            if (evt == CV_EVENT_LBUTTONDOWN)
            {
                cfg.y2 = yy;
                dstate = X11;
                DBOUT(std::cout << "Y2: " << cfg.y2 << std::endl);
            }
            break;
        case X11:
            if (evt == CV_EVENT_LBUTTONDOWN)
            {
                cfg.x11 = xx;
                dstate = X12;
                DBOUT(std::cout << "X11: " << cfg.x11 << std::endl);
            }
            break;
        case X12:
            if (evt == CV_EVENT_LBUTTONDOWN)
            {
                cfg.x12 = xx;
                dstate = X21;
                DBOUT(std::cout << "X12: " << cfg.x12 << std::endl);
            }
            break;
        case X21:
            if (evt == CV_EVENT_LBUTTONDOWN)
            {
                cfg.x21 = xx;
                dstate = X22;
                DBOUT(std::cout << "X21: " << cfg.x21 << std::endl);
            }
            break;
        case X22:
            if (evt == CV_EVENT_LBUTTONDOWN)
            {
                cfg.x22 = xx;
                dstate = LDIV;
                DBOUT(std::cout << "X22: " << cfg.x22 << std::endl);
            }
            break;
        case LDIV:
            if (evt == CV_EVENT_LBUTTONDOWN)
            {
                cfg.lane_devider.push_back(xx);
                dstate = LDIV;
                DBOUT(std::cout << "LD" << cfg.lane_devider.size() << ": " << xx << std::endl);
            }
            if (evt == CV_EVENT_RBUTTONDOWN)
            {
                dstate = DONE;
                cfg.line_defined = true;
                DBOUT(std::cout << "Done, press any key to continue" << std::endl);
            }
            break;
        case DONE:
            //dstate = RUNNING;
            cvSetMouseCallback("blinker", nullptr);
            break;
        default:
            break;
    }
    drawLine();
}

void LBlinker::drawLine()
{
    cv::Mat *pdrawing;
    if (dstate != RUNNING)
    {
        pdrawing = new cv::Mat(pframe->size(), pframe->type());
        pframe->copyTo(*pdrawing);
    }
    else
    {
        pdrawing = framebuffer.front();
    }
    //std::cout << "drawLine " << dstate << " y:" << y << " x1:" << x1 << " x2:" << x2 << std::endl;
    switch (dstate)
    {
        case INIT:
        case Y:
            cv::putText(*pdrawing, "Left click to define the horizontal counting line", cv::Point(10, 100),
                        cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 255));
            break;
        case X1:
            cv::putText(*pdrawing, "Left click to define left endpoint", cv::Point(10, 100),
                        cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 255));
            cv::line(*pdrawing, cv::Point(0, cfg.y), cv::Point(imgw, cfg.y), cv::Scalar(255, 0, 255));
            break;
        case X2:
            cv::putText(*pdrawing, "Left click to define right endpoint", cv::Point(10, 100),
                        cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 255));
            cv::line(*pdrawing, cv::Point(cfg.x1, cfg.y), cv::Point(imgw, cfg.y), cv::Scalar(255, 0, 255));
            cv::circle(*pdrawing, cv::Point(cfg.x1, cfg.y), 2, cv::Scalar(0, 0, 255));
            break;
        case Y1:
            cv::putText(*pdrawing, "Left click to define upper boundary", cv::Point(10, 100),
                        cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 255));
            cv::line(*pdrawing, cv::Point(cfg.x1, cfg.y), cv::Point(cfg.x2, cfg.y), cv::Scalar(255, 0, 255));
            cv::circle(*pdrawing, cv::Point(cfg.x1, cfg.y), 2, cv::Scalar(0, 0, 255));
            cv::circle(*pdrawing, cv::Point(cfg.x2, cfg.y), 2, cv::Scalar(0, 0, 255));
            break;
        case Y2:
            cv::putText(*pdrawing, "Left click to define lower boundary", cv::Point(10, 100),
                        cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 255));
            cv::line(*pdrawing, cv::Point(cfg.x1, cfg.y), cv::Point(cfg.x2, cfg.y), cv::Scalar(255, 0, 255));
            cv::circle(*pdrawing, cv::Point(cfg.x1, cfg.y), 2, cv::Scalar(0, 0, 255));
            cv::circle(*pdrawing, cv::Point(cfg.x2, cfg.y), 2, cv::Scalar(0, 0, 255));
            cv::line(*pdrawing, cv::Point(0, cfg.y1), cv::Point(imgw, cfg.y1), cv::Scalar(255, 0, 255));
            break;
        case X11:
            cv::putText(*pdrawing, "Left click to define upper left corner", cv::Point(10, 100),
                        cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 255));
            cv::line(*pdrawing, cv::Point(cfg.x1, cfg.y), cv::Point(cfg.x2, cfg.y), cv::Scalar(255, 0, 255));
            cv::circle(*pdrawing, cv::Point(cfg.x1, cfg.y), 2, cv::Scalar(0, 0, 255));
            cv::circle(*pdrawing, cv::Point(cfg.x2, cfg.y), 2, cv::Scalar(0, 0, 255));
            cv::line(*pdrawing, cv::Point(0, cfg.y1), cv::Point(imgw, cfg.y1), cv::Scalar(255, 0, 255));
            cv::line(*pdrawing, cv::Point(0, cfg.y2), cv::Point(imgw, cfg.y2), cv::Scalar(255, 0, 255));
            break;
        case X12:
            cv::putText(*pdrawing, "Left click to define upper right corner", cv::Point(10, 100),
                        cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 255));
            cv::line(*pdrawing, cv::Point(cfg.x1, cfg.y), cv::Point(cfg.x2, cfg.y), cv::Scalar(255, 0, 255));
            cv::circle(*pdrawing, cv::Point(cfg.x1, cfg.y), 2, cv::Scalar(0, 0, 255));
            cv::circle(*pdrawing, cv::Point(cfg.x2, cfg.y), 2, cv::Scalar(0, 0, 255));
            cv::line(*pdrawing, cv::Point(cfg.x11, cfg.y1), cv::Point(imgw, cfg.y1), cv::Scalar(255, 0, 255));
            cv::line(*pdrawing, cv::Point(0, cfg.y2), cv::Point(imgw, cfg.y2), cv::Scalar(255, 0, 255));
            cv::circle(*pdrawing, cv::Point(cfg.x11, cfg.y1), 2, cv::Scalar(0, 0, 255));
            break;
        case X21:
            cv::putText(*pdrawing, "Left click to define lower left corner", cv::Point(10, 100),
                        cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 255));
            cv::line(*pdrawing, cv::Point(cfg.x1, cfg.y), cv::Point(cfg.x2, cfg.y), cv::Scalar(255, 0, 255));
            cv::circle(*pdrawing, cv::Point(cfg.x1, cfg.y), 2, cv::Scalar(0, 0, 255));
            cv::circle(*pdrawing, cv::Point(cfg.x2, cfg.y), 2, cv::Scalar(0, 0, 255));
            cv::line(*pdrawing, cv::Point(cfg.x11, cfg.y1), cv::Point(cfg.x12, cfg.y1), cv::Scalar(255, 0, 255));
            cv::line(*pdrawing, cv::Point(0, cfg.y2), cv::Point(imgw, cfg.y2), cv::Scalar(255, 0, 255));
            cv::circle(*pdrawing, cv::Point(cfg.x11, cfg.y1), 2, cv::Scalar(0, 0, 255));
            cv::circle(*pdrawing, cv::Point(cfg.x12, cfg.y1), 2, cv::Scalar(0, 0, 255));
            break;
        case X22:
            cv::putText(*pdrawing, "Left click to define lower left corner", cv::Point(10, 100),
                        cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 255));
            cv::line(*pdrawing, cv::Point(cfg.x1, cfg.y), cv::Point(cfg.x2, cfg.y), cv::Scalar(255, 0, 255));
            cv::circle(*pdrawing, cv::Point(cfg.x1, cfg.y), 2, cv::Scalar(0, 0, 255));
            cv::circle(*pdrawing, cv::Point(cfg.x2, cfg.y), 2, cv::Scalar(0, 0, 255));
            cv::line(*pdrawing, cv::Point(cfg.x11, cfg.y1), cv::Point(cfg.x12, cfg.y1), cv::Scalar(255, 0, 255));
            cv::line(*pdrawing, cv::Point(cfg.x21, cfg.y2), cv::Point(imgw, cfg.y2), cv::Scalar(255, 0, 255));
            cv::circle(*pdrawing, cv::Point(cfg.x11, cfg.y1), 2, cv::Scalar(0, 0, 255));
            cv::circle(*pdrawing, cv::Point(cfg.x12, cfg.y1), 2, cv::Scalar(0, 0, 255));
            cv::circle(*pdrawing, cv::Point(cfg.x21, cfg.y2), 2, cv::Scalar(0, 0, 255));
            break;
        case LDIV:
            cv::putText(*pdrawing, "Left click to mark lanes. Right click to end.", cv::Point(10, 100),
                        cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 255));
            cv::line(*pdrawing, cv::Point(cfg.x1, cfg.y), cv::Point(cfg.x2, cfg.y), cv::Scalar(255, 0, 255));
            cv::circle(*pdrawing, cv::Point(cfg.x1, cfg.y), 2, cv::Scalar(0, 0, 255));
            cv::circle(*pdrawing, cv::Point(cfg.x2, cfg.y), 2, cv::Scalar(0, 0, 255));
            cv::line(*pdrawing, cv::Point(cfg.x11, cfg.y1), cv::Point(cfg.x12, cfg.y1), cv::Scalar(255, 0, 255));
            cv::line(*pdrawing, cv::Point(cfg.x21, cfg.y2), cv::Point(cfg.x22, cfg.y2), cv::Scalar(255, 0, 255));
            cv::circle(*pdrawing, cv::Point(cfg.x11, cfg.y1), 2, cv::Scalar(0, 0, 255));
            cv::circle(*pdrawing, cv::Point(cfg.x12, cfg.y1), 2, cv::Scalar(0, 0, 255));
            cv::circle(*pdrawing, cv::Point(cfg.x21, cfg.y2), 2, cv::Scalar(0, 0, 255));
            cv::circle(*pdrawing, cv::Point(cfg.x22, cfg.y2), 2, cv::Scalar(0, 0, 255));
            cv::line(*pdrawing, cv::Point(cfg.x11, cfg.y1), cv::Point(cfg.x21, cfg.y2), cv::Scalar(255, 0, 255));
            cv::line(*pdrawing, cv::Point(cfg.x12, cfg.y1), cv::Point(cfg.x22, cfg.y2), cv::Scalar(255, 0, 255));
            for (int i = 0; i < cfg.lane_devider.size(); i++)
            {
                cv::circle(*pdrawing, cv::Point(cfg.lane_devider[i], cfg.y), 2, cv::Scalar(0, 255, 0));
            }
            break;
        case DONE:
            cv::putText(*pdrawing, "Done. Press any key.", cv::Point(10, 100), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 255));
            cv::line(*pdrawing, cv::Point(cfg.x1, cfg.y), cv::Point(cfg.x2, cfg.y), cv::Scalar(255, 0, 255));
            cv::circle(*pdrawing, cv::Point(cfg.x1, cfg.y), 2, cv::Scalar(0, 0, 255));
            cv::circle(*pdrawing, cv::Point(cfg.x2, cfg.y), 2, cv::Scalar(0, 0, 255));
            cv::line(*pdrawing, cv::Point(cfg.x11, cfg.y1), cv::Point(cfg.x12, cfg.y1), cv::Scalar(255, 0, 255));
            cv::line(*pdrawing, cv::Point(cfg.x21, cfg.y2), cv::Point(cfg.x22, cfg.y2), cv::Scalar(255, 0, 255));
            cv::circle(*pdrawing, cv::Point(cfg.x11, cfg.y1), 2, cv::Scalar(0, 0, 255));
            cv::circle(*pdrawing, cv::Point(cfg.x12, cfg.y1), 2, cv::Scalar(0, 0, 255));
            cv::circle(*pdrawing, cv::Point(cfg.x21, cfg.y2), 2, cv::Scalar(0, 0, 255));
            cv::circle(*pdrawing, cv::Point(cfg.x22, cfg.y2), 2, cv::Scalar(0, 0, 255));
            cv::line(*pdrawing, cv::Point(cfg.x11, cfg.y1), cv::Point(cfg.x21, cfg.y2), cv::Scalar(255, 0, 255));
            cv::line(*pdrawing, cv::Point(cfg.x12, cfg.y1), cv::Point(cfg.x22, cfg.y2), cv::Scalar(255, 0, 255));
            for (int i = 0; i < cfg.lane_devider.size(); i++)
            {
                cv::circle(*pdrawing, cv::Point(cfg.lane_devider[i], cfg.y), 2, cv::Scalar(0, 255, 0));
            }
            break;
        default:

            cv::line(*pdrawing, cv::Point(cfg.x11, cfg.y1), cv::Point(cfg.x12, cfg.y1), cv::Scalar(255, 0, 255));
            cv::line(*pdrawing, cv::Point(cfg.x21, cfg.y2), cv::Point(cfg.x22, cfg.y2), cv::Scalar(255, 0, 255));
            cv::circle(*pdrawing, cv::Point(cfg.x11, cfg.y1), 2, cv::Scalar(0, 0, 255));
            cv::circle(*pdrawing, cv::Point(cfg.x12, cfg.y1), 2, cv::Scalar(0, 0, 255));
            cv::circle(*pdrawing, cv::Point(cfg.x21, cfg.y2), 2, cv::Scalar(0, 0, 255));
            cv::circle(*pdrawing, cv::Point(cfg.x22, cfg.y2), 2, cv::Scalar(0, 0, 255));
            cv::line(*pdrawing, cv::Point(cfg.x11, cfg.y1), cv::Point(cfg.x21, cfg.y2), cv::Scalar(255, 0, 255));
            cv::line(*pdrawing, cv::Point(cfg.x12, cfg.y1), cv::Point(cfg.x22, cfg.y2), cv::Scalar(255, 0, 255));

            break;
    }
    if (dstate != RUNNING)
    {
        imshow("blinker", *pdrawing);
        delete pdrawing;
    }
}

void LBlinker::setupLine()
{
    if (cfg.line_defined == false)
    {
        do
        {            
            drawLine();
            cvSetMouseCallback("blinker", &mouser, this);
            cvWaitKey(0);

            if(cfg.line_defined)
            {
                DBOUT(std::cout << "Counting line defined (" << cfg.x1 << "," << cfg.y << "," << cfg.x2 << "," << 6 << ")" << std::endl);
                saveConfig();
                break;
            }
            else
            {
                DBOUT(std::cout << "Counting line undefined!" << std::endl);
            }

        } while(1);
    }
    
    if (cfg.line_defined == true)
    {
        line_len = cfg.x2 - cfg.x1;
        line_offset = cfg.x1;
        std::sort(cfg.lane_devider.begin(), cfg.lane_devider.end());
        dstate = RUNNING;
    }
}

void LBlinker::initialize()
{
    imgw = pframe->cols;
    imgh = pframe->rows;
    
    loadConfig();
    setupLine();
    
    if (pframe->type() == CV_8UC3)
    {
        element_size = 3;
    }
    else if (pframe->type() == CV_8UC1)
    {
        element_size = 1;
    }
    
    narrowest_lane_width = getNarrowestLane();
    vm.setVehicleParams(cfg.rrate, cfg.aspeed, cfg.vwidth, cfg.vlen, cfg.lwidth, narrowest_lane_width, cfg.box_variation_ratio_h,
                        cfg.box_variation_ratio_w, cfg.lines_per_frame);
    vm.setMinMaxArea(cfg.min_area, cfg.max_area);
    vm.setLaneDevider(cfg.lane_devider, cfg.x1);
    os.setMinMaxArea(cfg.min_area, cfg.max_area);
    
    pbg->set("detectShadows", cfg.bg_shadow);
    pbg->set("history", cfg.bg_history);
    pbg->set("varThreshold", cfg.bg_thresh);
    
    ped->setLowThreshold(cfg.ed_thresh1);
    ped->setHighThreshold(cfg.ed_thresh2);
    
    pvt = new VehicleTracking(imgw, imgh);
    pvt->setCountingLine(cv::Point(0, cfg.BUF_LEN/2), cv::Point(imgw - 1, cfg.BUF_LEN/2));
}

int LBlinker::getNarrowestLane()
{
    int min = cfg.x2 - cfg.x1;
    if (cfg.lane_devider.size() == 0)
    {
        return min/8; // assume 8 lanes
    }
    
    min = cfg.lane_devider[0] - cfg.x1;
    
    int i;
    for (i = 1; i < cfg.lane_devider.size(); i++)
    {
        int d = cfg.lane_devider[i] - cfg.lane_devider[i-1];
        if (min > d)
        {
            min = d;
        }
    }
    
    if (min > cfg.x2 - cfg.lane_devider[i-1])
    {
        min = cfg.x2 - cfg.lane_devider[i-1];
    }
    
    return min;
}

void LBlinker::process(cv::InputArray frame, uint *count, uint64 fn)
{
    CV_Assert((frame.type() == CV_8UC3) || (frame.type() == CV_8UC1));
    
    cv::Mat im = frame.getMat();
    pframe = &im;
    if (first_run)
    {
        // First time
        initialize();
        first_run = false;
    }
    frame_num = fn;
    
    CV_Assert(imgw == im.cols);
    CV_Assert(imgh == im.rows);
    cacheLine();
    cacheFrame(frame, cfg.BUF_LEN/2/cfg.lines_per_frame);
    drawLine();
    
    cv::Mat lines, lines_contour;
    getPicture(lines);
    ped->process(lines, lines_contour, fn, cfg.contour_thickness);
    
    cv::Mat fore, back;
    cv::Mat ttt1[] = {lines_contour, lines_contour, lines_contour};
    cv::merge(ttt1, 3, back);
    pbg->operator()(back, fore);
    
    if (framebuffer.size() < cfg.BUF_LEN/2/cfg.lines_per_frame + 1)
    {
        // still in bg's background building period
        return;
    }
    
    DBSHOW_POS("fore", fore, 0, 0);

    cv::medianBlur(fore, ttemp, 5);
    
    DBSHOW_POS("1stBlur", ttemp, imgw, 0);
    
     // Not good. some car only gets detected as 2 short lines (headlights), and this will filter them out.
    //cv::Mat morphKernel = getStructuringElement(cv::MORPH_RECT, cv::Size(narrowest_lane_width/4, 1));
    cv::Mat morphKernel = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 4));
    cv::morphologyEx(fore, ttemp, cv::MORPH_OPEN, morphKernel);
    //morphKernel = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    //cv::morphologyEx(ttemp, ttemp, cv::MORPH_DILATE, morphKernel);
    cv::medianBlur(ttemp, ttemp, 7);

    DBSHOW_POS("lines", lines, imgw*2, 0);
    DBSHOW_POS("2ndBlur", ttemp, 0, cfg.BUF_LEN + 80);

    vm.process(ttemp, fore, fn);
    cv::Mat ttt[] = {fore, fore, fore};
    cv::merge(ttt, 3, temp_3c);
    if (os.process(fore, fn))
    {
        os.drawObjects(temp_3c, fn);
    }
    
    DBSHOW_POS("merged", temp_3c, imgw, cfg.BUF_LEN + 80);

    // Only going to be down direction
    uint nouse;
    std::vector<cv::Rect> crossed_counting_line_objs_AB, crossed_counting_line_objs_BA;
    pvt->process(temp_3c, os.getAllObjects(), &nouse, count, fn, &crossed_counting_line_objs_AB, &crossed_counting_line_objs_BA);
    drawBlinker(&crossed_counting_line_objs_BA);

    DBSHOW_POS("marked1", temp_3c, imgw * 2, cfg.BUF_LEN + 80);
    
    cv::Mat *pCanvas = framebuffer.front();
    std::stringstream msg;
    msg << *count;
    cv::putText(*pCanvas, msg.str(), cv::Point(10, 15), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,255,0), 2);

    DBSHOW_POS("blinker", *pCanvas, imgw * 3, 0);
    cout << "Count: " << *count << endl;
}

void LBlinker::drawBlinker(std::vector<cv::Rect> *crossed_line_objs)
{
    if (crossed_line_objs->empty())
    {
        return;
    }
    cv::Mat *pCanvas = framebuffer.front();
    //cv::Mat overlay(pCanvas->size(), pCanvas->type());
    for (std::vector<cv::Rect>::iterator it = crossed_line_objs->begin(); it != crossed_line_objs->end(); ++it)
    {
        cv::Rect r1(cfg.x1 + (*it).x, cfg.y - cfg.BUF_LEN/2 + (*it).y, (*it).width, (*it).height);
        cv::rectangle(*pCanvas, r1, cv::Scalar(0, 255, 0), 4);
    }
    //cv::addWeighted(*pCanvas, 0.7, overlay, 0.3, 0.0, *pCanvas);
}

// cache the current frame into framebuffer
void LBlinker::cacheFrame(cv::InputArray frame, int delay)
{
    cv::Mat im = frame.getMat();
    int framebuffer_len = delay + 1;
    if (framebuffer.size() < framebuffer_len)
    {
        cv::Mat *p = new cv::Mat(frame.size(), frame.type());
        im.copyTo(*p);
        framebuffer.push_back(p);
    }
    else
    {
        // We can reuse the buffer at framebuffer.front
        cv::Mat *p = framebuffer.front();
        framebuffer.pop_front();
        im.copyTo(*p);
        framebuffer.push_back(p);
    }
}

void LBlinker::cacheLine(cv::Mat *pLine)
{
    cv::Mat *pRow = new cv::Mat(1, pLine->cols, pLine->type());
    pLine->copyTo(*pRow);
    if (pl.size() >= cfg.BUF_LEN)
    {
        cv::Mat *ptmp = pl.back();
        //cout << "POP1: " << hex << (uint64)ptmp << endl;
        if (ptmp)
        {
            delete ptmp;
        }
        pl.pop_back();
    }
    pl.push_front(pRow);
}

void LBlinker::cacheLine()
{
    cv::Mat temptemp;
    if (pframe->type() == CV_8UC3)
    {
        std::vector<cv::Mat> channels;
        split(*pframe, channels);
        cv::Mat ttt[] = {channels[2], channels[1], channels[2]};
        cv::merge(ttt, 3, temptemp);
    }
    else
    {
        // CV_8UC1
        temptemp = *pframe;
    }
    cache_line_type = pframe->type();
    
    int yyy1 = cfg.y - cfg.lines_per_frame/2;
    for (int i = 0; i < cfg.lines_per_frame; i++)
    {
        cv::Mat ll = temptemp.row(yyy1 + i).colRange(line_offset, line_offset + line_len);
        cacheLine(&ll);
    /*
        cv::Mat *pRow = new cv::Mat(1, line_len, temptemp.type());
        temptemp.row(yyy1 + i).colRange(line_offset, line_offset + line_len).copyTo(*pRow);
        if (pl.size() >= cfg.BUF_LEN)
        {
            cv::Mat *ptmp = pl.back();
            //cout << "POP1: " << hex << (uint64)ptmp << endl;
            if (ptmp)
            {
                delete ptmp;
            }
            pl.pop_back();
        }
        pl.push_front(pRow);
     */
    }
}

void LBlinker::cacheLine(cv::InputArray frame)
{
    cv::Mat temptemp;
    if (frame.type() == CV_8UC3)
    {
        std::vector<cv::Mat> channels;
        split(frame, channels);
        cv::Mat ttt[] = {channels[2], channels[1], channels[2]};
        cv::merge(ttt, 3, temptemp);
    }
    else
    {
        // CV_8UC1
        temptemp = frame.getMat();
    }
    cache_line_type = frame.type();
    
    int yyy1 = cfg.y - cfg.lines_per_frame/2;
    for (int i = 0; i < cfg.lines_per_frame; i++)
    {
        cv::Mat ll = temptemp.row(yyy1 + i).colRange(line_offset, line_offset + line_len);
        cacheLine(&ll);
        /*
        cv::Mat *pRow = new cv::Mat(1, line_len, temptemp.type());
        temptemp.row(yyy1 + i).colRange(line_offset, line_offset + line_len).copyTo(*pRow);
        if (pl.size() >= cfg.BUF_LEN)
        {
            cv::Mat *ptmp = pl.back();
            //cout << "POP1: " << hex << (uint64)ptmp << endl;
            if (ptmp)
            {
                delete ptmp;
            }
            pl.pop_back();
        }
        pl.push_front(pRow);
         */
    }
}

void LBlinker::getPicture(cv::OutputArray pic)
{
    if (pframe == nullptr)
    {
        return;
    }
    if (pic.needed())
    {
        //cout << "Creating " << dec << cfg.BUF_LEN << "x" << line_len << " Type:" << cache_line_type << " Esize:" << element_size << endl;
        pic.create(cfg.BUF_LEN, line_len, cache_line_type);
    }
    
    cv::Mat picm = pic.getMat();
    picm.setTo(cv::Scalar(0, 0, 0));
    
    int rows_copied = 0;
    for (std::deque<cv::Mat *>::iterator it = pl.begin(); (it != pl.end()) && (rows_copied < cfg.BUF_LEN); ++it)
    {
        //cout << "COPY OUT: " << hex << (uint64)(*it) << endl;
        (*it)->copyTo(picm.row(rows_copied));
        /*
        unsigned char *p = picm.ptr(rows_copied);
        memcpy(p, *it, line_len * element_size);
         */
        rows_copied ++;
    }
}

// Head light is small so we cache 3 lines each frame
void LBlinkerHeadLight::cacheLine(cv::InputArray frame)
{
    cv::Mat temptemp;
    if (frame.type() == CV_8UC3)
    {
        std::vector<cv::Mat> channels;
        split(frame, channels);
        cv::Mat ttt[] = {channels[2], channels[1], channels[2]};
        cv::merge(ttt, 3, temptemp);
    }
    else
    {
        // CV_8UC1
        temptemp = frame.getMat();
    }
    cache_line_type = frame.type();
    
    int yyy1 = cfg.y - cfg.lines_per_frame/2;
    for (int i = 0; i < cfg.lines_per_frame; i++)
    {
        cv::Mat *pRow = new cv::Mat(1, line_len, temptemp.type());
        temptemp.row(yyy1 + i).colRange(line_offset, line_offset + line_len).copyTo(*pRow);
        if (pl.size() >= cfg.BUF_LEN)
        {
            cv::Mat *ptmp = pl.back();
            //cout << "POP1: " << hex << (uint64)ptmp << endl;
            if (ptmp)
            {
                delete ptmp;
            }
            pl.pop_back();
        }
        pl.push_front(pRow);
    }
}

void LBlinkerHeadLight::process(cv::InputArray frame, uint *count, uint64 fn)
{
    cv::Mat im = frame.getMat();
    pframe = &im;
    
    if (first_run)
    {
        // First time
        initialize();
        first_run = false;
    }
    frame_num = fn;
    
    CV_Assert(imgw == im.cols);
    CV_Assert(imgh == im.rows);
    
    std::vector<cv::Mat> channels;
    split(frame, channels);
    
    DBSHOW("B", channels[0]);
    DBSHOW("G", channels[1]);
    DBSHOW("R", channels[2]);
    
    // Get a mask marking out the brightest pixels
    double minVal;
    double maxVal;
    Point minLoc;
    Point maxLoc;
    cv::Mat temp_1c;
    minMaxLoc(channels[2], &minVal, &maxVal, &minLoc, &maxLoc);

    DBOUT(std::cout << "Max: " << maxVal << std::endl);

    threshold(channels[2], temp_1c, maxVal-1, 255, THRESH_BINARY);
    DBSHOW("brightest", temp_1c);
    
    // Blur
    GaussianBlur( temp_1c, temp_1c, Size(11, 11), 2, 2 );
    threshold(temp_1c, temp_1c, 20, 255, THRESH_BINARY);
    
    // Some cars have dimm light with brightness below 254, so we use edge detection to merge those in
    cv::Mat edmax;
    ped->process(channels[2], edmax, fn);
    
    temp_1c |= edmax;
    DBSHOW("blured", temp_1c);

    Mat morphKernel = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(10, 10));
    cv::morphologyEx(temp_1c, temp_1c, cv::MORPH_OPEN, morphKernel);
    DBSHOW("fore", temp_1c);
    
    cacheLine(temp_1c);
    cacheFrame(frame, cfg.BUF_LEN/2/cfg.lines_per_frame);
    drawLine();
    
    cv::Mat fore;
    getPicture(ttemp);
    DBSHOW_POS("lines", ttemp, 0, 0);

    morphKernel = getStructuringElement(cv::MORPH_RECT, cv::Size(2, 10));
    cv::morphologyEx(ttemp, ttemp, cv::MORPH_DILATE, morphKernel);
    
    vm.process(ttemp, fore, fn);

    // Vehicle length here is tricky since headlight isn't that big. But some truck does have multiple lights on its head which we
    // want to avoid detecting as multiple vehicles
    // These params are also related to how many lines are cached in each frame.
//    vm.setVehicleParams(cfg.rrate, cfg.aspeed, cfg.vwidth, cfg.vlen, cfg.lwidth, narrowest_lane_width,
//                        cfg.box_variation_ratio_h, cfg.box_variation_ratio_w, cfg.lines_per_frame);
    cv::Mat ttt[] = {fore, fore, fore};
    cv::merge(ttt, 3, temp_3c);

    if (os.process(fore, fn))
    {
        os.drawObjects(temp_3c, fn);
    }
    
    DBSHOW_POS("merged", temp_3c, imgw, 0);
    
    // Only going to be down direction
    uint nouse;
    std::vector<cv::Rect> crossed_counting_line_objs_AB, crossed_counting_line_objs_BA;
    pvt->process(temp_3c, os.getAllObjects(), &nouse, count, fn, &crossed_counting_line_objs_AB, &crossed_counting_line_objs_BA);
    drawBlinker(&crossed_counting_line_objs_BA);
    DBSHOW_POS("marked1", temp_3c, 0, cfg.BUF_LEN + 60);
    
    cv::Mat *pCanvas = framebuffer.front();
    std::stringstream msg;
    msg << *count;
    cv::putText(*pCanvas, msg.str(), cv::Point(10, 15), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,255,0), 2);
    
    cv::imshow("blinker", *pCanvas);
    cout << "Count: " << *count << endl;
}

void LBlinkerDay::initialize()
{
    average_gray_value = 0.0;
    average_gray_value_samples = 0;
    LBlinker::initialize();
    moving_average_window_len = cfg.lines_per_frame * 10 * cfg.rrate; // moving average window of 10s
}

// cache the stabilized lines to fight the camera flicker
#define PIXEL_AVERAGE(r, g, b) (((r)+(g)+(b))/3)
#define PIXEL_SQUARE_DIFF(x, a) (((x)-(a))*((x)-(a)))
#define PIXEL_VARIANCE(r, g, b) (PIXEL_SQUARE_DIFF(r, PIXEL_AVERAGE(r, g, b)) + PIXEL_SQUARE_DIFF(g, PIXEL_AVERAGE(r, g, b)) + PIXEL_SQUARE_DIFF(b, PIXEL_AVERAGE(r, g, b)))
void LBlinkerDay::cacheStabilizeLine(cv::InputArray frame)
{
    CV_Assert(frame.type() == CV_8UC3);
    
    cv::Mat im = frame.getMat();
    cache_line_type = frame.type();
    
    // 1. find gray values and get an average
    int yyy1 = cfg.y - cfg.lines_per_frame/2;
    for (int i = 0; i < cfg.lines_per_frame; i++)
    {
        double line_gray_average = 0;
        int line_gray_average_samples = 0;
        cv::Mat line = im.row(i + yyy1).colRange(line_offset, line_offset + line_len);
        cv::Mat adjusted;
        
        for (int j = line_offset; j < line_offset + line_len; j++)
        {
            float r, g, b;
            b = im.data[im.step[0]*(yyy1 + i) + im.step[1]*j + 0];
            g = im.data[im.step[0]*(yyy1 + i) + im.step[1]*j + 1];
            r = im.data[im.step[0]*(yyy1 + i) + im.step[1]*j + 2];
            double var = PIXEL_VARIANCE(r, g, b);
            double average = PIXEL_AVERAGE(r, g, b);
            if ((average > 20) && (average < 100) && (var < 10))
            {
                // We think this is gray
                line_gray_average = (line_gray_average * line_gray_average_samples + average)/(line_gray_average_samples + 1);
                line_gray_average_samples ++;
            }
        }
        
        if (average_gray_value_samples == 0)
        {
            average_gray_value = line_gray_average;
            average_gray_value_samples++;
            adjusted = line;
        }
        else if (abs(line_gray_average - average_gray_value) < ((double)cfg.bg_thresh))
        {
            if (average_gray_value_samples < moving_average_window_len)
            {
                average_gray_value = (average_gray_value * average_gray_value_samples + line_gray_average)/(average_gray_value_samples + 1);
                average_gray_value_samples++;
            }
            else
            {
                average_gray_value -= average_gray_value/moving_average_window_len;
                average_gray_value += line_gray_average/moving_average_window_len;
            }
            double ratio = average_gray_value/line_gray_average;
            adjusted = line * ratio;
        }
        else
        {
            double ratio = average_gray_value/line_gray_average;
            adjusted = line * ratio;
        }
        
        cacheLine(&adjusted);
        DBOUT(cout << "LA: " << line_gray_average << " A:" << average_gray_value << endl);
    }
}

void LBlinkerDay::process(cv::InputArray frame, uint *count, uint64 fn)
{
    CV_Assert((frame.type() == CV_8UC3) || (frame.type() == CV_8UC1));
    
    cv::Mat im = frame.getMat();
    pframe = &im;
    
    if (first_run)
    {
        // First time
        initialize();
        first_run = false;
    }
    frame_num = fn;
    
    CV_Assert(imgw == im.cols);
    CV_Assert(imgh == im.rows);
    setupLine();
    cacheStabilizeLine(frame);
    cacheFrame(frame, cfg.BUF_LEN/2/cfg.lines_per_frame);
    drawLine();
    
    cv::Mat lines, lines_contour;
    getPicture(lines);
    DBSHOW("lines", lines);

    cv::Mat fore, back;
    pbg->operator()(lines, fore);
    
    cv::medianBlur(fore, lines_contour, 7);
    
    cv::Mat morphKernel = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15));
    cv::morphologyEx(lines_contour, ttemp, cv::MORPH_ERODE, morphKernel);
    
    DBSHOW("fore", fore);
    DBSHOW("blur", lines_contour);
    DBSHOW("erode", ttemp);
    
    vm.process(ttemp, fore, fn);
    cv::Mat ttt[] = {fore, fore, fore};
    cv::merge(ttt, 3, temp_3c);
    if (os.process(fore, fn))
    {
        os.drawObjects(temp_3c, fn);
    }
    
    DBSHOW_POS("merged", temp_3c, imgw, cfg.BUF_LEN + 60);
    
    // Only going to be down direction
    uint nouse;
    std::vector<cv::Rect> crossed_counting_line_objs_AB, crossed_counting_line_objs_BA;
    pvt->process(temp_3c, os.getAllObjects(), &nouse, count, fn, &crossed_counting_line_objs_AB, &crossed_counting_line_objs_BA);
    drawBlinker(&crossed_counting_line_objs_BA);
    DBSHOW_POS("marked1", temp_3c, imgw, 2 * cfg.BUF_LEN + 60);
    
    cv::Mat *pCanvas = framebuffer.front();
    std::stringstream msg;
    msg << *count;
    cv::putText(*pCanvas, msg.str(), cv::Point(10, 15), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,255,0), 2);
    
    cv::imshow("blinker", *pCanvas);
    cout << "Count: " << *count << endl;

}
