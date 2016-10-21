//
//  DayNonJamProcessor.cpp
//  DCVehicleCounter
//
//  Created by hohe on 8/26/14.
//  Copyright (c) 2014 DongCheng. All rights reserved.
//

#include "DayNonJamProcessor.h"

DayNonJamProcessor::DayNonJamProcessor()
{
    myid = IDS_DAY_NONJAM_PROCESSOR;
    pvt = nullptr;
    pbg = nullptr;
    splitter = nullptr;
    dstate = INIT;
    first_run = true;
    cfg.bg_history = 200;
    cfg.bg_threshold = 8;
    cfg.bg_shadow = true;
    cfg.erosion_size = 6;
    cfg.dilation_size = 4;
    cfg.blur_size = 11;
    cfg.min_area = 100;
    cfg.max_area = 15000;
}

DayNonJamProcessor::~DayNonJamProcessor()
{
    saveConfig();
    if (pvt)
    {
        delete pvt;
    }
    if (pbg)
    {
        delete pbg;
    }
    if (splitter)
    {
        delete splitter;
    }
}

void DayNonJamProcessor::initialize(cv::InputArray frame, uint64 fn)
{
    loadConfig();
    if (cfg.roi_defined == false)
    {
        setupROI(frame);
    }
    else
    {
        dstate = RUNNING;
    }
    
    cv::Mat im = frame.getMat();
    pbg = new BackgroundProcessor();
    pbg->setConfigParams(cfg.bg_history, cfg.bg_threshold, cfg.bg_shadow, cfg.erosion_size, cfg.dilation_size, cfg.blur_size);
    
    pvt = new VehicleTracking(im.cols, im.rows);
    pvt->setCountingLine(cfg.clh);
    
    splitter = new ObjectSplitterVehicleCounter();
    splitter->setMinMaxArea(cfg.min_area, cfg.max_area);
    splitter = new ObjectSplitterVehicleCounter();
}

void DayNonJamProcessor::process(cv::InputArray frame, cv::OutputArray oframe, uint64 fn, unsigned int *cntAB, unsigned int *cntBA)
{
    cv::Mat im = frame.getMat();
    pframe = &im;
    if (first_run)
    {
        initialize(frame, fn);
        first_run = false;
        oframe.create(frame.size(), frame.type());
        cout << "Frame size: " << im.cols << "x" << im.rows << endl;
    }

    cv::Mat om = oframe.getMat();
    im.copyTo(om);
    Mat fore, back, enhanced, enhanced_color;
    Mat raw_frame;
    
    {
#if _USE_PBAS_
        pbas.process(frame, fore);
#else
        pbg->process(frame, fn, fore, back);
        DBSHOW("Fore", fore);

        if (fn < 60)
        {
            // Give background processor some time to build up
            return;
        }
#endif
        
        if (splitter->process(fore, fn))
        {
            splitter->drawObjects(im, fn);
        }
        
        std::vector<cv::Rect> objsAB, objsBA;
        pvt->process(im, splitter->getAllObjects(), cntAB, cntBA, fn, &objsAB, &objsBA);
        cout << "CountAB: " << *cntAB << " CountBA: " << *cntBA << endl;
        drawCountedVehicles(im, &objsAB, &objsBA);
        drawROI(im);
    }
    
    return;
}

void DayNonJamProcessor::drawCountedVehicles(cv::InputOutputArray frame, std::vector<cv::Rect> *pAB, std::vector<cv::Rect> *pBA)
{
    cv::Mat im = frame.getMat();
    int i;
    
    for (i = 0; i < pAB->size(); i++)
    {
        cv::rectangle(im, (*pAB)[i], cv::Scalar(0, 0, 255), 3);
    }
    
    for (i = 0; i < pBA->size(); i++)
    {
        cv::rectangle(im, (*pBA)[i], cv::Scalar(0, 255, 0), 3);
    }
}

void DayNonJamProcessor::saveConfig()
{
    std::string config_file = getConfigFileName();
    cv::FileStorage fs(config_file, cv::FileStorage::WRITE);
    if (fs.isOpened() == false)
    {
        ERR(std::cerr << "Failed to open " << config_file << " to save config" << std::endl);
        return;
    }
 
    fs << "BGH" << cfg.bg_history;
    fs << "BGT" << cfg.bg_threshold;
    fs << "BGS" << cfg.bg_shadow;
    fs << "MIA" << cfg.min_area;
    fs << "MAA" << cfg.max_area;
    fs << "BLS" << cfg.blur_size;
    fs << "DIS" << cfg.dilation_size;
    fs << "ERS" << cfg.erosion_size;
    fs << "ROI" << cfg.roi_defined;
    fs << "X11" << cfg.x11;
    fs << "X12" << cfg.x12;
    fs << "X21" << cfg.x21;
    fs << "X22" << cfg.x22;
    fs << "Y11" << cfg.y11;
    fs << "Y12" << cfg.y12;
    fs << "Y21" << cfg.y21;
    fs << "Y22" << cfg.y22;
    fs << "CLH" << cfg.clh;
    
    fs.release();
}

void DayNonJamProcessor::loadConfig()
{
    std::string config_file = getConfigFileName();
    cv::FileStorage fs(config_file, cv::FileStorage::READ);
    if (fs.isOpened() == false)
    {
        ERR(std::cerr << "Failed to open " << config_file << " file to load config" << std::endl);
        return;
    }
    
    cfg.bg_history = fs["BGH"];
    cfg.bg_threshold = fs["BGT"];
    cfg.bg_shadow = (int)fs["BGS"];
    cfg.min_area = fs["MIA"];
    cfg.max_area = fs["MAA"];
    cfg.erosion_size = fs["ERS"];
    cfg.dilation_size = fs["DIS"];
    cfg.blur_size = fs["BLS"];
    cfg.roi_defined = (int)fs["ROI"];
    cfg.x11 = fs["X11"];
    cfg.x12 = fs["X12"];
    cfg.x21 = fs["X21"];
    cfg.x22 = fs["X22"];
    cfg.y11 = fs["Y11"];
    cfg.y12 = fs["Y12"];
    cfg.y21 = fs["Y21"];
    cfg.y22 = fs["Y22"];
    fs["CLH"] >> cfg.clh;
    
    fs.release();
}

void DayNonJamProcessor::setupROI(cv::InputArray frame)
{
    cv::Mat imat = frame.getMat();
    if (cfg.roi_defined == false)
    {
        cfg.clh = cv::Mat(1, imat.cols, CV_32S, Scalar(-1));
        do
        {
            drawROI(imat);
            cvSetMouseCallback("ROI", &mouser, this);
            cvWaitKey(0);
            
            if(cfg.roi_defined)
            {
                DBOUT(std::cout << "Counting line defined" << std::endl);
                break;
            }
            else
            {
                ERR(std::cout << "Counting line undefined!" << std::endl);
            }
        } while(1);
    }
    
    if (cfg.roi_defined == true)
    {
        cvDestroyWindow("ROI");
        saveConfig();
    }
}

void DayNonJamProcessor::drawROI(cv::InputOutputArray frame)
{
    cv::Mat im = frame.getMat();
    switch (dstate)
    {
        case INIT:
        case LINE:
            cv::putText(im, "Drag to define the counting line", cv::Point(10, 100), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 255));
            break;
        case P11:
            cv::putText(im, "Left click to define the upper left point", cv::Point(10, 100), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 255));
            break;
        case P12:
            cv::putText(im, "Left click to define the upper right point", cv::Point(10, 100), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 255));
            break;
        case P21:
            cv::putText(im, "Left click to define the lower left point", cv::Point(10, 100), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 255));
            break;
        case P22:
            cv::putText(im, "Left click to define the lower right point", cv::Point(10, 100), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 255));
            break;
        case DONE:
            cv::putText(im, "DONE", cv::Point(10, 100), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 255));
        case RUNNING:
        default:
            break;
    }
    
    switch (dstate)
    {
        case INIT:
            break;
        default:
            break;
        case RUNNING:
        case DONE:
        case P22:
            cv::circle(im, cv::Point(cfg.x22, cfg.y22), 2, cv::Scalar(0, 255, 0));
            
            cv::line(im, cv::Point(cfg.x21, cfg.y21), cv::Point(1, im.rows - 10), cv::Scalar(0, 255, 0));
            cv::line(im, cv::Point(1, im.rows - 10), cv::Point(im.cols - 1, im.rows - 10), cv::Scalar(0, 255, 0));
            cv::line(im, cv::Point(cfg.x22, cfg.y22), cv::Point(im.cols - 1, im.rows - 10), cv::Scalar(0, 255, 0));
            cv::line(im, cv::Point(cfg.x22, cfg.y22), cv::Point(cfg.x12, cfg.y12), cv::Scalar(0, 255, 0));
            
        case P21:
            cv::circle(im, cv::Point(cfg.x21, cfg.y21), 2, cv::Scalar(0, 255, 0));
            cv::line(im, cv::Point(cfg.x11, cfg.y11), cv::Point(cfg.x21, cfg.y21), cv::Scalar(0, 255, 0));
        case P12:
            cv::circle(im, cv::Point(cfg.x12, cfg.y12), 2, cv::Scalar(0, 255, 0));
            cv::line(im, cv::Point(cfg.x11, cfg.y11), cv::Point(cfg.x12, cfg.y12), cv::Scalar(0, 255, 0));
        case P11:
            cv::circle(im, cv::Point(cfg.x11, cfg.y11), 2, cv::Scalar(0, 255, 0));
        case LINE:
#if DEBUG
            for (int i = 0; i < cfg.clh.cols; i++)
            {
                if ((cfg.clh.at<int>(i) > 0) && (cfg.clh.at<int>(i) < pframe->rows))
                {
                    cv::circle(im, cv::Point(i, cfg.clh.at<int>(i)), 2, cv::Scalar(255, 0, 0));
                }
            }
#endif
            break;
    }
    
    cv::imshow("ROI", im);
}

void DayNonJamProcessor::mouser(int evt, int x, int y, int flag, void* this_)
{
    static_cast<DayNonJamProcessor*>(this_)->onMouse(evt, x, y, flag);
}

void DayNonJamProcessor::onMouse(int evt, int x, int y, int flag)
{
    cv::Mat canvas = pframe->clone();
    switch (dstate)
    {
        case INIT:
            if (cfg.roi_defined)
            {
                dstate = RUNNING;
            }
            else if (evt == CV_EVENT_LBUTTONDOWN)
            {
                dstate = LINE;
            }
            break;
        case LINE:
            if (evt == CV_EVENT_MOUSEMOVE)
            {
                cfg.clh.at<int>(0, x) = y;
                DBOUT(std::cout << "LINE: " << x << ", " << y << std::endl);
            }
            if ((evt == CV_EVENT_LBUTTONUP) || (evt == CV_EVENT_LBUTTONDOWN))
            {
                dstate = P11;
                DBOUT(std::cout << "To P11" << std::endl);
            }
            break;
        case P11:
            if (evt == CV_EVENT_LBUTTONDOWN)
            {
                cfg.x11 = x;
                cfg.y11 = y;
                dstate = P12;
                DBOUT(std::cout << "P11: " << cfg.x11 << ", " << cfg.y11 << std::endl);
            }
            break;
        case P12:
            if (evt == CV_EVENT_LBUTTONDOWN)
            {
                cfg.x12 = x;
                cfg.y12 = y;
                dstate = P21;
                DBOUT(std::cout << "P12: " << cfg.x12 << ", " << cfg.y12 << std::endl);
            }
            break;
        case P21:
            if (evt == CV_EVENT_LBUTTONDOWN)
            {
                cfg.x21 = x;
                cfg.y21 = y;
                dstate = P22;
                DBOUT(std::cout << "P21: " << cfg.x21 << ", " << cfg.y21 << std::endl);
            }
            break;
        case P22:
            if (evt == CV_EVENT_LBUTTONDOWN)
            {
                cfg.x22 = x;
                cfg.y22 = y;
                dstate = DONE;
                DBOUT(std::cout << "P22: " << cfg.x22 << ", " << cfg.y22 << std::endl);
            }
            break;
        case DONE:
            dstate = RUNNING;
            cfg.roi_defined = true;
        case RUNNING:
        default:
            break;
    }
    drawROI(canvas);
}

void DayNonJamProcessor::drawROI()
{
    drawROI(*pframe);
}


