//
//  BackgroundProcessor.cpp
//  DCVehicleCounter
//
//  Created by hohe on 7/14/14.
//  Copyright (c) 2014 DongCheng. All rights reserved.
//

#include "BackgroundProcessor.h"

BackgroundProcessor::BackgroundProcessor()
{
    // TODO: better understand and use of the parameters
    //          http://docs.opencv.org/modules/video/doc/motion_analysis_and_object_tracking.html#backgroundsubtractor-operator
    pbg = new BackgroundSubtractorMOG2(MOG2_HISTORY, MOG2_THRESH, true);
    frame_number = 0;
    background = nullptr;
    smask = nullptr;
    first_run = true;
    DBOUT(std::cout << "MOG2 initialized: " << MOG2_HISTORY << ", " << MOG2_THRESH << ", shadow detection" << std::endl);
}

BackgroundProcessorNight::BackgroundProcessorNight()
{
    ped = new EdgeDetector(150, 450);
    // over write some of the BackgroundProcessor settings
    pbg->set("detectShadows", false);
    pbg->set("history", MOG2_HISTORY);
    pbg->set("varThreshold", MOG2_THRESH);
    
    pbg_edge = new BackgroundSubtractorMOG2(MOG2_HISTORY, MOG2_THRESH, false); // No shadows at night
    ped_back = new EdgeDetector(50, 150);

    DBOUT(std::cout << "MOG2 initialized: " << MOG2_HISTORY << ", " << MOG2_THRESH << ", no shadow detection" << std::endl);
}

BackgroundProcessor::~BackgroundProcessor()
{
    if (pbg)
    {
        delete pbg;
        pbg = nullptr;
    }
    if (background)
    {
        delete background;
        background = nullptr;
    }
    if (smask)
    {
        delete smask;
        smask = nullptr;
    }
}

BackgroundProcessorNight::~BackgroundProcessorNight()
{
    if (ped)
    {
        delete ped;
    }
    if (ped_back)
    {
        delete ped_back;
    }
}

void BackgroundProcessor::setConfigParams(int bh, int bt, bool bs, int es, int ds, int bls)
{
    cfg.bg_history = bh;
    cfg.bg_threshold = bt;
    cfg.bg_shadow = bs;
    cfg.erosion_size = es;
    cfg.dilation_size = ds;
    cfg.blur_size = bls;
    
    pbg->set("detectShadows", cfg.bg_shadow);
    pbg->set("history", cfg.bg_history);
    pbg->set("varThreshold", cfg.bg_threshold);
}

bool BackgroundProcessor::process(InputArray frame, uint64 fn, OutputArray fore, OutputArray back)
{
    Mat im = frame.getMat();
    if (first_run)
    {
        frame_number = fn;
        initialize(frame.size(), im.type());
        first_run = false;
    }
    
    CV_Assert(frame.type() == background->type());
    
    pbg->operator()(frame, fore);
    // Threshold to 128 because shadows are marked with 127
    cv::threshold(fore, fore, 128, 255, cv::THRESH_BINARY);
    last_fore.copyTo(temp);
    
    Mat om = fore.getMat();
    om.copyTo(last_fore);
    
    // To eliminate per frame noise. Any noise that only shows in one frame should be eliminated by this
    om = om & temp;

    pbg->getBackgroundImage(back);
    DBSHOW("fore2", fore);

    // Median blur seems to be best for this kind of use
    //cv::medianBlur(fore, fore, cfg.blur_size);
    cv::blur(fore, fore, cv::Size(cfg.blur_size/2, cfg.blur_size));
    double smart_threshold = 255 / 2;
    cv::threshold(fore, fore, smart_threshold, 255, CV_THRESH_BINARY);
    cv::medianBlur(fore, fore, cfg.blur_size);

    DBSHOW("back", back);
    DBSHOW("fore1", fore);

/*
    // First erosion then dilation, to reduce noise
    Mat morphKernel = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(cfg.erosion_size, cfg.erosion_size));
    cv::morphologyEx(fore, fore, cv::MORPH_ERODE, morphKernel);
#if DEBUG
    imshow("fore0", fore);
#endif
    morphKernel = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(cfg.dilation_size, cfg.dilation_size));
    cv::morphologyEx(fore, fore, cv::MORPH_DILATE, morphKernel);

    // Calculate still mask
    calcStillMask(fore, fn);
    
    if (isCameraMoving(frame, fn))
    {
        // If camera is moving then the foreground won't be accurate.
        return false;
    }
    else
    {
        return true;
    }
*/
    return true;
}

bool BackgroundProcessor::initialize(Size s, int type)
{
    CV_Assert((background == nullptr) && (smask == nullptr));
    background = new Mat(s, type);
    smask = new Mat(s, CV_8UC1);
    accu.create(s, CV_32FC1);
    CV_Assert((background != nullptr) && (smask != nullptr));
    
    return false;
}

void BackgroundProcessor::calcStillMask(InputArray fore, uint64 fn)
{
    // Revert foremask, then accumulateWeighted, then cut a threshold.
    // Those areas that changes frequently (like on the roads), tend to have a lower value below the threshold
    cv::bitwise_not(fore, temp);
    accumulateWeighted(temp, accu, ACCUMULATE_WEIGHT);
    accu.convertTo(*smask, CV_8UC1);
    DBSHOW("smask", *smask);
}

bool BackgroundProcessor::isCameraMoving(InputArray frame, uint64 fn)
{
    // TODO: Do optical flow analysis on some anchor points in smask
    Mat im = frame.getMat();
    im.copyTo(last_frame);
    return false;
}

/*
static void showHistogram(const char *window_name, Mat &hist, int histSize)
{
    // Draw the histograms for B, G and R
    int hist_w = 512; int hist_h = 400;
    int bin_w = cvRound( (double) hist_w/histSize );
    
    cv::Mat histImage( hist_h, hist_w, CV_8UC1, Scalar(0));
    
    /// Normalize the result to [ 0, histImage.rows ]
    normalize(hist, hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    
    /// Draw for each channel
    for( int i = 1; i < histSize; i++ )
    {
        int x1, y1, x2, y2;
        x1 = bin_w * (i - 1);
        y1 = hist_h - hist.at<float>(i-1);
        x2 = bin_w * i;
        y2 = hist_h - hist.at<float>(i);
        rectangle(histImage, Point(x1, y1), Point(x2, 0), 200);
    }
    
    /// Display
    imshow(window_name, histImage);
}
*/
#if 0
bool BackgroundProcessorNight::process(InputArray frame, uint64 fn, OutputArray fore, OutputArray back)
{
    Mat im = frame.getMat();
    if (frame_number == 0)
    {
        frame_number = fn;
        initialize(frame.size(), im.type());
    }
    
    CV_Assert(frame.type() == background->type());
    
    std::vector<cv::Mat> channels;
    split(frame, channels);
    
#if 0
    imshow("B", channels[0]);
    imshow("G", channels[1]);
    imshow("R", channels[2]);
#endif

    //cvtColor(frame, temp_1c, CV_RGB2GRAY);
    // Red channel is least affected by headlight beams.
    temp_1c = channels[2];
    
#if DEBUG
    imshow("Red", temp_1c);
#endif
    ped->process(temp_1c, edge, fn);
    
#if DEBUG
//    imshow("edge", edge);
#endif
    
/*
    edge = edge & (~lastlast_edge);
    edge = edge & (~last_edge);
    
#if DEBUG
    imshow("edgediff", edge);
#endif
    
    last_edge.copyTo(lastlast_edge);
    edge.copyTo(last_edge);

    {
        // Get a mask marking out the brightest pixels
        double minVal;
        double maxVal;
        Point minLoc;
        Point maxLoc;
        minMaxLoc(temp_1c, &minVal, &maxVal, &minLoc, &maxLoc);
        std::cout << "Max: " << maxVal << std::endl;
        threshold(temp_1c, temp_1c, maxVal-20, 255, THRESH_BINARY);
#if DEBUG
        imshow("brightest", temp_1c);
#endif
    }
*/
    cv::Mat ttt[] = {edge, edge, edge};
    cv::merge(ttt, 3, temp_3c);
    //temp_3c = edge;
    pbg->operator()(temp_3c, fore); // We don't want to use the fore here, instead we want to only get headlights out of the image
    
#if DEBUG
    imshow("bog2fore", fore);
//    imshow("pbasfore", temp_1c);
#endif

    pbg->getBackgroundImage(back);
/*
    // Cancel edges that are also in background's edges. Not quite useful, due to dynamic range too big and edges too thin.
    // Same edge could swing between ajacent pixels and fail the canceling
    ped_back->process(back, edge_back, fn);
    edge = edge & (~edge_back);

#if DEBUG
    imshow("fore_edge", edge);
#endif
*/
    
    Mat mfore = fore.getMat();
    Mat mback = back.getMat();
#if DEBUG
    imshow("Back", back);
#endif
    
    Mat morphKernel = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(8, 8));
    cv::morphologyEx(fore, fore, cv::MORPH_DILATE, morphKernel);
    
    last_fore.copyTo(temp);
    mfore.copyTo(last_fore);
    mfore = mfore & temp;
    
#if DEBUG
    imshow("fore0", fore);
#endif
/*
    temp_3c = im - mback;
#if DEBUG
    imshow("diff", temp_3c);
#endif
    cv::cvtColor(temp_3c, foreground, CV_BGR2GRAY);
    
#if DEBUG
    imshow("diffgray", foreground);
#endif
*/
    
/*
    Mat hist;
    int histSize = 256;
    float range[] = { 0, 256 } ;
    const float* histRange = { range };
    calcHist(&foreground, 1, 0, Mat(), hist, 1, &histSize, &histRange, true, false);

    showHistogram("Hist", hist, histSize);
    
    // Get rid of dark noise
    //threshold(foreground, foreground, 200, 255, THRESH_BINARY);
    
    // Equalize
//    cv::equalizeHist(foreground, foreground);

    
    // Get rid of brightest pixels from foreground
    mfore = mfore & (~temp_1c);
*/
 
/*
    
    last_fore.copyTo(temp);
    mfore.copyTo(last_fore);
    
    // To eliminate per frame noise. Any noise that only shows in one frame should be eliminated by this
    mfore = mfore & temp;
    
#if DEBUG
    imshow("fore2", fore);
#endif
    // Median blur seems to be best for this kind of use
    cv::medianBlur(fore, fore, BLUR_KERNEL_SIZE);
    
#if DEBUG
    imshow("fore1", fore);
#endif
*/
    /*
     // Opening with a 3x9 to try to cut horizontal links between different vehicles while maintain vertical foreground pixels
     Mat morphKernel = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(BLUR_KERNEL_SIZE+4, BLUR_KERNEL_SIZE+4));
     cv::morphologyEx(fore, fore, cv::MORPH_OPEN, morphKernel);
     */
    
/*
    // First erosion then dilation, to reduce noise
    Mat morphKernel = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(BLUR_KERNEL_SIZE, BLUR_KERNEL_SIZE));
    cv::morphologyEx(fore, fore, cv::MORPH_ERODE, morphKernel);
#if DEBUG
    imshow("fore0", fore);
#endif
    morphKernel = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(BLUR_KERNEL_SIZE, BLUR_KERNEL_SIZE));
    cv::morphologyEx(fore, fore, cv::MORPH_DILATE, morphKernel);
*/
    /*
    // And edge vs fore
    edge = edge & mfore;
#if DEBUG
    imshow("leftover_edge", edge);
#endif
     */
    /*
     // Calculate still mask
     calcStillMask(fore, fn);
     
     if (isCameraMoving(frame, fn))
     {
     // If camera is moving then the foreground won't be accurate.
     return false;
     }
     else
     {
     return true;
     }
     */
    return true;
}
#else
// This is to get fore ground first, then detect edge in it.
bool BackgroundProcessorNight::process(InputArray frame, uint64 fn, OutputArray fore, OutputArray back)
{
    Mat im = frame.getMat();
    if (frame_number == 0)
    {
        frame_number = fn;
        initialize(frame.size(), im.type());
    }
    
    CV_Assert(frame.type() == background->type());
    cv::GaussianBlur(frame, temp_3c, cv::Size(3,3), 0, 0, cv::BORDER_REPLICATE );
    
    {
        int ddepth = CV_16S;
        int scale = 1;
        int delta = 0;
        cv::Mat grad;
        cv::Mat grad_x, grad_y;
        cv::Mat abs_grad_x, abs_grad_y;

        cv::Sobel(frame, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
        cv::convertScaleAbs( grad_x, abs_grad_x );
        cv::Sobel(frame, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
        cv::convertScaleAbs( grad_y, abs_grad_y );
        /// Total Gradient (approximate)
        cv::addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );
        DBSHOW("sobel", grad );
    }
    
    /*
    // Only use red channel, it has least noise and least affected by blueish headlights
    std::vector<cv::Mat> channels;
    split(frame, channels);
    cv::Mat ttt[] = {channels[2], channels[2], channels[2]};
    cv::merge(ttt, 3, temp_3c);
    */
    pbg->operator()(temp_3c, fore);
    pbg->getBackgroundImage(back);
    
    Mat mback = back.getMat();
    Mat mfore;// = temp_3c - mback;
    
    cv::absdiff(temp_3c, mback, mfore);
    DBSHOW("diff", mfore);
    DBSHOW("back", back);
    DBSHOW("fore", fore);
    DBSHOW("blur", temp_3c);
    
    cv::cvtColor(mfore, temp_1c, CV_BGR2GRAY);
    ped->process(temp_1c, edge, fn);
    
    Mat morphKernel = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(8, 8));
    cv::morphologyEx(edge, fore, cv::MORPH_DILATE, morphKernel);

    return true;
}
#endif
