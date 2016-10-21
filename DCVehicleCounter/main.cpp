//
//  main.cpp
//  DCVehicleCounter
//
//  Created by hohe on 7/14/14.
//  Copyright (c) 2014 DongCheng. All rights reserved.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cvaux.hpp>
#include "BackgroundProcessor.h"
#include "ImageEnhancer.h"
#include "pt/PixelBasedAdaptiveSegmenter.h"
#include "Resizer.h"
#include "VehicleTracking.h"
#include "ObjectSplitterVehicleCounter.h"
#include "Blinker.h"
#include "DayNonJamProcessor.h"

using namespace cv;
using namespace std;

#define _USE_RESIZE_ 1
#define _USE_PBAS_ 0

#define CLEAR_NIGHT 1
#define DAY_MEDIUM_TRAFFIC 2
#define FOUR_LANE_NIGHT 3
#define DAY_MEDIUM_TRAFFIC_BLINK 4

//#define RUN_MODE DAY_MEDIUM_TRAFFIC
#define RUN_MODE DAY_MEDIUM_TRAFFIC_BLINK
//#define RUN_MODE CLEAR_NIGHT
//#define RUN_MODE FOUR_LANE_NIGHT

#if RUN_MODE==CLEAR_NIGHT
// main for ClearNight.mp4
int main(int argc, const char * argv[])
{
    VideoCapture *cap;
    LBlinker blbl;
    blbl.setMode(MODE_CLEAR_NIGHT);
    
    //cap = new VideoCapture("./Day_Medium_traffic.3gp");
    //cap = new VideoCapture("/Volumes/DATA/work/workplace/DCVehicleCounter/Resources/day-jam.3gp");
    cap = new VideoCapture("/Volumes/DATA/work/workplace/DCVehicleCounter/Resources/ClearNight.mp4");
    //cap = new VideoCapture("/Volumes/DATA/work/workplace/DCVehicleCounter/Resources/four_lane_night.mp4");
    //cap = new VideoCapture("/Volumes/DATA/work/workplace/DCVehicleCounter/Resources/four_lane_day_many_trucks.mp4");
    //cap = new VideoCapture("./Day_Medium_traffic.3gp");
    
    //VideoCapture cap("/Volumes/DATA/work/workplace/DCVehicleCounter/Resources/2014-7-10_nanpudaqiao.mp4");
    //VideoCapture cap("/Volumes/DATA/work/workplace/DCVehicleCounter/Resources/ClearNight.mp4");
    //VideoCapture cap("/Volumes/DATA/work/workplace/DCVehicleCounter/Resources/Rain_Night.mp4");
    //VideoCapture cap("/Volumes/DATA/work/workplace/DCVehicleCounter/Resources/four_lane_day_many_trucks.mp4");
    
    //ImageEnhancer ie;
    
#if _USE_RESIZE_
    Resizer resizer(0.6);
#endif
    if (!cap->isOpened())
    {
        ERR(cout << "Can't open video file" << endl);
        return -1;
    }
    
    //cap.set(CV_CAP_PROP_POS_MSEC, 12000); // start the video at 300ms
    
    double fps = cap->get(CV_CAP_PROP_FPS);
    cout << "Frame per second: " << fps << endl;
    
    uint64 framenumber = 0;
    uint cAB, cBA;
    Mat fore, back, enhanced, enhanced_color;
    Mat frame, raw_frame;
    while (1)
    {
        bool bSuccess = cap->read(raw_frame);
        
        if (!bSuccess)
        {
            ERR(cout << "Failed to read the frame from video file" << endl);
            break;
        }
        
#if _USE_RESIZE_
        // Resize to 1/4 of original size to speed up
        resizer.resize(raw_frame, frame);
#else
        frame = raw_frame;
#endif
        //imshow("Raw", frame);
        
        framenumber++;
        
        cout << "==Frame " << framenumber << endl;
        blbl.process(frame, &cAB, framenumber);
        if (waitKey(30) == 27) // wait for 'esc' key press from 30 ms. If 'esc' key is pressed, break loop
        {
            cout << "ESC" << endl;
            break;
        }
    }
    if (cap) delete cap;
    return 0;
}

#elif RUN_MODE==DAY_MEDIUM_TRAFFIC
// Main for Day_Medium_traffic.3gp
int main(int argc, const char * argv[])
{
    VideoCapture *cap;
    
    //cap = new VideoCapture("./Day_Medium_traffic.3gp");
    //cap = new VideoCapture("/Volumes/DATA/work/workplace/DCVehicleCounter/Resources/day-jam.3gp");
    //cap = new VideoCapture("./ClearNight.mp4");
    //cap = new VideoCapture("/Volumes/DATA/work/workplace/DCVehicleCounter/Resources/four_lane_night.mp4");
    //cap = new VideoCapture("/Volumes/DATA/work/workplace/DCVehicleCounter/Resources/four_lane_day_many_trucks.mp4");
    //cap = new VideoCapture("./Day_Medium_traffic.3gp");
    
    cap = new VideoCapture("./Day_Medium_traffic.3gp");
    
    //VideoCapture cap("/Volumes/DATA/work/workplace/DCVehicleCounter/Resources/2014-7-10_nanpudaqiao.mp4");
    //VideoCapture cap("/Volumes/DATA/work/workplace/DCVehicleCounter/Resources/ClearNight.mp4");
    //VideoCapture cap("/Volumes/DATA/work/workplace/DCVehicleCounter/Resources/Rain_Night.mp4");
    //VideoCapture cap("/Volumes/DATA/work/workplace/DCVehicleCounter/Resources/four_lane_day_many_trucks.mp4");
    
    //ImageEnhancer ie;
    
    DayNonJamProcessor dnp;
#if _USE_RESIZE_
    Resizer resizer(0.6);
#endif
    if (!cap->isOpened())
    {
        ERR(cout << "Can't open video file" << endl);
        return -1;
    }
    
    //cap.set(CV_CAP_PROP_POS_MSEC, 12000); // start the video at 300ms
    
    double fps = cap->get(CV_CAP_PROP_FPS);
    cout << "Frame per second: " << fps << endl;
    
    uint64 framenumber = 0;
    Mat fore, back, enhanced, enhanced_color;
    Mat frame, raw_frame, om;
    uint cntAB, cntBA;
    
    dnp.setMode(MODE_DAY_MEDIUM_TRAFFIC);
    while (1)
    {
        bool bSuccess = cap->read(raw_frame);
        
        if (!bSuccess)
        {
            ERR(cout << "Failed to read the frame from video file" << endl);
            break;
        }
        
#if _USE_RESIZE_
        // Resize to speed up
        resizer.resize(raw_frame, frame);
#else
        frame = raw_frame;
#endif
        //imshow("Raw", frame);
        dnp.process(frame, om, framenumber, &cntAB, &cntBA);
        framenumber++;
        
        cout << "==Frame " << framenumber << endl;
        cout << "CountAB: " << cntAB << " CountBA: " << cntBA << endl;
        
        if (waitKey(30) == 27) // wait for 'esc' key press from 30 ms. If 'esc' key is pressed, break loop
        {
            cout << "ESC" << endl;
            break;
        }
    }
    if (cap) delete cap;
    return 0;
}
#elif RUN_MODE==FOUR_LANE_NIGHT

int main(int argc, const char * argv[])
{
    VideoCapture *cap;
    LBlinkerHeadLight blbl(MODE_FOUR_LANES_NIGHT);
    
    //cap = new VideoCapture("./Day_Medium_traffic.3gp");
    //cap = new VideoCapture("/Volumes/DATA/work/workplace/DCVehicleCounter/Resources/day-jam.3gp");
    //cap = new VideoCapture("/Volumes/DATA/work/workplace/DCVehicleCounter/Resources/ClearNight.mp4");
    cap = new VideoCapture("/Volumes/DATA/work/workplace/DCVehicleCounter/Resources/four_lane_night.mp4");
    //cap = new VideoCapture("./four_lane_night.mp4");
    //cap = new VideoCapture("/Volumes/DATA/work/workplace/DCVehicleCounter/Resources/four_lane_day_many_trucks.mp4");
    //cap = new VideoCapture("./Day_Medium_traffic.3gp");
    
    //VideoCapture cap("/Volumes/DATA/work/workplace/DCVehicleCounter/Resources/2014-7-10_nanpudaqiao.mp4");
    //VideoCapture cap("/Volumes/DATA/work/workplace/DCVehicleCounter/Resources/ClearNight.mp4");
    //VideoCapture cap("/Volumes/DATA/work/workplace/DCVehicleCounter/Resources/Rain_Night.mp4");
    //VideoCapture cap("/Volumes/DATA/work/workplace/DCVehicleCounter/Resources/four_lane_day_many_trucks.mp4");
    
    //ImageEnhancer ie;
#if _USE_RESIZE_
    Resizer resizer(0.5);
#endif
    if (!cap->isOpened())
    {
        ERR(cout << "Can't open video file" << endl);
        return -1;
    }
    
    //cap->set(CV_CAP_PROP_POS_MSEC, 370000); // 1:07:06
    //cap->set(CV_CAP_PROP_POS_MSEC, 407000);
    //cap->set(CV_CAP_PROP_POS_MSEC, 415000);
    //cap->set(CV_CAP_PROP_POS_MSEC, 440000);   // 01:08:13
    //cap->set(CV_CAP_PROP_POS_MSEC, 595000);     //1:10:55
    
    double fps = cap->get(CV_CAP_PROP_FPS);
    cout << "Frame per second: " << fps << endl;
    
    uint64 framenumber = 0;
    uint cAB, cBA;
    Mat fore, back, enhanced, enhanced_color;
    Mat frame, raw_frame;
    while (1)
    {
        bool bSuccess = cap->read(raw_frame);
        
        if (!bSuccess)
        {
            ERR(cout << "Failed to read the frame from video file" << endl);
            break;
        }
        

#if 0//_USE_RESIZE_
        // Resize to 1/4 of original size to speed up
        resizer.resize(raw_frame, frame);
#else
        frame = raw_frame;
#endif

        framenumber++;
        cout << "==Frame " << framenumber << endl;
    
        blbl.process(frame, &cAB, framenumber);

        if (waitKey(30) == 27) // wait for 'esc' key press from 30 ms. If 'esc' key is pressed, break loop
        {
            cout << "ESC" << endl;
            break;
        }
    }
    if (cap) delete cap;
    return 0;
}

#elif RUN_MODE==DAY_MEDIUM_TRAFFIC_BLINK
#undef _USE_RESIZE_
#define _USE_RESIZE_ 0
// main for day_medium_traffic but using blinker
int main(int argc, const char * argv[])
{
    VideoCapture *cap;
    LBlinkerDay blbl;
    blbl.setMode(MODE_DAY_MEDIUM_TRAFFIC_BLINK);
    
    cap = new VideoCapture("./Day_Medium_traffic.3gp");
    //cap = new VideoCapture("/Volumes/DATA/work/workplace/DCVehicleCounter/Resources/day-jam.3gp");
    //cap = new VideoCapture("/Volumes/DATA/work/workplace/DCVehicleCounter/Resources/ClearNight.mp4");
    //cap = new VideoCapture("/Volumes/DATA/work/workplace/DCVehicleCounter/Resources/four_lane_night.mp4");
    //cap = new VideoCapture("/Volumes/DATA/work/workplace/DCVehicleCounter/Resources/four_lane_day_many_trucks.mp4");
    //cap = new VideoCapture("./Day_Medium_traffic.3gp");
    
    //VideoCapture cap("/Volumes/DATA/work/workplace/DCVehicleCounter/Resources/2014-7-10_nanpudaqiao.mp4");
    //VideoCapture cap("/Volumes/DATA/work/workplace/DCVehicleCounter/Resources/ClearNight.mp4");
    //VideoCapture cap("/Volumes/DATA/work/workplace/DCVehicleCounter/Resources/Rain_Night.mp4");
    //VideoCapture cap("/Volumes/DATA/work/workplace/DCVehicleCounter/Resources/four_lane_day_many_trucks.mp4");
    
    //ImageEnhancer ie;
    
#if _USE_RESIZE_
    Resizer resizer(0.6);
#endif
    if (!cap->isOpened())
    {
        ERR(cout << "Can't open video file" << endl);
        return -1;
    }
    
    cap->set(CV_CAP_PROP_POS_MSEC, 300000);
    
    double fps = cap->get(CV_CAP_PROP_FPS);
    cout << "Frame per second: " << fps << endl;
    
    uint64 framenumber = 0;
    uint cAB, cBA;
    Mat fore, back, enhanced, enhanced_color;
    Mat frame, raw_frame;
    while (1)
    {
        bool bSuccess = cap->read(raw_frame);
        
        if (!bSuccess)
        {
            ERR(cout << "Failed to read the frame from video file" << endl);
            break;
        }
        
#if _USE_RESIZE_
        // Resize to 1/4 of original size to speed up
        resizer.resize(raw_frame, frame);
#else
        frame = raw_frame;
#endif
        
        framenumber++;
        
        cout << "==Frame " << framenumber << endl;
        blbl.process(frame, &cAB, framenumber);
        if (waitKey(30) == 27) // wait for 'esc' key press from 30 ms. If 'esc' key is pressed, break loop
        {
            cout << "ESC" << endl;
            break;
        }
    }
    if (cap) delete cap;
    return 0;
}

#else

#endif
