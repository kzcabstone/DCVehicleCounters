//
//  SaveLoadConfig.h
//  DCVehicleCounter
//
//  Created by hohe on 8/11/14.
//  Copyright (c) 2014 DongCheng. All rights reserved.
//

#ifndef __DCVehicleCounter__SaveLoadConfig__
#define __DCVehicleCounter__SaveLoadConfig__

#include <iostream>

enum MODE
{
    MODE_DEFAULT = 0,
    MODE_CLEAR_NIGHT,
    MODE_DAY_MEDIUM_TRAFFIC,
    MODE_DAY_MEDIUM_TRAFFIC_BLINK,
    MODE_FOUR_LANES_NIGHT,
    MODE_FOUR_LANES_DAY,
    MODE_JAM
};

class SaveLoadConfig
{
protected:
    enum IDS
    {
        IDS_DEFAULT = 0,
        IDS_VEHICLE_TRACKING,
        IDS_OBJECT_SPLITTER,
        IDS_EDGE_DETECTOR,
        IDS_IMAGE_ENHANCER,
        IDS_RESIZER,
        IDS_BACKGROUND_PROCESSOR,
        IDS_BLINKER,
        IDS_VEHICLEMERGER,
        IDS_DAY_NONJAM_PROCESSOR
    };
    
    IDS myid;
    MODE mode;
    bool config_loaded;
    std::string getConfigFileName();
    virtual void saveConfig() = 0;
    virtual void loadConfig() = 0;
    
public:
    SaveLoadConfig() : SaveLoadConfig(IDS_DEFAULT, MODE_DEFAULT)
    {
    }
    
    SaveLoadConfig(IDS i, MODE m)
    {
        myid = i;
        mode = m;
        config_loaded = false;
    }
    
    ~SaveLoadConfig()
    {
    }
    
    virtual void setMode(MODE m)
    {
        mode = m;
    }
};

#endif /* defined(__DCVehicleCounter__SaveLoadConfig__) */
