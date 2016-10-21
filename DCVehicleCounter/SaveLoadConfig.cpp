//
//  SaveLoadConfig.cpp
//  DCVehicleCounter
//
//  Created by hohe on 8/11/14.
//  Copyright (c) 2014 DongCheng. All rights reserved.
//

#include "SaveLoadConfig.h"

std::string SaveLoadConfig::getConfigFileName()
{
#if defined MADEBYCMAKE
    std::string prefix = "./config/";
#else
    std::string prefix = "../../../../../DCVehicleCounter/config/";
#endif
    std::string filename = "";
    switch (myid)
    {
        case IDS_DEFAULT:
        default:
            filename = "";
            break;
        case IDS_VEHICLE_TRACKING:
        {
            switch (mode)
            {
                case MODE_DEFAULT:
                default:
                    filename = "VT.xml";
                    break;
                case MODE_CLEAR_NIGHT:
                    filename = "VT_CN.xml";
                    break;
                case MODE_DAY_MEDIUM_TRAFFIC:
                    filename = "VT_DMT.xml";
                    break;
                case MODE_DAY_MEDIUM_TRAFFIC_BLINK:
                    filename = "VT_DMTB.xml";
                    break;
                case MODE_FOUR_LANES_NIGHT:
                    filename = "VT_FLN.xml";
                    break;
            }
            break;
        }
        case IDS_BLINKER:
        {
            switch (mode)
            {
                case MODE_DEFAULT:
                default:
                    filename = "BL.xml";
                    break;
                case MODE_CLEAR_NIGHT:
                    filename = "BL_CN.xml";
                    break;
                case MODE_DAY_MEDIUM_TRAFFIC:
                    filename = "BL_DMT.xml";
                    break;
                case MODE_FOUR_LANES_NIGHT:
                    filename = "BL_FLN.xml";
                    break;
                case MODE_DAY_MEDIUM_TRAFFIC_BLINK:
                    filename = "BL_DMTB.xml";
                    break;
            }
            break;
        }
        case IDS_VEHICLEMERGER:
        {
            switch (mode)
            {
                case MODE_DEFAULT:
                default:
                    filename = "VM.xml";
                    break;
                case MODE_CLEAR_NIGHT:
                    filename = "VM_CN.xml";
                    break;
                case MODE_DAY_MEDIUM_TRAFFIC:
                    filename = "VM_DMT.xml";
                    break;
                case MODE_FOUR_LANES_NIGHT:
                    filename = "VM_FLN.xml";
                    break;
                case MODE_DAY_MEDIUM_TRAFFIC_BLINK:
                    filename = "VM_DMTB.xml";
                    break;
            }
            break;
        }
        case IDS_BACKGROUND_PROCESSOR:
        {
            switch (mode)
            {
                case MODE_DEFAULT:
                default:
                    filename = "BP.xml";
                    break;
                case MODE_CLEAR_NIGHT:
                    filename = "BP_CN.xml";
                    break;
                case MODE_DAY_MEDIUM_TRAFFIC:
                    filename = "BP_DMT.xml";
                    break;
                case MODE_FOUR_LANES_NIGHT:
                    filename = "BP_FLN.xml";
                    break;
                case MODE_DAY_MEDIUM_TRAFFIC_BLINK:
                    filename = "BP_DMTB.xml";
                    break;
            }
            break;
        }
        case IDS_OBJECT_SPLITTER:
        {
            switch (mode)
            {
                case MODE_DEFAULT:
                default:
                    filename = "OS.xml";
                    break;
                case MODE_CLEAR_NIGHT:
                    filename = "OS_CN.xml";
                    break;
                case MODE_DAY_MEDIUM_TRAFFIC:
                    filename = "OS_DMT.xml";
                    break;
                case MODE_FOUR_LANES_NIGHT:
                    filename = "OS_FLN.xml";
                    break;
                case MODE_DAY_MEDIUM_TRAFFIC_BLINK:
                    filename = "OS_DMTB.xml";
                    break;
            }
            break;
        }
        case IDS_EDGE_DETECTOR:
        {
            switch (mode)
            {
                case MODE_DEFAULT:
                default:
                    filename = "ED.xml";
                    break;
                case MODE_CLEAR_NIGHT:
                    filename = "ED_CN.xml";
                    break;
                case MODE_DAY_MEDIUM_TRAFFIC:
                    filename = "ED_DMT.xml";
                    break;
                case MODE_FOUR_LANES_NIGHT:
                    filename = "ED_FLN.xml";
                    break;
                case MODE_DAY_MEDIUM_TRAFFIC_BLINK:
                    filename = "ED_DMTB.xml";
                    break;
            }
            break;
        }
        case IDS_IMAGE_ENHANCER:
        {
            switch (mode)
            {
                case MODE_DEFAULT:
                default:
                    filename = "IE.xml";
                    break;
                case MODE_CLEAR_NIGHT:
                    filename = "IE_CN.xml";
                    break;
                case MODE_DAY_MEDIUM_TRAFFIC:
                    filename = "IE_DMT.xml";
                    break;
                case MODE_FOUR_LANES_NIGHT:
                    filename = "IE_FLN.xml";
                    break;
                case MODE_DAY_MEDIUM_TRAFFIC_BLINK:
                    filename = "IE_DMTB.xml";
                    break;
            }
            break;
        }
        case IDS_RESIZER:
        {
            switch (mode)
            {
                case MODE_DEFAULT:
                default:
                    filename = "RS.xml";
                    break;
                case MODE_CLEAR_NIGHT:
                    filename = "RS_CN.xml";
                    break;
                case MODE_DAY_MEDIUM_TRAFFIC:
                    filename = "RS_DMT.xml";
                    break;
                case MODE_FOUR_LANES_NIGHT:
                    filename = "RS_FLN.xml";
                    break;
                case MODE_DAY_MEDIUM_TRAFFIC_BLINK:
                    filename = "RS_DMTB.xml";
                    break;
            }
            break;
        }
        case IDS_DAY_NONJAM_PROCESSOR:
        {
            switch (mode)
            {
                case MODE_DEFAULT:
                default:
                    filename = "DNP.xml";
                    break;
                case MODE_CLEAR_NIGHT:
                    filename = "DNP_CN.xml";
                    break;
                case MODE_DAY_MEDIUM_TRAFFIC:
                    filename = "DNP_DMT.xml";
                    break;
                case MODE_FOUR_LANES_NIGHT:
                    filename = "DNP_FLN.xml";
                    break;
                case MODE_DAY_MEDIUM_TRAFFIC_BLINK:
                    filename = "DNP_DMTB.xml";
                    break;
            }
            break;
        }
    }
    
    return prefix + filename;
}