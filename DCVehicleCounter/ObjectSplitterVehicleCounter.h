//
//  ObjectSplitterVehicleCounter.h
//  DCVehicleCounter
//
//  Created by hohe on 7/18/14.
//  Copyright (c) 2014 DongCheng. All rights reserved.
//

#ifndef __DCVehicleCounter__ObjectSplitterVehicleCounter__
#define __DCVehicleCounter__ObjectSplitterVehicleCounter__

#include <iostream>
#include "ObjectSplitter.h"

class ObjectSplitterVehicleCounter : public ObjectSplitter
{
protected:
    virtual bool registerObject(InputArray mask, uchar val, Rect &rect, uint area, uint64 fn);
    
public:
    ObjectSplitterVehicleCounter() : ObjectSplitterVehicleCounter(100, 15000)
    {
        
    }
    
    ObjectSplitterVehicleCounter(uint min_size, uint max_size) : ObjectSplitter(100, 15000)
    {
        
    }
    virtual void drawObjects(InputOutputArray img, uint64 fn);
};

class ObjectSplitterVehicleCounterNight : public ObjectSplitterVehicleCounter
{
public:
    ObjectSplitterVehicleCounterNight() : ObjectSplitterVehicleCounter(100, 15000) // Night we only track headlights which is smaller
    {
        
    }
};

#endif /* defined(__DCVehicleCounter__ObjectSplitterVehicleCounter__) */
