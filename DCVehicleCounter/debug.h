//
//  debug.h
//  DCVehicleCounter
//
//  Created by hohe on 9/3/14.
//  Copyright (c) 2014 DongCheng. All rights reserved.
//

#ifndef DCVehicleCounter_debug_h
#define DCVehicleCounter_debug_h

#if DEBUG

#define DBOUT(x) x
#define ERR(x) x
#define DBSHOW(name, mat) imshow(name, mat)
#define DBSHOW_POS(name, mat, x, y) { \
                                    imshow((name), (mat)); \
                                    moveWindow((name), (x), (y)); \
                                }

#else

#define DBOUT(x)
#define ERR(x) x
#define DBSHOW( name , mat )
#define DBSHOW_POS(name, mat, x, y)

#endif

#endif
