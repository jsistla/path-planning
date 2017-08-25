/*
 * constants.h
 * Copyright (C) 2017 Janakiram Sistla <janakiram.sistla@gmail.com>
 *
 * Distributed under terms of the MIT license.
 */
#ifndef PATH_PLANNING_CONSTANTS_H
#define PATH_PLANNING_CONSTANTS_H



#define MAX_POINTS 50 // Total points in the future path

double METERS_PER_MILE = 1609.344;

#define MAX(a,b) ((a) > (b) ? a : b)
#define MIN(a,b) ((a) < (b) ? a : b)


#define CONSTANT   1000000 // very large number
#define LANE_CENTER 2 //lane centre
#define LANE_WIDTH 4   //lane width
#define SIM_TICK 0.02  // 20ms
#define DISTINC 0.5   //distance increment to adjust speed, not used
#define PATH_HORIZON 30.0   // path horizon for spline
#define LANE_HORIZON  30.0  // Check how far ahead inside your lane
#define LANE_OFF 10
#define REF_VEL 49.0
#define MAXCOST 10000
#define THRESH 0.03


#endif //PATH_PLANNING_CONSTANTS_H
