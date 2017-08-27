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

#define LANE_CENTER   2      //lane centre
#define LANE_WIDTH    4      //lane width
#define SIM_TICK      0.02   // 20ms
#define DISTINC       0.5    //distance increment to adjust speed, not used
#define PATH_HORIZON  30.0   // path horizon for spline
#define LANE_HORIZON  30.0   // Check how far ahead inside your lane
#define LANE_OFF      10
#define REF_VEL       49.0
#define MAX_COST       10000
#define WP_FILE       ("../data/highway_map.csv") /*!< The path to the waypoint file */
#define LARGE_CONSTANT      1000000 // very large number
#define MAX_S_VALUE   6945.554
  // The max s value before wrapping around the track back to 0
#define MAX_SPEED     22.2
#define MAX_ACCEL     9.5
#define MAX_JERK      45.0
#endif //PATH_PLANNING_CONSTANTS_H
