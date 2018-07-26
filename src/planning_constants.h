
#ifndef PATH_PLANNING_PLANNING_CONSTANTS_H
#define PATH_PLANNING_PLANNING_CONSTANTS_H

static const double MAX_SPEED = 50; // mph
static const double MARGIN_SPEED = 0.5;
static const double TARGET_SPEED = MAX_SPEED - MARGIN_SPEED;

static const double MAX_ACCEL = 0.224;
static const double MAX_S = 6945.554;    // The max s value before wrapping around the track back to 0

static const int LANES_AVAIL = 3;
static const double LANE_WIDTH = 4.0;
static const double CAR_LENGTH = 6.0;
static const double FAR_SEE = 30.0;

static const string MAP_FILE = "../data/highway_map.csv"; // Waypoint map to read from

#endif //PATH_PLANNING_PLANNING_CONSTANTS_H

