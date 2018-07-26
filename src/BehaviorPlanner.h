#ifndef PATH_PLANNING_BEHAVIORPLANNER_H
#define PATH_PLANNING_BEHAVIORPLANNER_H

#include <string>
#include <vector>
#include <map>

#include "json.hpp"

#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"


using json = nlohmann::json;
using namespace std;
using namespace spdlog;

class BehaviorPlanner {

public:

    int lane;
    int intended_lane;
    string state;
    bool cold;        // cold start flag
    double velocity;   // mph, move a reference velocity to target, from walkthrough
    double governor;
    shared_ptr<logger> console;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    BehaviorPlanner(); // Constructor
    BehaviorPlanner(shared_ptr<logger> console, int lane, string state = "CS");
    virtual ~BehaviorPlanner(); // Destructor

    vector<vector<double>> project(json j);

    void configure();

    vector<double> lane_speeds(json j);

    vector<string> successor_states();

    string choose_next_state(json ego, json sensor_fusion, double projection_factor);

    int car_ahead(int lane, json ego, json cars, int within_distance, int projection_factor);

    double inefficiency_cost(string state, json ego, json cars, double projection_factor);

    double calculate_cost(string state, json ego, json cars, double projection_factor);

    double tie_breaker_cost(string state, json ego, json cars, double projection_factor);

    double safety_cost(string state, json ego, json cars, double projection_factor);

    bool is_in_lane(float d_from_center, int lane_to_test);

    double calc_speed(json car);

};

#endif //PATH_PLANNING_BEHAVIORPLANNER_H
