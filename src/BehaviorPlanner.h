//
// Created by Bob Lutz on 7/22/18.
//

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

    static constexpr double LANE_WIDTH = 4.0;
    static constexpr double HALF_LANE_WIDTH = LANE_WIDTH / 2.0;
    static constexpr double MAX_ACCEL = 0.224;

    static constexpr double MAX_S = 6945.554; // The max s value before wrapping around the track back to 0

    // Waypoint map to read from
    string MAP_FILE = "../data/highway_map.csv";

    map<string, int> lane_direction = {{"PLCL", 1},
                                       {"LCL",  1},
                                       {"LCR",  -1},
                                       {"PLCR", -1}};

    int lane;
    string state;
    bool cold;        // cold start flag
    double velocity;   // mph, move a reference velocity to target, from walkthrough
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

//        vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> predictions);
//
    vector<string> successor_states();
//
//        vector<Vehicle> generate_trajectory(string state, map<int, vector<Vehicle>> predictions);
//
//        vector<float> get_kinematics(map<int, vector<Vehicle>> predictions, int lane);
//
//        vector<Vehicle> constant_speed_trajectory();
//
//        vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> predictions);
//
//        vector<Vehicle> lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);
//
//        vector<Vehicle> prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);
//
//        void increment(int dt);
//
//        float position_at(int t);
//
//        bool get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);
//
//        bool get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);
//
//        vector<Vehicle> generate_predictions(int horizon=2);
//
//        void realize_next_state(vector<Vehicle> trajectory);
//
//        void configure(vector<int> road_data);

};

#endif //PATH_PLANNING_BEHAVIORPLANNER_H
