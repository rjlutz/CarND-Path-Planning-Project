#include <fstream>
#include "BehaviorPlanner.h"
#include "spline.h"
#include "utils.h"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"

using namespace std;
using namespace spdlog;

BehaviorPlanner::BehaviorPlanner() = default; // Initializes Vehicle

BehaviorPlanner::BehaviorPlanner(shared_ptr<logger> console, int lane, string state) {
  this->console = console;
  this->lane = lane;
  this->state = state;
  this->cold = true;
  this->velocity = 0.0;

  this->configure();
}

vector<double> BehaviorPlanner::lane_speeds(json sensor_fusion) {

  // calc the average speed of each lane

  vector<double> speeds = {-1, -1, -1};
  for (int i = 0; i < LANES_AVAIL; i++) {
    double agg_speed = 0.0;
    int count = 0;
    for (int j = 0; j < sensor_fusion.size(); j++) {
      float d = sensor_fusion[j][6];
      double vx = sensor_fusion[j][3];
      double vy = sensor_fusion[j][4];
      if (d < 0 || d > 3 * LANE_WIDTH) {
        console->debug("dropping crazy d value, out of bounds:  {}", d);
        continue;
      }
      int l = 0; // the car's lane, assume 0 then adjust
      if (d > 2 * LANE_WIDTH)
        l = 2;
      else if (d > LANE_WIDTH)
        l = 1;
      if (l == i) {
        count++;
        double speed = sqrt(vx * vx + vy * vy);
        agg_speed += speed;
      }
    }
    if (agg_speed > 0.0)
      speeds[i] = agg_speed / count * 2.24;
  }
  return speeds;
}

vector<string> BehaviorPlanner::successor_states() {
  /*
  Provides the possible next states given the current state for the FSM
  discussed in the course, with the exception that lane changes happen
  instantaneously, so LCL and LCR can only transition back to KL.
  */
  vector<string> states;
  states.push_back("KL");
  string state = this->state;
  if(state.compare("KL") == 0) {
    states.push_back("PLCL");
    states.push_back("PLCR");
  } else if (state.compare("PLCL") == 0) {
    if (lane != LANES_AVAIL - 1) {
      states.push_back("PLCL");
      states.push_back("LCL");
    }
  } else if (state.compare("PLCR") == 0) {
    if (lane != 0) {
      states.push_back("PLCR");
      states.push_back("LCR");
    }
  }
  //If state is "LCL" or "LCR", then just return "KL"
  return states;
}

BehaviorPlanner::~BehaviorPlanner() = default;

void BehaviorPlanner::configure() {

  ifstream in_map_(MAP_FILE.c_str(), ifstream::in);
  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x, y, s, d_x, d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
}

vector<vector<double>> BehaviorPlanner::project(json j) {

  double car_x = j[1]["x"];     // Main car's localization data
  double car_y = j[1]["y"];
  double car_s = j[1]["s"];
  double car_d = j[1]["d"];
  double car_yaw = j[1]["yaw"];
  double car_speed = j[1]["speed"];

  auto previous_path_x = j[1]["previous_path_x"];    // Previous path data given to the Planner
  auto previous_path_y = j[1]["previous_path_y"];
  double end_path_s = j[1]["end_path_s"];            // Previous path's end s and d values
  double end_path_d = j[1]["end_path_d"];

  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  auto sensor_fusion = j[1]["sensor_fusion"];

  int prev_size = previous_path_x.size();

  vector<double> next_x_vals; // define the actual (x,y) pts we will use for the simulator planner
  vector<double> next_y_vals;
  vector<vector<double>> result;

  if (prev_size > 0) car_s = end_path_s; // from walkthrough video
  bool too_close = false;

  for (int i = 0; i < sensor_fusion.size(); i++) { // find ref_v to use
    float d = sensor_fusion[i][6]; // other car is in my lane?
    if (d < (HALF_LANE_WIDTH + LANE_WIDTH * lane + HALF_LANE_WIDTH) &&
        d > (HALF_LANE_WIDTH + LANE_WIDTH * lane - HALF_LANE_WIDTH)) {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx * vx + vy * vy);
      double check_car_s = sensor_fusion[i][5];

      check_car_s += ((double) prev_size * 0.02 * check_speed); // if using previous points can
      // project s value outwards in time
      if ((check_car_s > car_s) && ((check_car_s - car_s) < 30)) { // in meters
        // set flag to slow and attempt lane change
        too_close = true;
        if (lane > 0) { // need state machine here
          lane = 0;
          console->debug("change lane to {}", lane);
        }
      }
    }
  }

    vector<double> speeds = lane_speeds(sensor_fusion);
//    console->debug("lane speeds : {} {} {}", speeds[0], speeds[1], speeds[2]);

    if (too_close) {
      velocity -= MAX_ACCEL;
      console->debug("decelerate to {}", velocity);
    } else if (velocity < TARGET_SPEED) {
      velocity += MAX_ACCEL;
      console->debug("accelerate to {}", velocity);
    }
    // end of from walkthrough video

    // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
    // Later we will interpolate these waypoints with a spline and fill it in with more points that control speed
    vector<double> ptsx;
    vector<double> ptsy;

    // reference x,y,yaw states
    // either we will reference the starting point as where the car is or at the previous paths endpoint
    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = deg2rad(car_yaw);

    console->trace("prev_size = {}", prev_size);

    if (cold) {
      // hair is a fabricated vector, so that we have an initial direction
      console->info("Begin Path Planning");
      vector<double> hair = getXY(car_s + 0.1, car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
      ptsx.push_back(car_x);
      ptsx.push_back(hair[0]);
      ptsy.push_back(car_y);
      ptsy.push_back(hair[1]);
      cold = false;
      // if the previous size is almost empty, use car as starting reference
    } else if (prev_size < 2) {
      // use two points that make the path tangent to the car
      double prev_car_x = car_x - cos(car_yaw);
      double prev_car_y = car_y - sin(car_yaw);
      ptsx.push_back(prev_car_x);
      ptsx.push_back(car_x);
      ptsy.push_back(prev_car_y);
      ptsy.push_back(car_y);
    } else { // use the previous path's end point as the starting reference
      ref_x = previous_path_x[prev_size - 1]; // redefine the reference state as previous path end point
      ref_y = previous_path_y[prev_size - 1];
      double ref_x_prev = previous_path_x[prev_size - 2];
      double ref_y_prev = previous_path_y[prev_size - 2];
      ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
      ptsx.push_back(ref_x_prev); // use two pts that make the path tangent to the previous path's end point
      ptsx.push_back(ref_x);
      ptsy.push_back(ref_y_prev);
      ptsy.push_back(ref_y);
    }

    vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    for (int i = 0; i < ptsx.size(); i++) { //shift car reference angle to 0 degrees
      double shift_x = ptsx[i] - ref_x;
      double shift_y = ptsy[i] - ref_y;
      ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
      ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
    }

    tk::spline trajectory; // create a spline

    trajectory.set_points(ptsx, ptsy); // set (x,y) points to the spline

    for (int i = 0; i < previous_path_x.size(); i++) { // start with all the previous path points from last time
      next_x_vals.push_back(previous_path_x[i]);
      next_y_vals.push_back(previous_path_y[i]);
    }

    double target_x = 30; // calculate how to break up the spline so that we travel at our desired reference velocity
    double target_y = trajectory(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);

    double x_add_on = 0;

    for (int i = 0; i <= 50 -
                         previous_path_x.size(); i++) { // fill in rest of PP after filling in with prev pts, always outputs 50 pts
      double N = (target_dist / (0.02 * velocity / 2.24)); // 2.24 factor converts mph to m/s
      double x_point = x_add_on + (target_x) / N;
      double y_point = trajectory(x_point);

      x_add_on = x_point;

      double x_ref = x_point;
      double y_ref = y_point;

      x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
      y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

      x_point += ref_x;
      y_point += ref_y;

      next_x_vals.push_back(x_point);
      next_y_vals.push_back(y_point);

  }
  result.push_back(next_x_vals);
  result.push_back(next_y_vals);
  return result;
}


