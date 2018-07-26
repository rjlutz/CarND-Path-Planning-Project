#include <fstream>
#include <limits>

#include "BehaviorPlanner.h"
#include "spline.h"
#include "utils.h"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "planning_constants.h"

#include <algorithm>

using namespace std;
using namespace spdlog;

BehaviorPlanner::BehaviorPlanner() = default; // Initializes Vehicle

BehaviorPlanner::BehaviorPlanner(shared_ptr<logger> console, int lane, string state) {
  this->console = console;
  this->lane = lane;
  this->intended_lane = lane;
  this->state = state;
  this->cold = true;
  this->velocity = 0.0;
  this->governor = TARGET_SPEED;

  this->configure();
}

bool BehaviorPlanner::is_in_same_lane(float d, int lane_to_test) {
  double hlw = LANE_WIDTH / 2.0;
  double lw = LANE_WIDTH;
  return d < (hlw + lw * lane_to_test + hlw) && d > (hlw + lw * lane_to_test - hlw);
}

double BehaviorPlanner::calculate_cost(string state, json ego, json cars, double projection_factor) {
  double i_cost = inefficiency_cost(state, ego, cars, projection_factor);
  double s_cost = safety_cost(state, ego, intended_lane, cars, projection_factor);
  double t_cost = tie_breaker_cost(state, ego, cars, projection_factor);
  double total = 0.05 * i_cost + 0.90 * s_cost + 0.05 * t_cost;
  console->debug("candidate state:{:>4} i:{: 3.2f} s:{: 3.2f} t:{: 3.2f}",
                 state, i_cost, s_cost, t_cost);
  console->debug("                     i:{: 3.2f} s:{: 3.2f} t:{: 3.2f} total:{: 3.2f}",
                 0.05 * i_cost, 0.90 * s_cost, 0.05 * t_cost, total);
  return total;
}

double BehaviorPlanner::tie_breaker_cost(string state, json ego, json cars, double projection_factor) {
  // look ahead to see which side has more space ahead
  double cost = -1;
  if (lane == 0 || lane == LANES_AVAIL-1) return cost;

  int left_lane = lane - 1;
  int right_lane = lane + 1;
  int r_car_ahead = car_ahead(right_lane, ego, cars, 3 * FAR_SEE, projection_factor);
  int l_car_ahead = car_ahead(left_lane, ego, cars, 3 * FAR_SEE, projection_factor);

  if (state.compare("PLCL") == 0) {
    if (r_car_ahead != -1 && l_car_ahead != -1)
      cost = cars[l_car_ahead][5] > cars[r_car_ahead][5] ? 1.0 : 0.0;
    else if (r_car_ahead == -1 && l_car_ahead == -1)
      cost = 0.0;
    else
      cost = (l_car_ahead == -1) ? -1.0 : 0.0;
  }

  if (state.compare("PLCR") == 0) {
    if (r_car_ahead != -1 && l_car_ahead != -1)
      cost = cars[r_car_ahead][5] > cars[l_car_ahead][5] ? 1.0 : 0.0;
    else if (r_car_ahead == -1 && l_car_ahead == -1)
      cost = 0.0;
    else
      cost = (r_car_ahead == -1) ? -1.0 : 0.0;
  }
  return cost;
}


double BehaviorPlanner::safety_cost(string state, json ego, int intended_lane, json cars, double projection_factor) {

  double cost = -1;
  double ego_s = ego["s"];
  json other_car;
  double other_car_s, other_car_d;

  if (velocity < 25.0) return 1;

  if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {

    for (int i = 0; i < cars.size(); i++) {
      other_car = cars[i];
      other_car_s = other_car[5];
      other_car_d = other_car[6]; // is other car is in target lane?
      if (is_in_same_lane(other_car_d, intended_lane)) {
        other_car_s += projection_factor * calc_speed(other_car);
        double dist_s = other_car_s - ego_s;
        if (abs(dist_s) < 3 * CAR_LENGTH) {
          console->trace("s:{} check_car_s:{} dist_x:{} car is too close when attempting {}",
                         ego_s, other_car_s, dist_s, state);
          cost = 1;
          break; // we only need one!
        }
      }
    }
  }

//  if (state.compare("LCR") == 0) {
//
//    //int other_lane = lane + 1;
//    for (int i = 0; i < cars.size(); i++) {
//      json other_car = cars[i];
//      float d = other_car[6]; // is other car is in right lane?
//      if (is_in_same_lane(d, intended_lane)) {
//        double check_car_s = ((double)other_car[5]) + projection_factor * calc_speed(other_car);
//        double dist_s = check_car_s - s;
//        console->trace("s:{} check_car_s:{} dist_x:{}", s, check_car_s, dist_s);
//        if (abs(dist_s) < 3 * CAR_LENGTH) {
//          console->trace("car on right is too close {}", dist_s);
//          cost = 1;
//          break;
//        }
//      }
//    }
//  }

  return cost;

}

double BehaviorPlanner::inefficiency_cost(string state, json ego, json cars, double projection_factor) {

  double cost = -1.0;
  double delta = 0.0;
  vector<double> speeds = lane_speeds(cars);

  if (state.compare("KL") == 0) {
    int car_ahead_index = car_ahead(lane, ego, cars, 2 * FAR_SEE, projection_factor);

    if (car_ahead_index != -1) {                //penalize KL heavily if car ahead
      //cost = 1.0;
      json other = cars[car_ahead_index];
      double other_s = other[5];
      double s = ego["s"];
      cost = map_value(other_s - s, 60, 20, -1, 1);
    } else {
      auto fastest_lane = max_element(speeds.begin(), speeds.end());
      cost = map_value(*fastest_lane - speeds[lane], MAX_SPEED, 1, -1, 1);
    }
  }

  if (state.compare("PLCL") == 0) {
    if (lane == 0) {
      cost = 1.0; // no lanes to the left
    } else {
      delta = speeds[lane - 1] - speeds[lane];
      cost = map_value(delta, 60, -60, -1.0, 1.0);
    }
  }

  if (state.compare("PLCR") == 0) {
    if (lane == LANES_AVAIL-1) {
      cost = 1.0; // no lanes to the right or too slow to change
    } else {
      delta = speeds[lane + 1] - speeds[lane];
      cost = map_value(delta, 60, -60, -1.0, 1.0);
    }
  }

  console->trace("state: {} considering {} cost: {} lane: {} velocity: {} delta: {} Speeds: {} {} {}",
                 this->state, state, cost, lane, velocity, delta, speeds[0], speeds[1], speeds[2]);

  return cost;
}

vector<double> BehaviorPlanner::lane_speeds(json cars) {

  // calculate the average speed of each lane

  vector<double> speeds = {-1, -1, -1};
  for (int i = 0; i < LANES_AVAIL; i++) {
    double agg_speed = 0.0;
    int count = 0;
    for (int j = 0; j < cars.size(); j++) {
      float d = cars[j][6];
      double vx = cars[j][3];
      double vy = cars[j][4];
      if (d < 0 || d > 3 * LANE_WIDTH) {
        console->trace("dropping crazy d value, out of bounds:  {}", d);
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

// speed of a simulator (unity game engine) car that is returned in a sensor_fusion json object
double BehaviorPlanner::calc_speed(json car) {
  double vx = car[3];
  double vy = car[4];
  double speed = sqrt(vx * vx + vy * vy);
  return speed;
}

int BehaviorPlanner::car_ahead(int lane, json ego, json cars, int within_distance, int projection_factor) {

  int result = -1;
  //int closest = numeric_limits<double>::max();
  int closest = 999999.0;
  double s = ego["s"];

  for (int i = 0; i < cars.size(); i++) { // find ref_v to use

    json other = cars[i];
    float d = other[6]; // other car is in my lane?
    if (is_in_same_lane(d, lane)) {
      double check_car_s = other[5];
      check_car_s += ((double) projection_factor * calc_speed(other)); // prev pts can project s value outwards in time
      double distance_to_vehicle = check_car_s - s;
      if (distance_to_vehicle > 0 && (distance_to_vehicle < within_distance)) { // in meters
        if (distance_to_vehicle < closest) {
          closest = distance_to_vehicle;
          result = i;
        }
      }
    }
  }
  if (result != -1) {
    console->trace("returning lane: {} result: {} closest: {}", lane, result, closest);
  }
  return result;
}

vector<string> BehaviorPlanner::successor_states() {
  /*
  Provides the possible next states given the current state.
  */
  vector<string> states;
  string state = this->state;
  if(state.compare("KL") == 0) {
    states.push_back("KL");
    states.push_back("PLCL");
    states.push_back("PLCR");
  } else if (state.compare("PLCL") == 0) {
      states.push_back("PLCL");
      states.push_back("LCL");
      states.push_back("KL");
  } else if (state.compare("PLCR") == 0) {
      states.push_back("PLCR");
      states.push_back("LCR");
      states.push_back("KL");
  } else if (state.compare("LCL") == 0) {
    states.push_back("KL");
    states.push_back("LCL");
  } else if (state.compare("LCR") == 0) {
    states.push_back("KL");
    states.push_back("LCR");
  }
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

string BehaviorPlanner::choose_next_state(json ego, json cars, double projection_factor) {

  string best = successor_states()[0];
  float min_cost = numeric_limits<float>::max();

  for (auto candidate : successor_states()) {
    double cost = calculate_cost(candidate, ego, cars, projection_factor);
    console->trace("candidate {}: cost: {}", candidate, cost);
    if (cost < min_cost) {
      min_cost = cost;
      best = candidate;
    }
  }
  console->debug("best candidate {}: cost: {}\n", best, min_cost);
  return best;
}

vector<vector<double>> BehaviorPlanner::project(json j) {

  json ego = j[1];

  double car_x = ego["x"];     // Main car's localization data
  double car_y = ego["y"];
  double car_s = ego["s"];
  double car_d = ego["d"];
  double car_yaw = ego["yaw"];
  double car_speed = ego["speed"];

  auto previous_path_x = ego["previous_path_x"];    // Previous path data given to the Planner
  auto previous_path_y = ego["previous_path_y"];
  double end_path_s = ego["end_path_s"];            // Previous path's end s and d values
  double end_path_d = ego["end_path_d"];

  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  auto sensor_fusion = ego["sensor_fusion"];
  auto speeds = lane_speeds(sensor_fusion);

  int prev_size = previous_path_x.size();

  vector<double> next_x_vals; // define the actual (x,y) pts we will use for the simulator planner
  vector<double> next_y_vals;
  vector<vector<double>> result;

  if (prev_size > 0) car_s = end_path_s; // from walkthrough video

  string next_state;
  if (intended_lane != lane) {
    if (abs(car_d - (LANE_WIDTH / 2.0 + intended_lane * LANE_WIDTH)) < 0.5) {
      next_state = "KL";
      lane = intended_lane; // lane change complete
      console->debug("intended lane acquired new state: {} current lane {}", next_state, lane);

    } else {
      console->trace("NOT acquired, state maintained as : {}", state);
      next_state = state; // "LCL" or "LCR"
    }
  } else {
    next_state = choose_next_state(ego, sensor_fusion, prev_size * 0.02);
    console->trace("next state = {}", next_state);
  }

  if (next_state.compare("KL") == 0 || next_state.compare("PLCL") == 0 ||
      next_state.compare("PLCR") == 0) {
    // no action required
  } else if (next_state.compare("LCL") == 0) {
    intended_lane = lane - 1;
  } else if (next_state.compare("LCR") == 0) {
    intended_lane = lane + 1;
  }

  if (next_state.compare(state) != 0) {
    state = next_state;
    string msg = "";
    for (string s : successor_states()) {
      if (msg.compare("") != 0) msg += " ";
      msg += s;
    }
    console->debug("state changed to: {} successors: {}", state, msg);
  }

  int car_ahead_index = car_ahead(lane, ego, sensor_fusion, 2 * FAR_SEE, prev_size * 0.02);
  if (car_ahead_index != -1) {
    console->trace("car_ahead_index: {}", car_ahead_index);
  }
  if (intended_lane != lane) {
    console->trace("lane:{} intended lane:{}");
    governor =speeds[intended_lane];
    if (governor == -1) {
      governor = TARGET_SPEED;
    }
  } else if (car_ahead_index != -1) {
      json leader = sensor_fusion[car_ahead_index];
      governor = calc_speed(leader) * 2.24 - 2.0; // TODO adopt a PID or MPC here
  } else {
    governor = TARGET_SPEED;
  }

  if (abs(velocity - governor) < 0.1) {
    console->trace("state {} MAINTAIN velocity = {} lane = {} intended_lane = {}", state, velocity, lane , intended_lane);
  } else if (velocity > governor) {
    velocity -= MAX_ACCEL;
    console->trace("state {} too close to next car decelerate to {} governor {}", state, velocity, governor);
  } else {
    velocity += MAX_ACCEL;
    console->trace("state {} NOT too close to next car accelerate to {}", state, velocity);
  }

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
      console->info("Begin Path Planning");   // 'hair' is a fabricated vector, so that we have an initial direction
      vector<double> hair = getXY(car_s + 0.1, car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
      ptsx.push_back(car_x);
      ptsx.push_back(hair[0]);
      ptsy.push_back(car_y);
      ptsy.push_back(hair[1]);
      cold = false;
    } else if (prev_size < 2) {    // if the previous size is almost empty, use car as starting reference
      double prev_car_x = car_x - cos(car_yaw); // use two points that make the path tangent to the car
              double prev_car_y = car_y - sin(car_yaw);
              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);
              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);
    } else {                       // use the previous path's end point as the starting reference
      ref_x = previous_path_x[prev_size - 1];   // redefine the reference state as previous path end point
      ref_y = previous_path_y[prev_size - 1];
      double ref_x_prev = previous_path_x[prev_size - 2];
      double ref_y_prev = previous_path_y[prev_size - 2];
      ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
      ptsx.push_back(ref_x_prev); // use two pts that make the path tangent to the previous path's end point
      ptsx.push_back(ref_x);
      ptsy.push_back(ref_y_prev);
      ptsy.push_back(ref_y);
    }

    double lw = LANE_WIDTH, hlw = LANE_WIDTH / 2.0;
    vector<double> next_wp0 = getXY(car_s +   FAR_SEE, (hlw + lw * intended_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY(car_s + 2*FAR_SEE, (hlw + lw * intended_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = getXY(car_s + 3*FAR_SEE, (hlw + lw * intended_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

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

  // fill in rest of PP after filling in with prev pts, always outputs 50 pts
  for (int i = 0; i <= 50 - previous_path_x.size(); i++) {

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


