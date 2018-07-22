#include <fstream>
#include <cmath>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "BehaviorPlanner.h"
#include "utils.h"

using namespace std;
using namespace uWS;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of('[');
  auto b2 = s.find_first_of('}');
  if (found_null != string::npos)
    return "";
  else if (b1 != string::npos && b2 != string::npos)
    return s.substr(b1, b2 - b1 + 2);
  return "";
  //return (b1 != string::npos && b2 != string::npos) ? s.substr(b1, b2 - b1 + 2) : "";
}

int main() {

  Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  bool cold = true;        // cold start flag
  int lane = 1;            // start in lane 1 FROM Walkthrough
  double ref_vel = 0.0;   // mph, move a reference velocity to target, from walkthrough


  auto console = spdlog::stdout_color_mt("console");
  console->set_level(spdlog::level::debug);
  console->info("Begin Path Planning");

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
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

  BehaviorPlanner planner = BehaviorPlanner();

  h.onMessage([&console,&lane,&ref_vel,&cold,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](WebSocket<SERVER> ws, char *data, size_t length,
                     OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

      if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {     // j[1] is the data JSON object

          double car_x = j[1]["x"];     // Main car's localization Data
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

          if (prev_size > 0) car_s = end_path_s; // from walkthrough video
          bool too_close = false;
          double LANE_WIDTH = 4.0;
          double HALF_LANE_WIDTH = LANE_WIDTH / 2.0;
          for (int i = 0; i < sensor_fusion.size(); i++) { // find ref_v to use
            float d = sensor_fusion[i][6]; // car is in my lane
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
                }
              }
            }
          }

          if (too_close) {
            ref_vel -= 0.224;
            console->debug("decelerate to {}", ref_vel);
          } else if (ref_vel < 49.5) {
            ref_vel += 0.224;
            console->debug("accelerate to {}", ref_vel);
          }
           // end of from walkthrough video

          	json msgJson;

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
            vector<double> hair = getXY(car_s+0.1, car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
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
            ref_x = previous_path_x[prev_size-1]; // redefine the reference state as previous path end point
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            ptsx.push_back(ref_x_prev); // use two pts that make the path tangent to the previous path's end point
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);

          }

          vector<double> next_wp0 = getXY(car_s+30,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for (int i = 0; i < ptsx.size(); i++) { //shift car reference angle to 0 degrees
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
            ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
          }

          tk::spline trajectory; // create a spline

          trajectory.set_points(ptsx, ptsy); // set (x,y) points to the spline

          vector<double> next_x_vals; // define the actual (x,y) pts we will use for the planner
          vector<double> next_y_vals;

          for (int i =0; i < previous_path_x.size(); i++) { // start with all the previous path points from last time
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double target_x = 30; // calculate how to break up the spline so that we travel at our desired reference velocity
          double target_y = trajectory(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);

          double x_add_on = 0;


          for(int i = 0; i <= 50-previous_path_x.size(); i++) { // fill in rest of PP after filling in with prev pts, always outputs 50 pts
            double N = (target_dist/(0.02*ref_vel/2.24)); // 2.24 factor converts mph to m/s
            double x_point = x_add_on + (target_x)/N;
            double y_point = trajectory(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            x_point = (x_ref * cos(ref_yaw) - y_ref * sin (ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos (ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);

          }

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), OpCode::TEXT);
      }
    }
  });

  h.onConnection([](WebSocket<SERVER> ws, HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([](WebSocket<SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
