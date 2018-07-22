#include <uWS/uWS.h>
#include <chrono>
#include <thread>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "BehaviorPlanner.h"

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

  int lane = 1;            // start in lane 1 FROM Walkthrough
  //double ref_vel = 0.0;   // mph, move a reference velocity to target, from walkthrough

  BehaviorPlanner planner = BehaviorPlanner(lane, "CS");

  h.onMessage([&planner,&lane](WebSocket<SERVER> ws, char *data, size_t length,
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

            //TODO CALL!
            vector<vector<double>> projection = planner.project(j);

            json msgJson;
            msgJson["next_x"] = projection[0];
          	msgJson["next_y"] = projection[1];

            cout << "msgJson[\"next_x\"] = " << msgJson["next_x"];
            cout << "msgJson[\"next_y\"] = " << msgJson["next_y"];


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
