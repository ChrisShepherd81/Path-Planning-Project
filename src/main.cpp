#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "json.hpp"

#include "GlobalMap.h"
#include "Path.h"
#include "PathWriter.h"
#include "SensorFusion.h"
#include "Trajectory.h"
#include "PathPlanning.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s);

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  GlobalMap globalMap;
  globalMap.init();
  SensorFusion sensorFusion;
  Trajectory trajectory(sensorFusion, globalMap);
  PathPlanning pathPlanning(sensorFusion);

  int lane = Simulator::START_LANE;

  h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {

        //**** Set the current simulator state ****

          // Main car's localization Data
          CarState car_state;
          car_state.c_pos = CartesianPoint{j[1]["x"], j[1]["y"]};
          double curr_car_s = j[1]["s"];
          car_state.f_pos = FrenetPoint{curr_car_s, j[1]["d"]};
          car_state.Yaw = j[1]["yaw"];
          car_state.curr_speed = j[1]["speed"];
          car_state.lane = lane;

          // Previous path data given to the Planner
          CartesianPath previous_path(j[1]["previous_path_x"], j[1]["previous_path_y"]);

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          sensorFusion.update(j[1]["sensor_fusion"]);

          int prev_size = previous_path.size();
          if(prev_size > 0)
          {
            // Previous path's end s and d values
            car_state.f_pos.s = j[1]["end_path_s"];
            car_state.f_pos.d = j[1]["end_path_d"];
          }
        //*****************************************

        //**** Path planning ****
          static size_t counter = 1; //TODO: by time
          if(counter%100 == 0)
          {
            lane = pathPlanning.getTargetLane(car_state, prev_size);
          }
          ++counter;
        //*****************************************

        //**** Trajectory generation ****
          CartesianPath path = trajectory.getNextPath(car_state, curr_car_s, previous_path, lane);
        //*****************************************

        //**** Set new values to the simulator ****
          json msgJson;
          msgJson["next_x"] = path.getXValues();
          msgJson["next_y"] = path.getYValues();
          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        //*****************************************
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h, &trajectory](uWS::WebSocket<uWS::SERVER> ws, int code,
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

string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}















































































