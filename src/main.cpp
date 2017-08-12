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
#include "Prediction.h"
#include "TrajectoryGenerator.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
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

//constexpr double pi() { return M_PI; }
//double deg2rad(double x) { return x * pi() / 180; }
//double rad2deg(double x) { return x * 180 / pi(); }

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  GlobalMap globalMap;
  globalMap.init();
  Prediction prediction;
  TrajectoryGenerator trajectory(globalMap);
  CostCalculation costCalculation(prediction);

  int lane = Simulator::START_LANE;
  double target_speed = 0;

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
            // j[1] is the data JSON object

        	  // Main car's localization Data
          	CarState car_state;
          	car_state.c_pos = CartesianPoint{j[1]["x"], j[1]["y"]};
          	double curr_car_s = j[1]["s"];
          	car_state.f_pos = FrenetPoint{curr_car_s, j[1]["d"]};
          	car_state.Yaw = j[1]["yaw"];
          	car_state.curr_speed = j[1]["speed"];
          	car_state.lane = lane;

          	// Previous path data given to the Planner
          	std::vector<double> previous_path_x = j[1]["previous_path_x"];
          	std::vector<double> previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	std::vector<std::vector<double>> sensor_fusion = j[1]["sensor_fusion"];

          	int prev_size = previous_path_x.size();

            if(prev_size > 0)
            {
              car_state.f_pos.s = end_path_s;
              car_state.f_pos.d = end_path_d;
            }

            bool too_close = false;

            prediction.update(sensor_fusion);

            static size_t counter = 1;

            CarState next_car = prediction.getNextCarInLane(lane, curr_car_s);

            if(next_car.isValid)
            {
              double check_car_s = next_car.f_pos.s + ((double)prev_size)*Simulator::INTERVAL*next_car.avgSpeed();

              if(check_car_s > car_state.f_pos.s) //Other car is before car
              {
                double distance_future = check_car_s-car_state.f_pos.s;
                double distance_now = next_car.f_pos.s - curr_car_s;

                if(distance_future < Configuration::SAFETY_DISTANCE)
                {
                  too_close = true;
                  target_speed = next_car.avgSpeed();
                }

                if(distance_future < Configuration::EMERGENCY_DISTANCE ||
                    distance_now <  Configuration::EMERGENCY_DISTANCE)
                {
                  std::cout << "EMERGENCY BREAK!\n";
                  too_close = true;
                  target_speed = 0.0;
                }
              }
            }

            if(counter%100 == 0)
            {
              std::vector<double> costs = costCalculation.getCostsForLanes(car_state);
              size_t optimalLane = std::distance(costs.begin(), std::min_element(costs.begin(), costs.end()));

              std::cout << "Lane costs : ";
              for(size_t target_lane=0; target_lane < Simulator::LANES_COUNT; ++target_lane)
              {
                std::cout << costs[target_lane] << " ";
              }
              std::cout << std::endl;

              if(optimalLane != lane)
              {

                int target_lane = lane;

                if(optimalLane > lane)
                  ++target_lane;
                else
                  --target_lane;

                CarState car_ahead = prediction.getNextCarInLane(target_lane, car_state.f_pos.s);
                CarState car_behind = prediction.getPreviousCarInLane(target_lane, car_state.f_pos.s);

                if(!car_ahead.isValid) //No car ahead
                {
                  if(!car_behind.isValid) //No car behind
                  {
                    lane = target_lane;
                  }
                  else //A car behind
                  {
                    double check_car_s_behind = car_behind.f_pos.s + ((double)prev_size)*Simulator::INTERVAL*car_behind.avgSpeed();

                    if(car_state.f_pos.s - check_car_s_behind >= Configuration::MIN_DIST_BEHIND)
                      lane = target_lane;
                  }
                }
                else // A car ahead
                {
                  double check_car_s_ahead = car_ahead.f_pos.s + ((double)prev_size)*Simulator::INTERVAL*car_ahead.avgSpeed();

                  if(!car_behind.isValid) //No car behind
                  {
                    if(check_car_s_ahead-car_state.f_pos.s >= Configuration::MIN_DIST_AHEAD)
                      lane = target_lane;
                  }
                  else //A car behind
                  {
                    double check_car_s_behind = car_behind.f_pos.s + ((double)prev_size)*Simulator::INTERVAL*car_behind.avgSpeed();

                    if(check_car_s_ahead-car_state.f_pos.s >= Configuration::MIN_DIST_AHEAD &&
                        car_state.f_pos.s - check_car_s_behind >= Configuration::MIN_DIST_BEHIND)
                      lane = target_lane;
                  }
                }
              }

            }
            ++counter;

            //Reset target speed
            if(!too_close && target_speed < Configuration::MAX_SPEED)
            {
              target_speed = Configuration::MAX_SPEED;
            }

            CartesianPath path = trajectory.generate(car_state, lane, target_speed, previous_path_x, previous_path_y);

            std::vector<double> next_x_vals;
            std::vector<double> next_y_vals;

            for(auto p : path)
            {
              next_x_vals.push_back(p.X);
              next_y_vals.push_back(p.Y);
            }
            json msgJson;
            msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;
          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
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
















































































