/*
 * Trajectory.cpp
 *
 *  Created on: 30.07.2017
 */

#include "Trajectory.h"
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::tuple<std::vector<double>, std::vector<double>> Trajectory::update(
    std::vector<double> &prev_path_x,
    std::vector<double> &prev_path_y,
    double car_s, double car_d, double car_yaw,
    size_t lane, bool &laneShift)
{
  double pos_s, pos_d;
  int path_size = prev_path_x.size();
  std::vector<double> next_x_vals;
  std::vector<double> next_y_vals;

  for(int i = 0; i < path_size; i++)
  {
     next_x_vals.push_back(prev_path_x[i]);
     next_y_vals.push_back(prev_path_y[i]);
  }

  std::cout << path_size << std::endl;
  if(path_size == 0)
  {
     pos_s = car_s;
     pos_d = car_d;
  }
  else
  {
     for(size_t i = path_size; i < lastWayPoints; ++i )
     {
       s_vals.pop_front();
       d_vals.pop_front();
     }

//     double pos_x = prev_path_x[path_size-1];
//     double pos_y = prev_path_y[path_size-1];
//
//     double pos_x2 = prev_path_x[path_size-2];
//     double pos_y2 = prev_path_y[path_size-2];
//     angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
//
//     auto frenet = globalMap.TransformCartesianToFrenet(pos_x, pos_y, angle);
     pos_s = s_vals.back();
     pos_d = d_vals.back();
  }

  //TODO use JMT with different times to not exceed limits.
  double dist_inc = 0.43;
  if(laneShift)
  {
    //Remove previous path
    size_t start = 10;
    next_x_vals.erase(next_x_vals.begin()+start, next_x_vals.end());
    next_y_vals.erase(next_y_vals.begin()+start, next_y_vals.end());
    s_vals.erase(s_vals.begin()+start, s_vals.end());
    d_vals.erase(d_vals.begin()+start, d_vals.end());

    pos_s = s_vals.back();
    pos_d = d_vals.back();

    double totalTime = 1.75;

    auto a = JMT(std::vector< double>{pos_d, 0,0}, std::vector <double>{2.0 + (4*lane),0,0}, totalTime);
    for(int i = 0; i < totalTime/0.02; i++)
    {
      double t = (i+1)*0.02;
      double d = a[0] + a[1] * t + a[2] * t*t + a[3] * t*t*t + a[4] * t*t*t*t + a[5] * t*t*t*t*t;
      double s = pos_s+((i+1)*dist_inc);
      auto coord = globalMap.TransformFrenetToCartesian(s, d);
      next_x_vals.push_back(coord[0]);
      next_y_vals.push_back(coord[1]);
      storeFrenetPoint(s, d);
    }
    laneShift = false;
  }
  else
  {
    //straight path constant speed
    if(path_size < WAY_POINTS)
    {
      for(size_t i = 0; i < WAY_POINTS-path_size; i++)
      {
         double s = pos_s+((i+1)*dist_inc);
         double d = 2.0 + (4*lane);
         auto coord = globalMap.TransformFrenetToCartesian(s, d);
         next_x_vals.push_back(coord[0]);
         next_y_vals.push_back(coord[1]);
         storeFrenetPoint(s, d);
      }
    }
  }

  lastWayPoints = next_x_vals.size();
  return std::make_tuple(next_x_vals, next_y_vals);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<double> Trajectory::JMT(std::vector< double> start, std::vector <double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT
    an array of length 6, each value corresponding to a coefficent in the polynomial
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */

    Eigen::MatrixXd A = Eigen::MatrixXd(3, 3);
    A << T*T*T, T*T*T*T, T*T*T*T*T,
         3*T*T, 4*T*T*T, 5*T*T*T*T,
           6*T, 12*T*T,  20*T*T*T;

    Eigen::MatrixXd B = Eigen::MatrixXd(3,1);
    B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
          end[1]-(start[1]+start[2]*T),
          end[2]-start[2];

    Eigen::MatrixXd Ai = A.inverse();

    Eigen::MatrixXd C = Ai*B;

    std::vector <double> result = {start[0], start[1], .5*start[2]};
    for(int i = 0; i < C.size(); i++)
    {
        result.push_back(C.data()[i]);
    }

    return result;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Trajectory::storeFrenetPoint(double s, double d)
{
  s_vals.push_back(s);
  d_vals.push_back(d);
}
