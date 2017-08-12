/*
 * TrajectoryGenerator.cpp
 *
 *  Created on: 05.08.2017
 */

#include "TrajectoryGenerator.h"

constexpr double Simulator::LANE_D_VALUE [];

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Path<double> TrajectoryGenerator::JMT(std::vector< double> start, std::vector <double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.
    !!! Does not really work well with simulator so only splines are used !!!
    */

    double T2 = T*T;
    double T3 = T2*T;
    double T4 = T2*T2;

    Eigen::MatrixXd A = Eigen::MatrixXd(3, 3);
    A << T3,   T4,    T4*T,
         3*T2, 4*T3,  5*T4,
         6*T,  12*T2, 20*T3;

    Eigen::MatrixXd B = Eigen::MatrixXd(3,1);
    B << end[0]-(start[0]+start[1]*T+.5*start[2]*T2),
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
CartesianPath TrajectoryGenerator::generate(CarState car, int target_lane, double target_speed,
                          std::vector<double> &previous_path_x, std::vector<double> &previous_path_y)
{
  std::vector<double> ptsx;
  std::vector<double> ptsy;

  double ref_x = car.c_pos.X;
  double ref_y = car.c_pos.Y;
  double ref_yaw = deg2rad(car.Yaw);
  double ref_vel = 0;

  int prev_size = previous_path_x.size();

  if(prev_size < 2)
  {
    double prev_car_x = car.c_pos.X - cos(car.Yaw);
    double prev_car_y = car.c_pos.Y - sin(car.Yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car.c_pos.X);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car.c_pos.Y);
  }
  else
  {
    ref_x = previous_path_x[prev_size-1];
    ref_y = previous_path_y[prev_size-1];

    double ref_x_prev = previous_path_x[prev_size-2];
    double ref_y_prev = previous_path_y[prev_size-2];

    ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);

    ref_vel = (distance(ref_x_prev, ref_y_prev, ref_x, ref_y )/Simulator::INTERVAL);
  }

  for(size_t i=1; i <= 3; ++i)
  {
    CartesianPoint next_wp = _map.TransformFrenetToCartesian(car.f_pos.s+(30*i), Simulator::LANE_D_VALUE[target_lane]);
    ptsx.push_back(next_wp.X);
    ptsy.push_back(next_wp.Y);

  }

  //Transformation
  for(int i=0; i < ptsx.size(); ++i)
  {
    double shift_x = ptsx[i]-ref_x;
    double shift_y = ptsy[i]-ref_y;

    ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
    ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
  }

  tk::spline s;

  s.set_points(ptsx, ptsy);

  std::vector<double> next_x_vals;
  std::vector<double> next_y_vals;

  //copy last path
  for(int i=0; i < previous_path_x.size(); ++i)
  {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt((target_x*target_x)+(target_y*target_y));

  double x_add_on = 0;

  for(int i = 0; i < 50-prev_size; i++)
  {
    if(ref_vel < Configuration::MAX_SPEED && ref_vel < target_speed)
      ref_vel += Simulator::MAX_SPEED_CHANGE;
    else if(ref_vel > target_speed )
      ref_vel -= Simulator::MAX_SPEED_CHANGE;
    else
      ref_vel = Configuration::MAX_SPEED;

    double N = (target_dist/(Simulator::INTERVAL*ref_vel)); //mph -> mps
    double x_point = x_add_on+target_x/N;
    double y_point = s(x_point);

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

  CartesianPath result;

  for(size_t i=0; i < next_x_vals.size(); ++i)
  {
    result.push_back(CartesianPoint{next_x_vals[i], next_y_vals[i]});
  }

  return result;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
