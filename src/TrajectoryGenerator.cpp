/*
 * TrajectoryGenerator.cpp
 *
 *  Created on: 05.08.2017
 */

#include "TrajectoryGenerator.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::tuple<CartesianPath, FrenetPath> TrajectoryGenerator::generate(FrenetState start, FrenetState stop, double time)
{
  std::normal_distribution<double> distribution_s(stop.position.s,1.0*time);
  std::normal_distribution<double> distribution_d(stop.position.d,0.1);

  std::tuple<CartesianPath, FrenetPath> best_path;
  double best_cost = 1000;

  for(size_t i=0; i < 10; ++i)
  {
    double end_s = distribution_s(_rnd_gen) - start.position.s;
    double end_d = distribution_d(_rnd_gen) - start.position.d;

    //Calculate estimated time
    double delta_v = start.velocity.s - stop.velocity.s;

    double time_for_s = std::fabs(end_s)/((start.velocity.s+stop.velocity.s)/2.0);
    double time_for_v = std::fabs(delta_v)/(10);
    double time_for_d = std::fabs(end_d)/(2.0);

    time = std::max(time_for_s, std::max(time_for_v, time_for_d));

    std::cout << "Caluclation target " << end_s << "," << end_d << " in " << time << std::endl;


    auto coeff_s = JMT(std::vector<double>{0, start.velocity.s, 0}, std::vector<double>{end_s, stop.velocity.s, 0}, time);
    auto coeff_d = JMT(std::vector<double>{0, start.velocity.d, 0}, std::vector<double>{end_d, stop.velocity.d, 0}, time);

    auto frenet_path = calculate(coeff_s, coeff_d, time, start, stop);
    auto cartesian_path = transform(frenet_path);

    double cost = _costs.calculate(frenet_path, cartesian_path);

    if(cost < best_cost)
    {
      best_path = std::make_tuple(cartesian_path, frenet_path);
      best_cost = cost;
    }
  }

  std::cout << "Choosen path cost=" << best_cost << std::endl;
  return best_path;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Path<double> TrajectoryGenerator::JMT(std::vector< double> start, std::vector <double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.
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
FrenetPath TrajectoryGenerator::calculate(std::vector<double> const& c_s, std::vector<double> const& c_d,
                                          double time, FrenetState const& start, FrenetState const& stop)
{
  std::vector<FrenetPoint> result;

  bool sameLane = false;
  if((start.position.d - stop.position.d) < 1.0)
    sameLane = true;

  for(double t=0; t <= time; t+=_timeDelta)
  {
    FrenetPoint p;
    p.s = c_s[0] + c_s[1]*t + c_s[2]*std::pow(t,2) + c_s[3]*std::pow(t,3)+ c_s[4]*std::pow(t,4) + c_s[5]*std::pow(t,5) + start.position.s;
    if(!sameLane)
      p.d = c_d[0] + c_d[1]*t + c_d[2]*std::pow(t,2) + c_d[3]*std::pow(t,3)+ c_d[4]*std::pow(t,4) + c_d[5]*std::pow(t,5) + start.position.d;
    else //otherwise it's wiggles!
      p.d = start.position.d + ((stop.position.d-start.position.d)/(time/_timeDelta))*t;

    result.emplace_back(p);
  }
  return result;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
CartesianPath TrajectoryGenerator::transform(FrenetPath & path)
{
  std::vector<CartesianPoint> result;
  for(auto point : path)
  {
    CartesianPoint p(_map.TransformFrenetToCartesian(point.s, point.d));
    result.emplace_back(p);
  }

  return result;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
