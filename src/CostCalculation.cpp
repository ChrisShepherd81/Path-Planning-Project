/*
 * CostCalculation.cpp
 *
 *  Created on: 05.08.2017
 */

#include "CostCalculation.h"
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double CostCalculation::calculate(FrenetPath const& frenetPath, CartesianPath const& cartesianPath )
{
  //std::cout << "Calculate costs\n";
  return 10*speedCost(cartesianPath) + jerkInDCost(frenetPath);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double CostCalculation::jerkInDCost(FrenetPath const& frenetPath)
{
  double max_diff_in_d = 0;
  for(size_t i = 1; i < frenetPath.size(); ++i)
  {
    double diff_in_d = frenetPath[i].d - frenetPath[i-1].d ;
    if(diff_in_d > max_diff_in_d)
    {
      max_diff_in_d = diff_in_d;
      //std::cout << "Max diff in d" << max_diff_in_d << std::endl;
    }
  }

  return max_diff_in_d;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double CostCalculation::speedCost(CartesianPath const& cartesianPath)
{
  double max_speed = 0;
  double average_speed = 0;
  double max_acceleration = 0;
  double last_speed = 0;
  for(size_t i = 1; i < cartesianPath.size(); ++i)
  {
    double speed = cartesianPath[i].distanceTo(cartesianPath[i-1]) / deltaTime;
    average_speed += speed/(cartesianPath.size()-1);

    if(last_speed == 0)
      last_speed = speed;

    double acc = std::abs(last_speed-speed)/ deltaTime;
    last_speed = speed;

    if(speed > max_speed)
      max_speed = speed;

    if(acc > max_acceleration)
      max_acceleration = acc;

  }

  if(max_speed > MAX_SPEED)
  {
    std::cout << "Speed exceeded\n";
    return MAX_COST;
  }

  if(max_acceleration > MAX_ACC)
  {
    std::cout << "ACC exceeded\n";
    return MAX_COST;
  }


  double cost = ((MAX_SPEED-average_speed)/MAX_SPEED + (MAX_ACC-max_acceleration)/MAX_ACC)/2.0;
  std::cout << "Accepted Path cost " << cost << "\n";

  return cost;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
