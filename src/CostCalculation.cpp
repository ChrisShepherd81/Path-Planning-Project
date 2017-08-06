/*
 * CostCalculation.cpp
 *
 *  Created on: 05.08.2017
 */

#include "CostCalculation.h"
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Costs CostCalculation::calculate(FrenetPath const& frenetPath, CartesianPath const& cartesianPath )
{
  //std::cout << "Calculate costs\n";
  Costs result;
  speedCost(cartesianPath, result);
  jerkInDCost(frenetPath, result);
  return result;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CostCalculation::jerkInDCost(FrenetPath const& frenetPath, Costs &result)
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

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CostCalculation::speedCost(CartesianPath const& cartesianPath, Costs &result)
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
    result.maxSpeedCost = Costs::MAX_COST;
  }
  else
    result.maxSpeedCost = Costs::MIN_COST;

  if(max_acceleration > MAX_ACC)
  {
    //std::cout << "ACC exceeded\n";
    result.accelerationCosts = Costs::MAX_COST;
  }

  result.avgSpeedCost = (MAX_SPEED-average_speed)/MAX_SPEED;
  result.accelerationCosts = (MAX_ACC-max_acceleration)/MAX_ACC;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
