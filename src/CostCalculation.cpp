/*
 * CostCalculation.cpp
 *
 *  Created on: 05.08.2017
 */

#include "CostCalculation.h"
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<double> CostCalculation::getCostsForLanes(CarState car_state)
{
  std::vector<double> costs = {MIN_COST, MIN_COST, MIN_COST};
  std::vector<double> distances = {0, 0, 0};
  double max_dist = 0;

  for(size_t lane=0; lane < Simulator::LANES_COUNT; ++lane)
  {
    if(lane != car_state.lane)
      costs[lane] += CHANGE_COST;

    CarState next_car = _sensorFusion.getNextCarInLane(lane, car_state.f_pos.s);

    if(!next_car.isValid) //No car in lane ahead
      continue;

    double distance = next_car.f_pos.s - car_state.f_pos.s;
    double speed_cost = 1.5*(Configuration::MAX_SPEED - next_car.curr_speed) / Configuration::MAX_SPEED;

    if(distance > max_dist)
      max_dist = distance;

    costs[lane] += speed_cost >= 0 ? speed_cost : 0;
    distances[lane] = distance;
  }

  for(size_t lane=0; lane < Simulator::LANES_COUNT; ++lane)
  {
    if(distances[lane] > 0 && max_dist > 0)
    {
      costs[lane] += (MAX_COST - (distances[lane]/max_dist));
    }
  }
  return costs;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
