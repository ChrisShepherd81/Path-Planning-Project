/*
 * CostCalculation.cpp
 *
 *  Created on: 05.08.2017
 */

#include "CostCalculation.h"
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<double> CostCalculation::getCostsForLanes(const CarState& car_state)
{
  std::vector<double> costs = {MIN_COST, MIN_COST, MIN_COST};

  getAllNextCars(car_state.curr_s);

  for(size_t i=0; i < costs.size(); ++i)
    costs[i] += getCostForLane(car_state, i);

  return costs;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CostCalculation::getAllNextCars(double curr_car_s)
{
  reset();

  for(size_t lane=0; lane < Simulator::LANES_COUNT; ++lane)
  {
    CarState car_ahead = _sensorFusion.getNextCarInLane(lane, curr_car_s);
    CarState car_behind = _sensorFusion.getPreviousCarInLane(lane, curr_car_s);

    _carsAhead.push_back(car_ahead);
    _carsBehind.push_back(car_behind);

    if(car_ahead.isValid)
      setMaxDistAhead(car_ahead.f_pos.s - curr_car_s);
    if(car_behind.isValid)
      setMaxDistBehind(car_behind.f_pos.s - curr_car_s);
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CostCalculation::reset()
{
  _carsAhead.clear();
  _carsBehind.clear();
  _maxDistAhead = 0;
  _maxDistBehind = 0;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CostCalculation::setMaxDistAhead(double distance)
{
  if(_maxDistAhead < distance)
    _maxDistAhead = distance;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CostCalculation::setMaxDistBehind(double distance)
{
  if(_maxDistBehind < distance)
    _maxDistBehind = distance;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double CostCalculation::getCostForLane(const CarState& car_state, size_t lane)
{
  double cost = 0;

  if(car_state.lane != lane)
    cost += LANE_CHANGE_COST;

  if(lane != Simulator::MIDDLE_LANE)
    cost += PREFER_MIDDLE_LANE;

  cost += 3*distanceAheadCost(car_state, lane);
  cost += 0.5*distanceBehindCost(car_state, lane);
  cost += 2*speedAheadCost(car_state, lane);
  cost += 0.5*speedBehindCost(car_state, lane);

  return cost;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double CostCalculation::distanceAheadCost(const CarState& car_state, size_t lane)
{
  if(_carsAhead[lane].isValid && _maxDistAhead > 0)
    return (MAX_COST - (_carsAhead[lane].f_pos.s - car_state.curr_s)/_maxDistAhead );
  return MIN_COST;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double CostCalculation::distanceBehindCost(const CarState& car_state, size_t lane)
{
  if(_carsBehind[lane].isValid && _maxDistBehind > 0)
    return (MAX_COST - (car_state.curr_s - _carsBehind[lane].f_pos.s)/_maxDistBehind );
  return MIN_COST;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double CostCalculation::speedBehindCost(const CarState& car_state, size_t lane)
{
  double cost = MIN_COST;
  if(_carsBehind[lane].isValid)
    cost = (_carsBehind[lane].curr_speed) / Configuration::MAX_SPEED;
  return cost >= 0 ? cost : MIN_COST;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double CostCalculation::speedAheadCost(const CarState& car_state, size_t lane)
{
  double cost = MIN_COST;
  if(_carsAhead[lane].isValid)
    cost = (Configuration::MAX_SPEED - _carsAhead[lane].curr_speed) / Configuration::MAX_SPEED;
  return cost >= 0 ? cost : MIN_COST;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
