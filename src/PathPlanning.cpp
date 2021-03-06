/*
 * PathPlanning.cpp
 *
 *  Created on: 13.08.2017
 */

#include "PathPlanning.h"
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PathPlanning::PathPlanning(SensorFusion& sensorFusion) :  _sensorFusion(sensorFusion), _costCalculation(sensorFusion)
{
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
size_t PathPlanning::getTargetLane(const CarState& car_state, double horizont_time)
{
  size_t resultLane = car_state.lane;
  std::vector<double> costs = _costCalculation.getCostsForLanes(car_state);
  size_t optimalLane = std::distance(costs.begin(), std::min_element(costs.begin(), costs.end()));

  if(optimalLane != car_state.lane)
  {
    size_t target_lane = car_state.lane;

    if(optimalLane > car_state.lane)
      ++target_lane;
    else
      --target_lane;

    CarState car_ahead = _sensorFusion.getNextCarInLane(target_lane, car_state.f_pos.s);
    CarState car_behind = _sensorFusion.getPreviousCarInLane(target_lane, car_state.f_pos.s);

    if(!car_ahead.isValid) //No car ahead
    {
      if(!car_behind.isValid) //No car behind
      {
        resultLane = target_lane;
      }
      else //A car behind
      {
        double check_car_s_behind = car_behind.f_pos.s + horizont_time*car_behind.avgSpeed();

        if(car_state.f_pos.s - check_car_s_behind >= Configuration::MIN_DIST_BEHIND)
          resultLane = target_lane;
      }
    }
    else // A car ahead
    {
      double check_car_s_ahead = car_ahead.f_pos.s + horizont_time*car_ahead.avgSpeed();

      if(!car_behind.isValid) //No car behind
      {
        if(check_car_s_ahead-car_state.f_pos.s >= Configuration::MIN_DIST_AHEAD)
          resultLane = target_lane;
      }
      else //A car behind
      {
        double check_car_s_behind = car_behind.f_pos.s + horizont_time*car_behind.avgSpeed();

        if(check_car_s_ahead-car_state.f_pos.s >= Configuration::MIN_DIST_AHEAD &&
            car_state.f_pos.s - check_car_s_behind >= Configuration::MIN_DIST_BEHIND)
          resultLane = target_lane;
      }
    }
  }

  return resultLane;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
