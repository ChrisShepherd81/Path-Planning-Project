/*
 * Trajectory.cpp
 *
 *  Created on: 13.08.2017
 */

#include "Trajectory.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
CartesianPath Trajectory::getNextPath(CarState car_state, CartesianPath previous_path, int target_lane)
{
  bool too_close = false;
  double target_speed = Configuration::MAX_SPEED;

  CarState next_car = _sensorFusion.getNextCarInLane(car_state.lane, car_state.curr_s);

  if(next_car.isValid)
  {
    double check_car_s = next_car.f_pos.s + ((double)previous_path.size())*Simulator::INTERVAL*next_car.avgSpeed();

    if(check_car_s > car_state.f_pos.s) //Other car is before car
    {
      double distance_future = check_car_s-car_state.f_pos.s;
      double distance_now = next_car.f_pos.s - car_state.curr_s;

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

  //Reset target speed
  if(!too_close && target_speed < Configuration::MAX_SPEED)
  {
    target_speed = Configuration::MAX_SPEED;
  }

  return _trajectoryGenerator.generate(car_state, target_lane, target_speed, previous_path);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



