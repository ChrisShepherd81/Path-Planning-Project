/*
 * PathPlanning.cpp
 *
 *  Created on: 01.08.2017
 */

#include "PathPlanning.h"
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PathPlanning::updateCarPosition()
{
  auto car_pos = _trajectory.getCarPosition();
  _car_s = car_pos[0];
  _lane = std::floor(car_pos[1]/4.0);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PathPlanning::plan()
{
  updateCarPosition();

  Car nextCar = _prediction.getNextCarInLane(_lane,_car_s);
  if(!nextCar.isValid) //No next car in current lane
    _trajectory.updateState(Trajectory::KeepLane);
  else
  {
    //Keep lane if next car is faster equals target speed
    if(nextCar.speed >= _targetSpeed)
      _trajectory.updateState(Trajectory::KeepLane);
    else
    {
      //_prediction.
      //Try Change lane
      //Get NextCar in other lanes
      std::vector<Car> nextCars;
      for(size_t i = 0; i < 3; ++i)
      {
        if(i != _lane)
          nextCars.push_back(_prediction.getNextCarInLane(i, _car_s));

        for(auto neighborCar : nextCars)
        {
          if(!neighborCar.isValid) //Free lane
          {
            //Check if neighbor lane

            _trajectory.updateState(getManeuver(neighborCar.lane, _lane));
            return;
          }

          //Check if car in neighbor lanes is faster
          if(neighborCar.speed > nextCar.speed )
          {
            if(neighborCar.getLastS() > _car_s + 25 )
            {
             _trajectory.updateState(getManeuver(neighborCar.lane, _lane));
             return;
            }
          }

        }
      }

    }

    _trajectory.updateState(Trajectory::State::KeepLane);
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
