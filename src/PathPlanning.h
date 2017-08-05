/*
 * PathPlanning.h
 *
 *  Created on: 01.08.2017
 */

#ifndef SRC_PATHPLANNING_H_
#define SRC_PATHPLANNING_H_

#include "Trajectory.h"
#include "Prediction.h"

class PathPlanning {

 public:
  PathPlanning(Trajectory& tracectory, Prediction &prediction) :
    _trajectory(tracectory),
    _prediction(prediction) {}

  void plan();

 private:
  void updateCarPosition();

  Trajectory::State getManeuver(size_t targetLane, size_t currentLane)
   {
     //std::cout << "From lane " << currentLane << " to " << targetLane;
     if(targetLane >= (currentLane + 1))
       return Trajectory::State::ChangeLaneRight;
     if(targetLane <= (currentLane - 1))
       return Trajectory::State::ChangeLaneLeft;
       
     return Trajectory::State::KeepLane;
   }

   Trajectory& _trajectory;
   Prediction& _prediction;
   double _car_s;
   size_t _lane;
   double _targetSpeed = 21.5; // meters per second;;

};

#endif /* SRC_PATHPLANNING_H_ */
