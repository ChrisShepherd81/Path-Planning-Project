/*
 * Trajectory.h
 *
 *  Created on: 30.07.2017
 */

#ifndef SRC_TRAJECTORY_H_
#define SRC_TRAJECTORY_H_

#include "TrajectoryGenerator.h"
#include "GlobalMap.h"
#include "Prediction.h"

#include "Eigen-3.3/Eigen/Dense"

#include <tuple>
#include <vector>
#include <deque>

class Trajectory {
public:
  enum State { KeepLane, ChangeLaneLeft, ChangeLaneRight};

  Trajectory(GlobalMap &globalMap, Prediction& predict) : _state(KeepLane), prediction(predict), generator(globalMap) {}

  std::tuple<std::vector<double>, std::vector<double>> update(
      std::vector<double> &prev_path_x,
      std::vector<double> &prev_path_y,
      double car_s, double car_d, double car_yaw);

  void updateState(State state)
  {
    return;

    if(!_stateChangeInProgress)
    {
      _state = state;
      return;
    }

    //std::cout << "State " << state << " not applied, still in progress\n";
  }

  std::vector<double> getCarPosition() const
  {
    return std::vector<double>{s_vals.front(), d_vals.front()};
  }

 private:
  State _state;

  void storeFrenetPoint(double s, double d);
  size_t currentLane(double d)
  {
    std::cout << "Car d pos is " << d << std::endl;
    if(d <= 2.0)
      return 0;
    if(d <= 6.0)
      return 1;
    if(d <= 10)
      return 2;

    throw "no lane exeption";
  }

  std::deque<double> s_vals;
  std::deque<double> d_vals;

  Prediction& prediction;
  TrajectoryGenerator generator;

  const double TARGET_SPEED = 21.5; // meters per second
  double _target_speed = TARGET_SPEED;
  const double INTERVAL = 0.02; // meters per second
  const size_t WAY_POINTS = 50;
  const double SAFETY_DISTANCE = 20; //meters
  const double SAFETY_DISTANCE_BEHIND = 5.0;
  const double SAFETY_DISTANCE_AHEAD = 10.0;
  const double MAX_ACC_IN_S = 5.0;
  const double MAX_ACC_IN_D = 2.0;
  size_t lastWayPoints = 0;
  bool _stateChangeInProgress = false;
  size_t _targetLane = 1;
  bool _laneShiftQueued = true;

};

#endif /* SRC_TRAJECTORY_H_ */
