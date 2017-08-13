/*
 * Trajectory.h
 *
 *  Created on: 13.08.2017
 */

#ifndef SRC_TRAJECTORY_H_
#define SRC_TRAJECTORY_H_

#include "TrajectoryGenerator.h"

class Trajectory
{
 public:
  Trajectory(SensorFusion& sensorFusion, GlobalMap& map) : _sensorFusion(sensorFusion), _trajectoryGenerator(map) {}

  CartesianPath getNextPath(CarState car_state, double curr_car_s, CartesianPath previous_path, int target_lane);
 private:
  SensorFusion& _sensorFusion;
  TrajectoryGenerator _trajectoryGenerator;
};



#endif /* SRC_TRAJECTORY_H_ */
