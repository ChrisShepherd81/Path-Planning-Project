/*
 * PathPlanning.h
 *
 *  Created on: 13.08.2017
 */

#ifndef SRC_PATHPLANNING_H_
#define SRC_PATHPLANNING_H_

#include "SensorFusion.h"
#include "CostCalculation.h"

class PathPlanning {
 public:
  PathPlanning(SensorFusion& sensorFusion);
  size_t getTargetLane(CarState car_state, size_t prev_size);

 private:
  const SensorFusion& _sensorFusion;
  CostCalculation _costCalculation;
};

#endif /* SRC_PATHPLANNING_H_ */
