/*
 * CostCaculation.h
 *
 *  Created on: 05.08.2017
 */

#ifndef SRC_COSTCALCULATION_H_
#define SRC_COSTCALCULATION_H_

#include "Configuration.h"
#include "Path.h"
#include <iostream>

#include "SensorFusion.h"


class CostCalculation {

 public:

  CostCalculation(SensorFusion& prediction) : _sensorFusion(prediction) {}

  std::vector<double> getCostsForLanes(CarState car_state);

 private:
  const SensorFusion& _sensorFusion;
  double MIN_COST = 0.0;
  double MAX_COST = 1.0;
  double CHANGE_COST = 0.05;


};

#endif /* SRC_COSTCALCULATION_H_ */
