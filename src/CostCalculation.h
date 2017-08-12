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

#include "Prediction.h"


class CostCalculation {

 public:

  CostCalculation(Prediction& prediction) : _prediction(prediction) {}

  std::vector<double> getCostsForLanes(CarState car_state);

 private:
  const Prediction& _prediction;
  double MIN_COST = 0.0;
  double MAX_COST = 1.0;
  double CHANGE_COST = 0.05;


};

#endif /* SRC_COSTCALCULATION_H_ */
