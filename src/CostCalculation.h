/*
 * CostCaculation.h
 *
 *  Created on: 05.08.2017
 */

#ifndef SRC_COSTCALCULATION_H_
#define SRC_COSTCALCULATION_H_

#include "Path.h"
#include <iostream>

#define MPH_TO_MPS 0.44704

class CostCalculation {

 public:

  double calculate(FrenetPath const& frenetPath, CartesianPath const& cartesianPath );

  double jerkInDCost(FrenetPath const& frenetPath);
  double speedCost(CartesianPath const& cartesianPath);

 private:
  const double MAX_COST = 1.0;
  const double MIN_COST = 0.0;
  const double MAX_SPEED = 49.5 * MPH_TO_MPS;
  const double MAX_ACC = 9.5;
  const double deltaTime = 0.02;

};

#endif /* SRC_COSTCALCULATION_H_ */
