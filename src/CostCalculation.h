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

struct Configuration
{
  static constexpr double MAX_SPEED = 49.0 * MPH_TO_MPS;
  static constexpr double MAX_ACC = 9.5;
  static constexpr double INTERVAL = 0.02;
};

struct Costs
{
  static constexpr double MAX_COST = 1.0;
  static constexpr double MIN_COST = 0.0;

  double avgSpeedCost = MAX_COST;
  double maxSpeedCost = MAX_COST;
  double accelerationCosts = MAX_COST;
  double total()
  {
    return (avgSpeedCost+accelerationCosts+maxSpeedCost)/3.0;
  }

};

class CostCalculation {

 public:

  Costs calculate(FrenetPath const& frenetPath, CartesianPath const& cartesianPath );

  void jerkInDCost(FrenetPath const& frenetPath, Costs &result);
  void speedCost(CartesianPath const& cartesianPath, Costs &result);

 private:
  const double MAX_SPEED = 49.0 * MPH_TO_MPS;
  const double MAX_ACC = 9.5;
  const double deltaTime = 0.02;

};

#endif /* SRC_COSTCALCULATION_H_ */
