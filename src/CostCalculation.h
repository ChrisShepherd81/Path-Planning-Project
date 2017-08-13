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

  std::vector<double> getCostsForLanes(const CarState& car_state);

 private:
  double getCostForLane(const CarState& car_state, size_t lane);
  void getAllNextCars(double curr_car_s);
  double distanceAheadCost(const CarState& car_state, size_t lane);
  double distanceBehindCost(const CarState& car_state, size_t lane);
  double speedBehindCost(const CarState& car_state, size_t lane);
  double speedAheadCost(const CarState& car_state, size_t lane);
  void reset();
  void setMaxDistAhead(double distance);
  void setMaxDistBehind(double distance);

  const SensorFusion& _sensorFusion;
  double MIN_COST = 0.0;
  double MAX_COST = 1.0;
  double LANE_CHANGE_COST = 0.1;
  double PREFER_MIDDLE_LANE = 0.05;

  std::vector<CarState> _carsAhead;
  std::vector<CarState> _carsBehind;
  double _maxDistAhead;
  double _maxDistBehind;


};

#endif /* SRC_COSTCALCULATION_H_ */
