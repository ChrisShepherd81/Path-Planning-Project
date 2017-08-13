/*
 * SensorFusion.h
 *
 *  Created on: 30.07.2017
 */

#ifndef SRC_SENSORFUSION_H_
#define SRC_SENSORFUSION_H_

#include "Path.h"

#include <vector>
#include <iostream>
#include <cmath>

class SensorFusion {

 public:
  void update(std::vector<std::vector<double>> sensorFusionData);
  CarState getNextCarInLane(size_t lane, double s) const;
  CarState getPreviousCarInLane(size_t lane, double s) const;

 private:
  CarState getCarInLane(size_t lane, double s, std::function<bool(double, double)> &op) const;
  void updateCar(CarState& car, std::vector<double> &raw_data);

  std::vector<CarState> other_cars;

};

#endif /* SRC_SENSORFUSION_H_ */
