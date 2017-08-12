/*
 * Prediction.h
 *
 *  Created on: 30.07.2017
 */

#ifndef SRC_PREDICTION_H_
#define SRC_PREDICTION_H_

#include "Path.h"

#include <vector>
#include <iostream>
#include <cmath>

class Prediction {

 public:
  void update(std::vector<std::vector<double>> &other_cars);
  CarState getNextCarInLane(size_t lane, double s) const;
  CarState getPreviousCarInLane(size_t lane, double s) const;

 private:
  CarState getCarInLane(size_t lane, double s, std::function<bool(double, double)> &op) const;
  void updateCar(CarState& car, std::vector<double> &raw_data);

  std::vector<CarState> other_cars;

};

#endif /* SRC_PREDICTION_H_ */
