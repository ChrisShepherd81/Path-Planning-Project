/*
 * Prediction.h
 *
 *  Created on: 30.07.2017
 */

#ifndef SRC_PREDICTION_H_
#define SRC_PREDICTION_H_

#include <vector>
#include <iostream>
#include <cmath>


struct Car
{
  size_t id = -1;
  size_t lane = -1;
  double speed = 0;
  bool isValid = false;

  std::vector<double> s_vals;
  std::vector<double> d_vals;
  double getLastS() const
  {
    return s_vals.back();
  }
  double getLastD() const
  {
    return d_vals.back();
  }

  void print(double pos_s)
  {
    std::cout << "Car " << id << " speed: " << speed << " in "
        << getLastS() - pos_s <<  "m in lane " << lane << "\n";

  }
};

class Prediction {

 public:
  void update(std::vector<double> &other_car);
  Car getNextCarInLane(size_t lane, double s);
  Car getPreviousCarInLane(size_t lane, double s);

 private:
  Car getCarInLane(size_t lane, double s, std::function<bool(double, double)> &op);
  void updateCar(Car& car, std::vector<double> &raw_data);

  std::vector<Car> other_cars;

};

#endif /* SRC_PREDICTION_H_ */
