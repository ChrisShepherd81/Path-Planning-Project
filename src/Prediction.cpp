/*
 * Prediction.cpp
 *
 *  Created on: 30.07.2017
 */

#include "Prediction.h"
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Prediction::update(std::vector<double> &other_car)
{
  // [ id, x, y, vx, vy, s, d]
  size_t idx = other_car[0];

  if(other_cars.size() > idx)
  {
    updateCar(other_cars[idx], other_car);
  }
  else
  {
    //Create new car
    Car new_car;
    updateCar(new_car, other_car);
    other_cars.insert(other_cars.begin()+idx, new_car);
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Prediction::updateCar(Car& car, std::vector<double> &raw_data)
{
  // [ id, x, y, vx, vy, s, d]
  size_t idx = raw_data[0];
  double vx = raw_data[3];
  double vy = raw_data[4];
  double s = raw_data[5];
  double d = raw_data[6];

  car.id = idx;

  car.s_vals.emplace_back(s);
  car.d_vals.emplace_back(d);

  if(d < 0)
  {
//    if(car.isValid)
//      std::cout << "Car " << car.id << " got unvalid\n";

    car.isValid = false;
    car.lane = -1;
  }
  else
  {
    car.lane = std::floor(d/4.0);
    car.isValid = true;
  }

  car.speed = std::sqrt(std::pow(vx,2.0)+std::pow(vy,2.0));

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Car Prediction::getNextCarInLane(size_t lane, double s)
{
  std::function<bool(double, double)> op = [](double a, double b){ return a > b; };
  return getCarInLane(lane, s, op);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Car Prediction::getPreviousCarInLane(size_t lane, double s)
{
  std::function<bool(double, double)> op = [](double a, double b){ return a < b; };
  return getCarInLane(lane, s, op);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Car Prediction::getCarInLane(size_t lane, double s, std::function<bool(double, double)> &op)
{
  Car result;
  result.isValid = false;
  result.lane = lane;

  for(auto car : other_cars)
  {
    //Car is not valid nor on same lane
    if(!car.isValid || car.lane != lane)
      continue;

    //Set result car
    if(!result.isValid && op(car.getLastS(),s))
    {
      result = car;
      continue;
    }

    //Update result car
    if(op(car.getLastS(),s) && op(result.getLastS(),car.getLastS()))
    {
      result = car;
      continue;
    }
  }

  return result;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
