/*
 * Prediction.cpp
 *
 *  Created on: 30.07.2017
 */

#include "SensorFusion.h"
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SensorFusion::update(std::vector<std::vector<double>> sensorFusionData)
{
  //find
  for(size_t i=0; i < sensorFusionData.size(); ++i)
  {
    // [ id, x, y, vx, vy, s, d]
    size_t idx = sensorFusionData[i][0];

    if(other_cars.size() > idx)
    {
      updateCar(other_cars[idx], sensorFusionData[i]);
    }
    else
    {
      //Create new car
      CarState new_car;
      updateCar(new_car, sensorFusionData[i]);
      other_cars.insert(other_cars.begin()+idx, new_car);
    }
  }

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SensorFusion::updateCar(CarState& car, std::vector<double> &raw_data)
{
  // [ id, x, y, vx, vy, s, d]
  double vx = raw_data[3];
  double vy = raw_data[4];
  car.id = raw_data[0];
  car.f_pos.s = raw_data[5];
  car.f_pos.d = raw_data[6];

  if(car.f_pos.d < 0)
  {
    car.isValid = false;
    car.lane = 99;
  }
  else
  {
    car.lane = (size_t)std::floor(car.f_pos.d/4.0);
    car.isValid = true;
  }

  car.setSpeed(std::sqrt(std::pow(vx,2.0)+std::pow(vy,2.0)));

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
CarState SensorFusion::getNextCarInLane(size_t lane, double s) const
{
  std::function<bool(double, double)> op = [](double a, double b){ return a > b; };
  return getCarInLane(lane, s, op);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
CarState SensorFusion::getPreviousCarInLane(size_t lane, double s) const
{
  std::function<bool(double, double)> op = [](double a, double b){ return a < b; };
  return getCarInLane(lane, s, op);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
CarState SensorFusion::getCarInLane(size_t lane, double s, std::function<bool(double, double)> &op) const
{
  CarState result;
  result.isValid = false;
  result.lane = lane;

  for(auto car : other_cars)
  {
    //Car is not valid nor on same lane
    if(!car.isValid || car.lane != lane)
      continue;

    //Set result car
    if(!result.isValid && op(car.f_pos.s,s))
    {
      result = car;
      continue;
    }

    //Update result car
    if(op(car.f_pos.s,s) && op(result.f_pos.s,car.f_pos.s))
    {
      result = car;
      continue;
    }
  }

  return result;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
