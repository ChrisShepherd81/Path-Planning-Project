/*
 * Trajectory.cpp
 *
 *  Created on: 30.07.2017
 */

#include "Trajectory.h"
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::tuple<std::vector<double>, std::vector<double>> Trajectory::update(
    std::vector<double> &prev_path_x,
    std::vector<double> &prev_path_y,
    double car_s, double car_d, double car_yaw)
{
  int path_size = prev_path_x.size();
  std::vector<double> next_x_vals;
  std::vector<double> next_y_vals;

  //Use last path as next values for current path
  for(int i = 0; i < path_size; i++)
  {
     next_x_vals.push_back(prev_path_x[i]);
     next_y_vals.push_back(prev_path_y[i]);
  }

  size_t lane = 0;
  if(path_size > 0)
  {
    lane = std::floor(d_vals.front()/4.0);;
  }
  else
    lane = 2.0; //std::floor(car_d/4.0);


  if(_stateChangeInProgress)
  {
    if(lane == _targetLane)
    {
      //_stateChangeInProgress = false;
      _state = State::KeepLane;
    }
  }

  double pos_s, pos_d, acc_in_s;
  double speed_in_s = 0;
  if(path_size == 0)
  {
     pos_s = car_s;
     pos_d = car_d;
  }
  else
  {
     for(size_t i = path_size; i < lastWayPoints; ++i )
     {
       s_vals.pop_front();
       d_vals.pop_front();
     }

     pos_s = s_vals.back();
     pos_d = d_vals.back();
  }

  Car carBefore = prediction.getNextCarInLane(lane, pos_s);

  if(carBefore.isValid && ((carBefore.getLastS() - pos_s) <= SAFETY_DISTANCE))
  {
    //Slow down to next car speed
//    _target_speed = carBefore.speed;
//    if(_target_speed > TARGET_SPEED)
      _target_speed = TARGET_SPEED;
  }
  else
  {
    _target_speed = TARGET_SPEED;
  }

//  if(carBefore.isValid)
//    carBefore.print(pos_s);

  if(_state != State::KeepLane && !_stateChangeInProgress)
  {

    if(_state == State::ChangeLaneLeft)
    {
      _targetLane = lane-1;
    }
    else
    {
      _targetLane = lane+1;
    }

    //std::cout << "Check state change for target lane" << _targetLane <<"\n";
    //Check if possible
    Car carAhead = prediction.getNextCarInLane(_targetLane, pos_s);
    double distanceAhead = SAFETY_DISTANCE_AHEAD;
    if(carAhead.isValid)
    {
      //std::cout << "Car ahead ";
      distanceAhead = carAhead.getLastS() - pos_s;
      carAhead.print(pos_s);
    }
      
    Car carBehind = prediction.getPreviousCarInLane(_targetLane, car_s);
    double distanceBehind = SAFETY_DISTANCE_BEHIND;
    if(carBehind.isValid)
    {
       distanceBehind = pos_s - carBehind.getLastS();
       //std::cout << "Car behind ";
       carBehind.print(pos_s);
    }
    

    if(distanceBehind >= SAFETY_DISTANCE_BEHIND
        && distanceAhead >= SAFETY_DISTANCE_AHEAD
        && _targetLane <= 2 )
    {
      _stateChangeInProgress = true;
      _laneShiftQueued = false;

      //std::cout << "Prepare state change\n";

      //Remove previous path
      size_t start = 5;
      next_x_vals.erase(next_x_vals.begin()+start, next_x_vals.end());
      next_y_vals.erase(next_y_vals.begin()+start, next_y_vals.end());
      s_vals.erase(s_vals.begin()+start, s_vals.end());
      d_vals.erase(d_vals.begin()+start, d_vals.end());

      double pos_s_before = s_vals[s_vals.size()-2];
      pos_s = s_vals.back();
      pos_d = d_vals.back();

      //Calculate speed
      speed_in_s = (pos_s - pos_s_before)/INTERVAL;

    }
    else
    {
      //std::cout << "Canceled state change\n";
      _targetLane = lane;
      _stateChangeInProgress = false;
      _state = State::KeepLane;
    }
  }

  //straight path constant speed
  if(path_size < WAY_POINTS || ( _stateChangeInProgress && !_laneShiftQueued ) )
  {
    double pos_d_2 = 2.0 + (4*_targetLane);
    double deltaSpeed = _target_speed - speed_in_s;
    double timeInSec = std::abs(deltaSpeed / MAX_ACC_IN_S);
    double timeForDInS = std::abs(pos_d-pos_d_2)/MAX_ACC_IN_D;
    if(timeInSec < timeForDInS)
      timeInSec = timeForDInS;
    double timeInQueue = path_size * INTERVAL;
    timeInSec = (timeInSec+timeInQueue) < 1.0 ? 1.0-timeInQueue : timeInSec;

    double pos_s_2 = pos_s + ((speed_in_s+_target_speed)/2.0)*timeInSec;

    timeInSec = (WAY_POINTS-path_size)*INTERVAL;
    FrenetState start(pos_s, pos_d, _lastSpeed, 0, 0,0);
    FrenetState stop(pos_s_2, pos_d_2, _target_speed, 0, 0,0);
    _lastSpeed = _target_speed;


    //std::cout << "Target d is " << pos_d_2 << std::endl;

    auto path = generator.generate(start, stop, timeInSec);
    auto cartesianPath = std::get<0>(path);
    auto frenetPath = std::get<1>(path);

    for(size_t i=1; i < cartesianPath.size(); ++i) //omit first == actual value
    {
      next_x_vals.push_back(cartesianPath.at(i).X);
      next_y_vals.push_back(cartesianPath.at(i).Y);
      storeFrenetPoint(frenetPath.at(i).s, frenetPath.at(i).d);
      _pathWriter.AddNewPointToPath(cartesianPath.at(i).X, cartesianPath.at(i).Y,frenetPath.at(i).s, frenetPath.at(i).d);
    }

    _pathWriter.AddNewPointToPath(0,0,0,0);
    _laneShiftQueued = true;
  }

  lastWayPoints = next_x_vals.size();
  return std::make_tuple(next_x_vals, next_y_vals);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Trajectory::storeFrenetPoint(double s, double d)
{
  s_vals.push_back(s);
  d_vals.push_back(d);
}
