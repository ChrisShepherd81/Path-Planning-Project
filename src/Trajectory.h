/*
 * Trajectory.h
 *
 *  Created on: 30.07.2017
 */

#ifndef SRC_TRAJECTORY_H_
#define SRC_TRAJECTORY_H_

#include "GlobalMap.h"
#include "Eigen-3.3/Eigen/Dense"

#include <tuple>
#include <vector>
#include <deque>

class Trajectory {
public:
  Trajectory(GlobalMap &globalMap) : globalMap(globalMap) {}

  std::tuple<std::vector<double>, std::vector<double>> update(
      std::vector<double> &prev_path_x,
      std::vector<double> &prev_path_y,
      double car_s, double car_d, double car_yaw,
      size_t lane, bool &laneShift );

 private:
  std::vector<double> JMT(std::vector< double> start, std::vector <double> end, double T);

  void storeFrenetPoint(double s, double d);

  std::deque<double> s_vals;
  std::deque<double> d_vals;

  GlobalMap& globalMap;

  const size_t WAY_POINTS = 50;
  size_t lastWayPoints = 0;

};

#endif /* SRC_TRAJECTORY_H_ */
