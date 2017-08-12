/*
 * TrajectoryGenerator.h
 *
 *  Created on: 05.08.2017
 */

#ifndef SRC_TRAJECTORYGENERATOR_H_
#define SRC_TRAJECTORYGENERATOR_H_

#include "Path.h"
#include "GlobalMap.h"
#include "Configuration.h"

#include "Eigen-3.3/Eigen/Dense"

#include <vector>

class TrajectoryGenerator {
 public:

  TrajectoryGenerator(GlobalMap& map) : _map(map) {}

  CartesianPath generate(CarState car, int target_lane, double target_speed,
                          std::vector<double> &previous_path_x, std::vector<double> &previous_path_y);
 private:
  GlobalMap& _map;
  Path<double> JMT(std::vector< double> start, std::vector <double> end, double T);

};

#endif /* SRC_TRAJECTORYGENERATOR_H_ */
