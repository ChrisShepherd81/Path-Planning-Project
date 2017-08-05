/*
 * TrajectoryGenerator.h
 *
 *  Created on: 05.08.2017
 */

#ifndef SRC_TRAJECTORYGENERATOR_H_
#define SRC_TRAJECTORYGENERATOR_H_

#include "Path.h"
#include "CostCalculation.h"
#include "GlobalMap.h"

#include "Eigen-3.3/Eigen/Dense"

#include <random>
#include <vector>

class TrajectoryGenerator {
 public:

  TrajectoryGenerator(GlobalMap& map) : _map(map) {}

  std::tuple<CartesianPath, FrenetPath> generate(FrenetState start, FrenetState stop, double time);

 private:
  GlobalMap& _map;
  const double _timeDelta = 0.02;
  CostCalculation _costs;
  std::default_random_engine _rnd_gen;

  double calculateSpeedCost();

  Path<double> JMT(std::vector< double> start, std::vector <double> end, double T);
  FrenetPath calculate(std::vector<double> const& c_s, std::vector<double> const& c_d, double time, FrenetState const& start, FrenetState const& stop);
  CartesianPath transform(FrenetPath& path);
};

#endif /* SRC_TRAJECTORYGENERATOR_H_ */
