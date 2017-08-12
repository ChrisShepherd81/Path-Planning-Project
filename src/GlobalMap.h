/*
 * GlobalMap.h
 *
 *  Created on: 24.07.2017
 */

#ifndef SRC_GLOBALMAP_H_
#define SRC_GLOBALMAP_H_

#include "CostCalculation.h"
#include "spline.h"

#include <vector>
#include <fstream>
#include <string>
#include <iostream>
#include <sstream>
#include <cmath>

class GlobalMap
{
 public:
  void init();

  // Transform from Frenet s,d coordinates to Cartesian x,y
  CartesianPoint TransformFrenetToCartesian(double s, double d);
  FrenetPoint TransformCartesianToFrenet(double x, double y, double theta);

 private:

  int closestWaypoint(double x, double y);
  int nextWaypoint(double x, double y, double theta);

  size_t max_index;

  std::vector<double> _map_x;
  std::vector<double> _map_y;
  std::vector<double> _map_s;
  std::vector<double> _map_dx;
  std::vector<double> _map_dy;

};

#endif /* SRC_GLOBALMAP_H_ */
