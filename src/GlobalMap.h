/*
 * GlobalMap.h
 *
 *  Created on: 24.07.2017
 */

#ifndef SRC_GLOBALMAP_H_
#define SRC_GLOBALMAP_H_

#include <vector>
#include <fstream>
#include <string>
#include <iostream>
#include <sstream>
#include <cmath>
#include "spline.h"

class GlobalMap
{
 public:
  void init();

  // Transform from Frenet s,d coordinates to Cartesian x,y
  std::vector<double> TransformFrenetToCartesian(double s, double d);
  std::vector<double> TransformCartesianToFrenet(double x, double y, double theta);

 private:

  double distance(double x1, double y1, double x2, double y2);
  int closestWaypoint(double x, double y);
  int nextWaypoint(double x, double y, double theta);

  const double max_s = 6945.554;
  size_t max_index;

  std::vector<double> _map_x;
  std::vector<double> _map_y;
  std::vector<double> _map_s;
  std::vector<double> _map_dx;
  std::vector<double> _map_dy;

};

#endif /* SRC_GLOBALMAP_H_ */
