/*
 * Path.cpp
 *
 *  Created on: 11.08.2017
 */
#include "Path.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double distance(CartesianPoint p0, CartesianPoint p1)
{
  return distance(p0.X, p0.Y, p1.X, p1.Y);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double angle(double x1, double y1, double x2, double y2)
{
  return std::atan2(y1 - y2, x1 - x2);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double angle(CartesianPoint p0, CartesianPoint p1)
{
  return angle(p0.X, p0.Y, p1.X, p1.Y);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
CartesianPoint shiftAndRotate(CartesianPoint pointToShift, CartesianPoint refPoint, double angle)
{
  std::cout << "angle" << angle << std::endl;
  CartesianPoint t_p = CartesianPoint{pointToShift.X-refPoint.X, pointToShift.Y-refPoint.Y};
  double x_prime = std::cos(angle) * t_p.X + std::sin(angle) * t_p.Y;
  double y_prime = -std::sin(angle) * t_p.X + std::cos(angle) * t_p.Y;

  return CartesianPoint{x_prime, y_prime};
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
CartesianPoint rotateAndShift(CartesianPoint pointToShift, CartesianPoint refPoint, double angle)
{
  double x_prime = std::cos(angle) * pointToShift.X - std::sin(angle) * pointToShift.Y;
  double y_prime = std::sin(angle) * pointToShift.X + std::cos(angle) * pointToShift.Y;
  CartesianPoint t_p = CartesianPoint{x_prime+refPoint.X, y_prime+refPoint.Y};

  return t_p;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




