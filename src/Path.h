/*
 * Path.h
 *
 *  Created on: 05.08.2017
 */

#ifndef SRC_PATH_H_
#define SRC_PATH_H_

#include <cmath>
#include <vector>

struct CartesianPoint
{
  CartesianPoint(double x, double y) : X(x), Y(y) {}

  CartesianPoint(std::vector<double> v)
  {
    X = v[0];
    Y = v[1];
  }

  double distanceTo(CartesianPoint const& p) const
  {
    return std::sqrt((p.X-X)*(p.X-X)+(p.Y-Y)*(p.Y-Y));
  }

  double X;
  double Y;
};

struct FrenetPoint
{
  FrenetPoint() : s(0), d(0){}
  FrenetPoint(double s, double d) : s(s), d(d) {}
  double s;
  double d;
};

struct FrenetState
{
  FrenetState (double s, double d, double s_dot, double d_dot, double s_ddot, double d_ddot) :
    position(s, d), velocity(s_dot, d_dot), acceleration(s_ddot, d_ddot)
  { }

  FrenetPoint position;
  FrenetPoint velocity;
  FrenetPoint acceleration;

  std::vector<double> d_vec() { return std::vector<double>{position.d, velocity.d, acceleration.d}; }
  std::vector<double> s_vec() { return std::vector<double>{position.s, velocity.s, acceleration.s}; }
};

template <typename T>
using Path = std::vector<T>;
using CartesianPath = Path<CartesianPoint>;
using FrenetPath = Path<FrenetPoint>;


#endif /* SRC_PATH_H_ */
