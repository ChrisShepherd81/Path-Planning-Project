/*
 * Path.h
 *
 *  Created on: 05.08.2017
 */

#ifndef SRC_PATH_H_
#define SRC_PATH_H_

#include <cmath>
#include <vector>
#include <iostream>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct CartesianPoint
{
  CartesianPoint() = default;
  CartesianPoint(double x, double y) : X(x), Y(y) {}
  CartesianPoint(std::vector<double> v)
  {
    X = v[0];
    Y = v[1];
  }

  double X;
  double Y;
};
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct FrenetPoint
{
  FrenetPoint() : s(0), d(0){}
  FrenetPoint(double s, double d) : s(s), d(d) {}
  double s;
  double d;
};
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class CarState
{
 public:
  CarState()
  {
    _speed_buffer = std::vector<double>(_bufferSize);
  }

  CartesianPoint c_pos;
  FrenetPoint f_pos;
  double Yaw;

  void setSpeed(double speed)
  {
    curr_speed = speed;
    _speed_buffer[_index%(_bufferSize)] = speed;
    _index++;
  }

  double avgSpeed()
  {
    double sum = 0;
    for(auto a : _speed_buffer)
      sum += a;

    return sum/_speed_buffer.size();
  }

  double curr_speed;
  double curr_s;
  size_t lane;
  bool isValid;
  int id;
 private:
  static constexpr size_t _bufferSize = 10;
  size_t _index = 0;
  std::vector<double> _speed_buffer;
};
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename T>
using Path = std::vector<T>;
class CartesianPath
{
 public:
  CartesianPath() = default;
  CartesianPath(Path<double> X_vals, Path<double> Y_vals)
  {
    if(X_vals.size() == Y_vals.size())
    {
      for(size_t i=0; i < X_vals.size(); ++i)
        push_back(CartesianPoint{X_vals[i], Y_vals[i]});
    }
  }

  void push_back(CartesianPoint p)
  {
    _path.push_back(p);
    _Xpath.push_back(p.X);
    _Ypath.push_back(p.Y);
    _size++;
  }

  Path<double>& getXValues()
  {
    return _Xpath;
  }

  Path<double>& getYValues()
  {
    return _Ypath;
  }

  CartesianPoint operator[](size_t index)
  {
    return _path[index];
  }

  size_t size()
  {
    return _size;
  }

 private:
  Path<CartesianPoint> _path;
  Path<double> _Xpath;
  Path<double> _Ypath;
  size_t _size = 0;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// *** Helper functions ***
double distance(double x1, double y1, double x2, double y2);
double distance(CartesianPoint p0, CartesianPoint p1);
constexpr double pi();
double deg2rad(double x);

#endif /* SRC_PATH_H_ */
