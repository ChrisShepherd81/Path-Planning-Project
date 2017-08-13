/*
 * GlobalMap.cpp
 *
 *  Created on: 24.07.2017
 */

#include "GlobalMap.h"
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GlobalMap::init()
{
  // Waypoint map to read from
  std::string map_file_ = "../data/highway_map.csv";

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  std::string line;
  size_t lineCount = 0;

    while (getline(in_map_, line))
    {
      std::istringstream iss(line);
      double x;
      double y;
      float s;
      float d_x;
      float d_y;
      iss >> x;
      iss >> y;
      iss >> s;
      iss >> d_x;
      iss >> d_y;
      _map_x.push_back(x);
      _map_y.push_back(y);
      _map_s.push_back(s);
      _map_dx.push_back(d_x);
      _map_dy.push_back(d_y);
      lineCount++;
    }

    this->max_index = lineCount-1;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Transform from Frenet s,d coordinates to Cartesian x,y
CartesianPoint GlobalMap::TransformFrenetToCartesian(double s, double d)
{
  int prev_wp = -1;

  while((prev_wp < (int)(_map_s.size()-1) ) && s > _map_s[prev_wp+1])
  {
    ++prev_wp;
  }

  tk::spline x_s;
  tk::spline y_s;
  size_t max = this->max_index;

  bool outOfRange = true;
  std::function<size_t (int)> outOfRangeIndexer = [&max, &outOfRange](int i) { return i < 0 ? max+i+1 : (i > max ? i-max-1 : i) ; };

  int pointStart = outOfRangeIndexer(prev_wp-3);

  std::vector<double> x_vals;
  std::vector<double> y_vals;
  std::vector<double> s_vals;
  std::vector<double> dx_vals;
  std::vector<double> dy_vals;

  //Get 4 waypoints before and 4 after given s
  int waypoints = 8;
  for(int i=0; i < waypoints; ++i)
  {
    size_t index = outOfRangeIndexer(pointStart+i);

    x_vals.emplace_back(_map_x[index]);
    y_vals.emplace_back(_map_y[index]);

    if(((int)index - waypoints -1 < 0) && (s > Simulator::MAX_S/2)) //Out of range
      s_vals.emplace_back(_map_s[index] + Simulator::MAX_S);
    else if((int)index + waypoints -1 > max_index && (s < Simulator::MAX_S/2)) //Out of range
      s_vals.emplace_back(_map_s[index] - Simulator::MAX_S);
    else
      s_vals.emplace_back(_map_s[index]);

    dx_vals.emplace_back(_map_dx[index]);
    dy_vals.emplace_back(_map_dy[index]);
  }

  //Shift middle lane to correct d position
  size_t count = x_vals.size();
  for(size_t i = 0; i < count; ++i)
  {
    x_vals[i] += (dx_vals[i] * d);
    y_vals[i] += (dy_vals[i] * d);
  }

  y_s.set_points(s_vals, y_vals);
  x_s.set_points(s_vals, x_vals);

  return CartesianPoint{x_s(s),y_s(s)};
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
FrenetPoint GlobalMap::TransformCartesianToFrenet(double x, double y, double theta)
{
  int next_wp = nextWaypoint(x,y, theta);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0) //Overflow
  {
    prev_wp  = _map_x.size()-1;
  }

  double n_x = _map_x[next_wp]-_map_x[prev_wp];
  double n_y = _map_y[next_wp]-_map_y[prev_wp];
  double x_x = x - _map_x[prev_wp];
  double x_y = y - _map_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-_map_x[prev_wp];
  double center_y = 2000-_map_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(_map_x[i],_map_y[i],_map_x[i+1],_map_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return FrenetPoint{frenet_s,frenet_d};
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int GlobalMap::closestWaypoint(double x, double y)
{

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(size_t i = 0; i < _map_x.size(); i++)
  {
    double map_x = _map_x[i];
    double map_y = _map_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }

  }

  return closestWaypoint;

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int GlobalMap::nextWaypoint(double x, double y, double theta)
{

  int closest = closestWaypoint(x,y);

  double map_x = _map_x[closest];
  double map_y = _map_y[closest];

  double heading = atan2( (map_y-y),(map_x-x) );

  //Check direction
  double angle = std::abs(theta-heading);

  if(angle > M_PI/4)
  {
    closest++;
  }

  return closest;

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
