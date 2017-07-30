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
std::vector<double> GlobalMap::TransformFrenetToCartesian(double s, double d)
{
  int prev_wp = -1;

  while(s > _map_s[prev_wp+1] && (prev_wp < (int)(_map_s.size()-1) ))
  {
    prev_wp++;
  }

  tk::spline x_s;
  tk::spline y_s;

  size_t max = this->max_index;
  std::function<size_t (int)> ringIndexer = [&max](int i) { return i < 0 ? max-i : (i > max ? max-i : i) ; };

  size_t pointsBefore = ringIndexer(prev_wp-4);
  size_t pointsAfter = ringIndexer(prev_wp+5);

  std::vector<double> x_vals(&_map_x[pointsBefore], &_map_x[pointsAfter]);
  std::vector<double> y_vals(&_map_y[pointsBefore], &_map_y[pointsAfter]);
  std::vector<double> s_vals(&_map_s[pointsBefore], &_map_s[pointsAfter]);
  std::vector<double> dx_vals(&_map_dx[pointsBefore], &_map_dx[pointsAfter]);
  std::vector<double> dy_vals(&_map_dy[pointsBefore], &_map_dy[pointsAfter]);

  //Shift middle lane to correct d position
  size_t count = x_vals.size();
  for(size_t i = 0; i < count; ++i)
  {
    x_vals[i] += (dx_vals[i] * d);
    y_vals[i] += (dy_vals[i] * d);
  }

  y_s.set_points(s_vals, y_vals);
  x_s.set_points(s_vals, x_vals);


  //int wp2 = (prev_wp+1)%_map_x.size();

  //TODO print values and inspect graphical

  //TODO try to use splines
  //double heading = std::atan2((_map_y[wp2]-_map_y[prev_wp]),(_map_x[wp2]-_map_x[prev_wp]));

  // the x,y,s along the segment
  //double seg_s = (s-_map_s[prev_wp]);

//  double seg_x = _map_x[prev_wp]+seg_s*cos(heading);
//  double seg_y = _map_y[prev_wp]+seg_s*sin(heading);

  double x = x_s(s);
  double y = y_s(s);

  std::cout << "Value at " << s << ": " << "(" << x << "," << y << ")\n";

  return {x,y};
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> GlobalMap::TransformCartesianToFrenet(double x, double y, double theta)
{
  int next_wp = nextWaypoint(x,y, theta);
//  size_t max = this->max_index;
//  std::function<size_t (int)> ringIndexer = [&max](int i) { return i < 0 ? max-i : (i > max ? max-i : i) ; };
//
//  size_t pointsBefore = ringIndexer(next_wp-5);
//  size_t pointsAfter = ringIndexer(next_wp+4);
//
//  std::vector<double> x_vals(&_map_x[pointsBefore], &_map_x[pointsAfter]);
//  std::vector<double> y_vals(&_map_y[pointsBefore], &_map_y[pointsAfter]);
//  std::vector<double> s_vals(&_map_s[pointsBefore], &_map_s[pointsAfter]);
//  std::vector<double> dx_vals(&_map_dx[pointsBefore], &_map_dx[pointsAfter]);
//  std::vector<double> dy_vals(&_map_dy[pointsBefore], &_map_dy[pointsAfter]);
//
//  tk::spline x_s;
//  tk::spline path_s;
//
//  path_s.set_points(x_vals, y_vals);
//  x_s.set_points(s_vals, x_vals);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
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

  return {frenet_s,frenet_d};
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double GlobalMap::distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int GlobalMap::closestWaypoint(double x, double y)
{

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < _map_x.size(); i++)
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

  double angle = std::abs(theta-heading);

  if(angle > M_PI/4)
  {
    closest++;
  }

  return closest;

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
