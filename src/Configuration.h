/*
 * Configuration.h
 *
 *  Created on: 11.08.2017
 */

#ifndef SRC_CONFIGURATION_H_
#define SRC_CONFIGURATION_H_


struct Configuration
{
  static constexpr double MPH_TO_MPS =  0.44704;
  static constexpr double MAX_SPEED = 49.5*MPH_TO_MPS;
  static constexpr double SAFETY_DISTANCE = 30.0;
  static constexpr double EMERGENCY_DISTANCE = 10.0;
  static constexpr double MIN_DIST_AHEAD = 25.0;
  static constexpr double MIN_DIST_BEHIND = 15.0;
};

struct Simulator
{
  static constexpr unsigned int LANES_COUNT = 3;
  static constexpr unsigned int START_LANE = 1;
  static constexpr double MAX_SPEED_CHANGE = 0.224*Configuration::MPH_TO_MPS;
  static constexpr double LANE_D_VALUE [] = {2.0, 6.0, 9.6 };
  static constexpr double INTERVAL = 0.02;
  static constexpr double MAX_S = 6945.554;
};




#endif /* SRC_CONFIGURATION_H_ */
