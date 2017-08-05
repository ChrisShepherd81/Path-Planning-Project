/*
 * TrajectoryGeneratorTest.cpp
 *
 *  Created on: 05.08.2017
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "../src/TrajectoryGenerator.h"
#include "../src/GlobalMap.h"

TEST(TrajectoryGenerator, TestGeneralPathGeneration)
{
  GlobalMap map;
  map.init();

  TrajectoryGenerator sut(map);

  FrenetState start(0,0,0,0,0,0);
  FrenetState stop(10,0,10,0,0,0);

  auto result = sut.generate(start, stop, 1.0);

  auto path = std::get<0>(result);

  EXPECT_NEAR(path.back().X - path.front().X, 10, 0.5);
  EXPECT_NEAR(path.back().Y - path.front().Y, 0, 0.5);

  start = FrenetState(10,0,10,0,0,0);
  stop = FrenetState(20,0,10,0,0,0);

  result = sut.generate(start, stop, 1.0);


  EXPECT_NEAR(path.back().X - path.front().X, 10, 0.5);
  EXPECT_NEAR(path.back().Y - path.front().Y, 0, 0.5);

}

