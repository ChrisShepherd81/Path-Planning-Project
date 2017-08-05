/*
 * CostCalculationTest.cpp
 *
 *  Created on: 05.08.2017
 */


#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "../src/TrajectoryGenerator.h"
#include "../src/GlobalMap.h"
#include "../src/CostCalculation.h"

TEST(CostCaculationTest, TestSpeedCostCalculation)
{
  CostCalculation cost;

  CartesianPath path{ {0,0}, {0,0} };

  EXPECT_NEAR(1, cost.speedCost(path), 1e-6);

  path = CartesianPath{ {0,0}, {0,0.5} };
  EXPECT_EQ(1, cost.speedCost(path));

  path = CartesianPath{ {0,0}, {0.5,0} };
  EXPECT_EQ(1, cost.speedCost(path));

  path = CartesianPath{ {0,0}, {0.4,0} };
  double cost1 = cost.speedCost(path);
  EXPECT_LT(cost1, 1);

  path = CartesianPath{ {0,0}, {0.3,0} };
  double cost2 = cost.speedCost(path);
  EXPECT_LT(cost2, 1);
  EXPECT_GT(cost2, cost1);

  path = CartesianPath{ {0,0}, {0.44,0} };
  double cost3 = cost.speedCost(path);
  EXPECT_LT(cost3, 1);
  EXPECT_GT(cost1, cost3);

}

