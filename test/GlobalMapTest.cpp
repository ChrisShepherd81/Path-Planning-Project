/*
 * GlobalMapTest.cpp
 *
 *  Created on: 04.08.2017
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "../src/GlobalMap.h"

TEST(OutOfRangeIndex, TestOutOfRangeIndexer)
{
  size_t max = 132;
  std::function<size_t (int)> outOfRangeIndexer = [&max](int i) { return i < 0 ? max+i+1 : (i > max ? i-max-1 : i) ; };

  EXPECT_EQ(outOfRangeIndexer(-2), 131);
  EXPECT_EQ(outOfRangeIndexer(-1), 132);
  EXPECT_EQ(outOfRangeIndexer(0), 0);
  EXPECT_EQ(outOfRangeIndexer(131), 131);
  EXPECT_EQ(outOfRangeIndexer(132), 132);
  EXPECT_EQ(outOfRangeIndexer(133), 0);
  EXPECT_EQ(outOfRangeIndexer(134), 1);
  EXPECT_EQ(outOfRangeIndexer(265), 132);

}

TEST(GloablMapTest, TestBidirectionalTransformation)
{
  GlobalMap sut;

  sut.init();

  auto cartesian = sut.TransformFrenetToCartesian(0, 0);
  auto frenet = sut.TransformCartesianToFrenet(cartesian[0], cartesian[1], 3.141);
  EXPECT_NEAR(0, frenet[0], 0.1);
  EXPECT_NEAR(0, frenet[1], 0.1);

  cartesian = sut.TransformFrenetToCartesian(100, 0);
  frenet = sut.TransformCartesianToFrenet(cartesian[0], cartesian[1], 3.141);
  EXPECT_NEAR(100, frenet[0], 0.1);
  EXPECT_NEAR(0, frenet[1], 0.1);

  cartesian = sut.TransformFrenetToCartesian(1000, 10);
  frenet = sut.TransformCartesianToFrenet(cartesian[0], cartesian[1], 3.141);
  EXPECT_NEAR(1000, frenet[0], 0.1);
  EXPECT_NEAR(10, frenet[1], 0.1);

  cartesian = sut.TransformFrenetToCartesian(5000, 2);
  frenet = sut.TransformCartesianToFrenet(cartesian[0], cartesian[1], 3.141);
  EXPECT_NEAR(5000, frenet[0], 0.1);
  EXPECT_NEAR(2, frenet[1], 0.1);
}


TEST(GloablMapTest, TestBasicFrenetToCartesianTransformation)
{
  GlobalMap sut;

  sut.init();

  //784.6001 1135.571 0
  auto result = sut.TransformFrenetToCartesian(0, 0);
  ASSERT_NEAR( result[0], 784.6001, 0.1);
  ASSERT_NEAR( result[1], 1135.571, 0.1);

  //875.0436 1134.808 90.4504146575928 -0.001847863 -0.9999983
  result = sut.TransformFrenetToCartesian(90.45, 0);
  ASSERT_NEAR( result[0], 875.0436, 0.1);
  ASSERT_NEAR( result[1], 1134.808, 0.1);

  //905.283 1134.799 120.689735412598 0.004131136 -0.9999915
  result = sut.TransformFrenetToCartesian(120.68, 0);
  ASSERT_NEAR( result[0], 905.28, 0.1);
  ASSERT_NEAR( result[1], 1134.799, 0.1);

  //934.9677 1135.055 150.375551223755 0.05904382 -0.9982554
  result = sut.TransformFrenetToCartesian(150.375, 0);
  ASSERT_NEAR( result[0], 934.96, 0.1);
  ASSERT_NEAR( result[1], 1135.05, 0.1);

  //964.7734 1138.318 180.359313964844 0.1677761 -0.9858252
  result = sut.TransformFrenetToCartesian(180.35, 0);
  ASSERT_NEAR( result[0], 964.77, 0.1);
  ASSERT_NEAR( result[1], 1138.31, 0.1);

  //440.6 1431.6 6465.03196716309
  result = sut.TransformFrenetToCartesian(6465.03, 0);
  ASSERT_NEAR( result[0], 440.6, 0.1);
  ASSERT_NEAR( result[1], 1431.6, 0.1);

  //670.6 1159 6828.09141921997
  result = sut.TransformFrenetToCartesian(6828.091, 0);
  ASSERT_NEAR( result[0], 670.6, 0.1);
  ASSERT_NEAR( result[1], 1159, 0.1);

  //711.2 1143.5 6871.54959487915 -0.2637061 -0.9646032
  result = sut.TransformFrenetToCartesian(6871.549, 0);
  ASSERT_NEAR( result[0], 711.2, 0.1);
  ASSERT_NEAR( result[1], 1143.5, 0.1);

  //753.2067 1136.417 6914.14925765991 -0.107399 -0.9942161
  result = sut.TransformFrenetToCartesian(6914.14, 0);
  ASSERT_NEAR( result[0], 753.20, 0.1);
  ASSERT_NEAR( result[1], 1136.4, 0.1);

}


