/*
 * test_ProcessPathGenerator.cpp
 *
 *  Created on: May 9, 2014
 *      Author: Dan Solomon
 */

#include <gtest/gtest.h>
#include "godel_process_path_generation/process_path_generator.h"

using godel_process_path::ProcessPathGenerator;
using godel_process_path::PolygonPt;

TEST(ProcessPathGeneratorTest, DISABLED_init)
{
  ProcessPathGenerator ppg;
  ppg.setTraverseHeight(.05);
  EXPECT_FALSE(ppg.createProcessPath()); // variables not properly set
  ppg.setMargin(-.01);
  ppg.setOverlap(.01);
  ppg.setToolRadius(.02);
  EXPECT_FALSE(ppg.createProcessPath()); // variables not properly set (margin)
  ppg.setMargin(0.);
  ppg.setOverlap(.04);
  ppg.setToolRadius(.02);
  EXPECT_FALSE(ppg.createProcessPath()); // variables not properly set (overlap)

  ppg.setMargin(.005);
  ppg.setOverlap(.01);
  ppg.setToolRadius(.025);
  EXPECT_FALSE(ppg.createProcessPath()); // Configure not done

  // Create single PolygonBoundary
  //  godel_process_path::PolygonBoundaryCollection boundaries;
  godel_process_path::PolygonBoundary boundary;
  boundary.push_back(PolygonPt(0., 0.));
  boundary.push_back(PolygonPt(.1, 0.));
  boundary.push_back(PolygonPt(0., .1));
  ppg.verbose_ = true;
  godel_process_path::PolygonBoundaryCollection boundaries(1, boundary);
  std::vector<double> offsets(1, .03); // Offset = tool radius + margin
  EXPECT_TRUE(ppg.setPathPolygons(&boundaries, &offsets));
  EXPECT_FALSE(ppg.createProcessPath());

  boundary.clear();
  boundary.push_back(PolygonPt(0., 0.));
  boundary.push_back(PolygonPt(.5, 0.));
  boundary.push_back(PolygonPt(0., .5));
  EXPECT_TRUE(ppg.setPathPolygons(&boundaries, &offsets));
  EXPECT_TRUE(ppg.createProcessPath());
}

TEST(ProcessPathGeneratorTest, complete)
{
  ProcessPathGenerator ppg;
  ppg.setTraverseHeight(.05);
  ppg.setMargin(.005);
  ppg.setOverlap(.01);
  ppg.setToolRadius(.025);
  ppg.setVelocity(godel_process_path::ProcessVelocity());

  godel_process_path::PolygonBoundary boundary;
  boundary.push_back(PolygonPt(0., 0.));
  boundary.push_back(PolygonPt(.5, 0.));
  boundary.push_back(PolygonPt(.5, .5));
  boundary.push_back(PolygonPt(.25, .125));
  boundary.push_back(PolygonPt(.0, .5));
  ppg.verbose_ = true;
  godel_process_path::PolygonBoundaryCollection boundaries(1, boundary);
  std::vector<double> offsets(1, .03); // Offset = tool radius + margin
  EXPECT_TRUE(ppg.setPathPolygons(&boundaries, &offsets));
  EXPECT_TRUE(ppg.createProcessPath());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
