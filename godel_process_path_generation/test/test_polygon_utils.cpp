/*
* Software License Agreement (Apache License)
*
* Copyright (c) 2014, Southwest Research Institute
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/
/*
 * test_polygon_utils.cpp
 *
 *  Created on: Jun 9, 2014
 *      Author: Dan Solomon
 */

#include <gtest/gtest.h>
#include "godel_process_path_generation/polygon_utils.h"

TEST(PolygonSegment, simple)
{
  /*
   *     *   * p2 p4
   *     |  /
   *  p0 *---* p1
   *     |/
   *  p3 *
   */
  godel_process_path::PolygonPt p0(0., 0.), p1(1., 0.), p2(0., 1.), p3(0., -1), p4(1., 1.);

  godel_process_path::polygon_utils::PolygonSegment s01(p0, p1), s02(p0, p2), s23(p2, p3),
      s34(p3, p4);

  EXPECT_TRUE(s01.intersects(s23));
  EXPECT_TRUE(s01.intersects(s02));
  EXPECT_TRUE(s01.intersects(s34));
}

TEST(PolygonSegment, complex)
{
  /*
   *         *        p1
   *      // |
   *   *--*--*  p3 p0 p2
   *  //
   *  *         p4
   */
  godel_process_path::PolygonPt p0(0.035726, 0.017804), p1(0.038726, 0.021804),
      p2(0.038726, 0.017804), p3(0.032726, 0.017804), p4(0.038726, 0.014804);

  godel_process_path::polygon_utils::PolygonSegment s01(p0, p1), s12(p1, p2), s23(p2, p3),
      s34(p3, p4), s40(p4, p0), s41(p4, p1);

  EXPECT_FALSE(s01.intersects(s34));
  EXPECT_TRUE(s01.intersects(s23, .001));
  EXPECT_TRUE(s41.intersects(s23));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
