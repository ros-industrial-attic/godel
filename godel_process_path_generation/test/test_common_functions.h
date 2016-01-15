/* Note: Portions of this file fall under a different license! */
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
 * test_common_functions.h
 *
 *  Created on: May 8, 2014
 *      Author: Dan Solomon
 */

#ifndef TEST_COMMON_FUNCTIONS_H_
#define TEST_COMMON_FUNCTIONS_H_

#include <vector>

namespace /*alternate license*/
{

/*
* Note: The code in this block is under a different license than the rest of the file!
*
* Software License Agreement (BSD License)
*
* Point Cloud Library (PCL) - www.pointclouds.org
* Copyright (c) 2009-2012, Willow Garage, Inc.
* Copyright (c) 2012-, Open Perception, Inc.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
*  * Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above
*    copyright notice, this list of conditions and the following
*    disclaimer in the documentation and/or other materials provided
*    with the distribution.
*  * Neither the name of the copyright holder(s) nor the names of its
*    contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* $Id$
*
*/
/** \brief Check if the 'actual' is a circular permutation of 'expected' (only clockwise).
  * \example [0 1 2 3] [1 2 3 0] [2 3 0 1] [3 0 1 2] are all equal.
  */
template <class ContainerT>
bool isCircularPermutation(const ContainerT& expected, const ContainerT& actual,
                           const bool verbose = false)
{
  const unsigned int n = static_cast<unsigned int>(expected.size());
  EXPECT_EQ(n, actual.size());
  if (n != actual.size())
  {
    if (verbose)
      std::cerr << "expected.size () != actual.size (): " << n << " != " << actual.size() << "\n";
    return (false);
  }

  for (unsigned int i = 0; i < n; ++i)
  {
    bool all_equal = true;
    for (unsigned int j = 0; j < n; ++j)
    {
      //      if (verbose) std::cerr << actual [(i+j)%n] << " " << expected [j];
      if (actual[(i + j) % n] != expected[j])
      {
        all_equal = false;
      }
      if (verbose)
        std::cerr << " | ";
    }
    if (all_equal)
    {
      if (verbose)
        std::cerr << " SUCCESS\n";
      return (true);
    }
    if (verbose)
      std::cerr << "\n";
  }
  return (false);
}
} // end alternate license

#endif /* TEST_COMMON_FUNCTIONS_H_ */
