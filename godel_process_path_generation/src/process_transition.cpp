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
 * process_transition.cpp
 *
 *  Created on: May 14, 2014
 *      Author: Dan Solomon
 */

#include "godel_process_path_generation/process_transition.h"

namespace descartes
{

bool JointVelocityConstraint::isValid()
{
  if (min.size() != max.size() || min.size() != desired.size())
  {
    return false;
  }
  for (size_t ii = 0; ii < min.size(); ++ii)
  {
    if (min.at(ii) > max.at(ii) || min.at(ii) > desired.at(ii) || desired.at(ii) > max.at(ii))
    {
      return false;
    }
  }
  return true;
}

bool LinearVelocityConstraint::isValid()
{
  if (min > max || min > desired || desired > max)
  {
    return false;
  }
  return true;
}

bool RotationalVelocityConstraint::isValid()
{
  if (min > max || min > desired || desired > max)
  {
    return false;
  }
  return true;
}

} /* namespace descartes */
