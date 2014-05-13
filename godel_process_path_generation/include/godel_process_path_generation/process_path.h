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
 * process_path.h
 *
 *  Created on: May 13, 2014
 *      Author: ros
 */

#ifndef PROCESS_PATH_H_
#define PROCESS_PATH_H_

#include "godel_process_path_generation/process_pt.h"
#include <visualization_msgs/Marker.h>

namespace descartes
{

class ProcessPath
{
public:
  ProcessPath() {};
  virtual ~ProcessPath() {};

  /**@ Add a ProcessPt to pts
   * @param pt Reference to ProcessPt to add to points
   */
  void addPoint(const ProcessPt &pt) {pts_.push_back(pt);}

  /**@brief Convert ProcessPath to line_list marker
   * Does not populate header, ns, id, lifetime, frame_locked
   * @return Marker populated with red lines
   */
  visualization_msgs::Marker asMarker();

  void clear() {pts_.clear();}

protected:
  std::vector<ProcessPt> pts_;
};

} /* namespace descartes */
#endif /* PROCESS_PATH_H_ */
