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
 *      Author: Dan Solomon
 */

#ifndef PROCESS_PATH_H_
#define PROCESS_PATH_H_

#include "godel_process_path_generation/process_pt.h"
#include "godel_process_path_generation/process_transition.h"
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>

namespace descartes
{

class ProcessPath
{
public:
  ProcessPath(){};
  virtual ~ProcessPath(){};

  /**@brief Add a ProcessPt to pts
   * @param pt Reference to ProcessPt to add to points
   */
  void addPoint(const ProcessPt& pt) { pts_.push_back(pt); }

  /**@brief Add a ProcessTranstition to transition_
   * @param trans Reference to ProcessTransition to add to points
   */
  void addTransition(const ProcessTransition& trans) { transitions_.push_back(trans); }

  /**@brief Get copy of data */
  std::pair<std::vector<ProcessPt>, std::vector<ProcessTransition> > data() const
  {
    return std::make_pair(pts_, transitions_);
  }

  /**@brief Convert ProcessPath to line_list marker
   * Does not populate header, ns, id, lifetime, frame_locked
   * @return Marker populated with red lines
   */
  visualization_msgs::Marker asMarker() const;

  /**
   * @brief Convert ProcessPath to geometry_msgs::PoseArray
   * @return PoseArray
   */
  geometry_msgs::PoseArray asPoseArray() const;

  void clear()
  {
    pts_.clear();
    transitions_.clear();
  }

protected:
  std::vector<ProcessPt> pts_;
  std::vector<ProcessTransition> transitions_;
};

} /* namespace descartes */
#endif /* PROCESS_PATH_H_ */
