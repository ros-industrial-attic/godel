/*
  Copyright Feb, 2015 Southwest Research Institute

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

          http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*/

#ifndef MOTOMAN_SIA20D_ROBOT_MODEL
#define MOTOMAN_SIA20D_ROBOT_MODEL

#include <descartes_moveit/moveit_state_adapter.h>
#include <motoman_sia20d_ikfast_manipulator/motoman_sia20d_manipulator_ikfast_moveit_plugin.hpp>

namespace motoman_sia20d_descartes
{
class MotomanSia20dRobotModel
    : public descartes_moveit::MoveitStateAdapter,
      public motoman_sia20d_ikfast_manipulator::ikfast_kinematics_plugin::IKFastKinematicsPlugin
{
public:
  MotomanSia20dRobotModel();

  virtual bool initialize(const std::string& robot_description, const std::string& group_name,
                          const std::string& world_frame, const std::string& tcp_frame);

  virtual bool getAllIK(const Eigen::Affine3d& pose,
                        std::vector<std::vector<double> >& joint_poses) const;

protected:
  descartes_core::Frame world_to_base_; // world to arm base
  descartes_core::Frame tool_to_tip_;   // from urdf tool to arm tool
};
}

#endif // MOTOMAN_SIA20D_ROBOT_MODEL
