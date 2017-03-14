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

#include <abb_irb2400_descartes/abb_irb2400_robot_model.h>
#include <eigen_conversions/eigen_kdl.h>
#include <pluginlib/class_list_macros.h>

static const std::string MOTOMAN_SIA20D_BASE_LINK = "base_link";
static const std::string MOTOMAN_SIA20D_TOOL_LINK = "tool0";
static const double JOINT_LIMIT_TOLERANCE = .0000001f;

using namespace descartes_moveit;
using namespace irb2400_ikfast_manipulator_plugin;

namespace abb_irb2400_descartes
{
AbbIrb2400RobotModel::AbbIrb2400RobotModel()
    : world_to_base_(Eigen::Affine3d::Identity()), tool_to_tip_(Eigen::Affine3d::Identity())
{
}

bool AbbIrb2400RobotModel::initialize(const std::string& robot_description,
                                      const std::string& group_name, const std::string& world_frame,
                                      const std::string& tcp_frame)
{
  MoveitStateAdapter::initialize(robot_description, group_name, world_frame, tcp_frame);
  irb2400_ikfast_manipulator_plugin::IKFastKinematicsPlugin::initialize(
      robot_description, group_name, MOTOMAN_SIA20D_BASE_LINK, MOTOMAN_SIA20D_TOOL_LINK, 0.001);

  // initialize world transformations
  if (tcp_frame != getTipFrame())
  {
    tool_to_tip_ = descartes_core::Frame(robot_state_->getFrameTransform(tcp_frame).inverse() *
                                         robot_state_->getFrameTransform(getTipFrame()));
  }

  if (world_frame != getBaseFrame())
  {
    world_to_base_ = descartes_core::Frame(world_to_root_.frame *
                                           robot_state_->getFrameTransform(getBaseFrame()));
  }

  return true;
}

bool AbbIrb2400RobotModel::getAllIK(const Eigen::Affine3d& pose,
                                    std::vector<std::vector<double> >& joint_poses) const
{
  std::vector<double> vfree(free_params_.size(), 0.0);
  KDL::Frame frame;
  Eigen::Affine3d tool_pose = world_to_base_.frame_inv * pose * tool_to_tip_.frame;
  tf::transformEigenToKDL(tool_pose, frame);

  ikfast::IkSolutionList<IkReal> solutions;

  int numsol = solve(frame, vfree, solutions);

  joint_poses.clear();

  if (numsol)
  {
    for (int s = 0; s < numsol; ++s)
    {
      std::vector<double> sol;
      getSolution(solutions, s, sol);

      if (isValid(sol))
        joint_poses.push_back(sol);

      // So, IKFast returns the unique configurations of the robot (e.g. elbow up, wrist down)
      // and the solutions have joint values between -pi and +pi. If the robot can rotate more
      // than this, then we need to check to see if we have extra solutions that have the same
      // configuration but a different joint position. In our case, joint 6 has this kind of
      // extra motion, so here we check for valid solutions 360 degrees from each solution.
      sol[5] += 2 * M_PI;
      if (isValid(sol))
        joint_poses.push_back(sol);

      sol[5] -= 2 * 2 * M_PI;
      if (isValid(sol))
        joint_poses.push_back(sol);

    }
  }

  return !joint_poses.empty();
}

}

PLUGINLIB_EXPORT_CLASS(abb_irb2400_descartes::AbbIrb2400RobotModel, descartes_core::RobotModel)
