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
 * process_pt.h
 *
 *  Created on: May 12, 2014
 *      Author: Dan Solomon
 */

#ifndef PROCESS_PT_H_
#define PROCESS_PT_H_

//#include <moveit/constraint_samplers/default_constraint_samplers.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <Eigen/Geometry>

namespace descartes
{

/**Modeled on kinematic_constraint/OrientationConstraint (c) 2011 Willow Garage
 */
class OrientationConstraint
{
public:
  OrientationConstraint(){};
  virtual ~OrientationConstraint(){};

  void setTolerance(double x, double y, double z)
  {
    setToleranceX(x);
    setToleranceY(y);
    setToleranceZ(z);
  }

  void setToleranceX(double tol) { x_axis_tolerance_ = std::abs(tol); }

  void setToleranceY(double tol) { y_axis_tolerance_ = std::abs(tol); }

  void setToleranceZ(double tol) { z_axis_tolerance_ = std::abs(tol); }

protected:
  Eigen::AngleAxisd nominal_orientation_;
  double x_axis_tolerance_, y_axis_tolerance_, z_axis_tolerance_; /**<Applied as nominal +/- tol */
};
typedef boost::shared_ptr<OrientationConstraint> OrientationConstraintPtr;

/**Modeled on kinematic_constraint/PositionConstraint (c) 2011 Willow Garage
 */
class PositionConstraint
{
public:
  PositionConstraint(){};
  virtual ~PositionConstraint(){};

  void addBody(const bodies::BodyPtr& body) { constraint_region_.push_back(body); }

  const std::vector<bodies::BodyPtr>& getConstraintRegion() { return constraint_region_; }

protected:
  std::vector<bodies::BodyPtr> constraint_region_;
};
typedef boost::shared_ptr<PositionConstraint> PositionConstraintPtr;

class ProcessPt
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
  ProcessPt() : nominal_pose_(Eigen::Affine3d::Identity()){};
  virtual ~ProcessPt(){};

  const Eigen::Affine3d& getFrame() const { return frame_transform_; }

  Eigen::Affine3d& pose() { return nominal_pose_; }
  const Eigen::Affine3d& pose() const { return nominal_pose_; }

  void setFrame(const std::pair<std::string, Eigen::Affine3d>& frame)
  {
    pt_frame_ = frame.first;
    frame_transform_ = frame.second;
  }

  void setPosePosition(double x, double y, double z) { nominal_pose_.translation() << x, y, z; }

private:
  Eigen::Affine3d nominal_pose_;    /**<Default pose of process point */
  std::string pt_frame_;            /**<Frame pt is expressed in */
  Eigen::Affine3d frame_transform_; /**<Transform from planning frame to current pt_frame */
  PositionConstraintPtr position_constraint_;
  OrientationConstraintPtr orientation_constraint_;
  //  kinematic_constraints::KinematicConstraintSetPtr constraints_;
};

} /* namespace descartes*/
#endif /* PROCESS_PT_H_ */
