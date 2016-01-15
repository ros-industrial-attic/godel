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
 * process_transition.h
 *
 *  Created on: May 14, 2014
 *      Author: Dan Solomon
 */

#ifndef PROCESS_TRANSITION_H_
#define PROCESS_TRANSITION_H_

#include <ros/ros.h>
#include <algorithm>

namespace descartes
{

class ProcessTransitionConstraint
{
public:
  ProcessTransitionConstraint(){};
  virtual ~ProcessTransitionConstraint(){};

  virtual bool isValid() = 0;
};

struct JointVelocityConstraint : public ProcessTransitionConstraint
{
  JointVelocityConstraint(){};
  JointVelocityConstraint(const JointVelocityConstraint& other)
      : min(other.min), desired(other.desired), max(other.max){};

  // TODO write
  // TODO comment
  virtual bool isValid();

  std::vector<double> min, desired, max;
};
typedef boost::shared_ptr<JointVelocityConstraint> JointVelocityConstraintPtr;

struct LinearVelocityConstraint : public ProcessTransitionConstraint
{
  LinearVelocityConstraint() : min(0.), desired(0.), max(0.){};
  LinearVelocityConstraint(double _min, double _desired, double _max)
      : min(_min), desired(_desired), max(_max){};
  LinearVelocityConstraint(double _desired)
      : min(0.), desired(_desired), max(std::numeric_limits<double>::max()){};
  LinearVelocityConstraint(const LinearVelocityConstraint& other)
      : min(other.min), desired(other.desired), max(other.max){};

  // TODO write
  // TODO comment
  virtual bool isValid();

  double min, desired, max;
};
typedef boost::shared_ptr<LinearVelocityConstraint> LinearVelocityConstraintPtr;

struct RotationalVelocityConstraint : public ProcessTransitionConstraint
{
  RotationalVelocityConstraint() : min(0.), desired(0.), max(0.){};
  RotationalVelocityConstraint(const RotationalVelocityConstraint& other)
      : min(other.min), desired(other.desired), max(other.max){};
  // TODO write
  // TODO comment
  virtual bool isValid();

  double min, desired, max;
};
typedef boost::shared_ptr<RotationalVelocityConstraint> RotationalVelocityConstraintPtr;

class ProcessTransition
{
public:
  ProcessTransition(){};
  virtual ~ProcessTransition(){};

  // TODO write
  // TODO comment
  bool constraintsValid();

  void setJointVelocityConstraint(const JointVelocityConstraint& jvc)
  {
    joint_velocity_.reset(new JointVelocityConstraint(jvc));
  }
  void setJointVelocityConstraint(const JointVelocityConstraintPtr& jvc) { joint_velocity_ = jvc; }

  void setLinearVelocityConstraint(const LinearVelocityConstraint& lvc)
  {
    linear_velocity_.reset(new LinearVelocityConstraint(lvc));
  }
  void setLinearVelocityConstraint(const LinearVelocityConstraintPtr& lvc)
  {
    linear_velocity_ = lvc;
  }

  void setRotationalVelocityConstraint(const RotationalVelocityConstraint& rvc)
  {
    rotational_velocity_.reset(new RotationalVelocityConstraint(rvc));
  }
  void setRotationalVelocityConstraint(const RotationalVelocityConstraintPtr& rvc)
  {
    rotational_velocity_ = rvc;
  }

  const LinearVelocityConstraintPtr& getLinearVelocity() const { return linear_velocity_; }

protected:
  JointVelocityConstraintPtr joint_velocity_;           /**<n*rad/s */
  LinearVelocityConstraintPtr linear_velocity_;         /**<m/s */
  RotationalVelocityConstraintPtr rotational_velocity_; /**<rad/s */
};

} /* namespace descartes */
#endif /* PROCESS_TRANSITION_H_ */
