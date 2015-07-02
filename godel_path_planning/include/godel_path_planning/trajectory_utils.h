#ifndef TRAJECTORY_UTILS_H_
#define TRAJECTORY_UTILS_H_

#include <descartes_trajectory/cart_trajectory_pt.h>

namespace godel_path_planning
{
  typedef std::vector<descartes_core::TrajectoryPtPtr> TrajectoryVec;

  inline
  descartes_core::TrajectoryPtPtr makeCartesianPt(const Eigen::Affine3d& pose, 
                                                  const Eigen::Vector3d& pos_tol,
                                                  const Eigen::Vector3d& orient_tol,
                                                  double ds, double dtheta,
                                                  const descartes_core::TimingConstraint& timing)
  {

    using namespace descartes_core;
    using namespace descartes_trajectory;

    Eigen::Vector3d rpy =  pose.rotation().eulerAngles(0,1,2);
    double rx = rpy(0);
    double ry = rpy(1);
    double rz = rpy(2);
    double x = pose.translation()(0);
    double y = pose.translation()(1);
    double z = pose.translation()(2);
    
    return descartes_core::TrajectoryPtPtr (new CartTrajectoryPt(
                          TolerancedFrame(pose,
                            ToleranceBase::createSymmetric<PositionTolerance>(x, y, z, pos_tol(0), pos_tol(1), pos_tol(2)),
                            ToleranceBase::createSymmetric<OrientationTolerance>(rx, ry, rz, orient_tol(0), orient_tol(1), orient_tol(2))),
                          ds,
                          dtheta,
                          timing));
  }

  // Translates descartes trajectory points to ROS trajectory Ppoints
  void populateTrajectoryMsg(const TrajectoryVec& solution,
                             const descartes_core::RobotModel& robot_model,
                             trajectory_msgs::JointTrajectory& trajectory);
  

}

#endif
