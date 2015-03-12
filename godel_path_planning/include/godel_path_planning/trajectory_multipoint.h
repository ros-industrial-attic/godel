#ifndef TRAJECTORY_MULTIPOINT_H
#define TRAJECTORY_MULTIPOINT_H

#include <descartes_trajectory/cart_trajectory_pt.h>


namespace godel_path_planning
{

  class CartTrajectoryMultiPt : public descartes_trajectory::CartTrajectoryPt
  {
  public:
    CartTrajectoryMultiPt(const std::vector<descartes_trajectory::CartTrajectoryPt>& pts,
                      const descartes_core::TimingConstraint& timing)
      : descartes_trajectory::CartTrajectoryPt(timing)
      , points_(pts)
    {}

    virtual ~CartTrajectoryMultiPt() {}

    virtual void getJointPoses(const descartes_core::RobotModel &model,
                             std::vector<std::vector<double> > &joint_poses) const
    {
      joint_poses.clear();
      std::vector<std::vector<double> > poses;
      for (const auto& point : points_)
      {
        point.getJointPoses(model, poses);
        joint_poses.insert(joint_poses.end(), poses.begin(), poses.end());
      }
    }

  
  private:
    std::vector<descartes_trajectory::CartTrajectoryPt> points_;
  };


}

#endif
