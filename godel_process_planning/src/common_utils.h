#ifndef COMMON_UTILS_H
#define COMMON_UTILS_H

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>

#include <descartes_core/trajectory_pt.h>
#include <descartes_core/robot_model.h>

#include <sensor_msgs/JointState.h>

#include <Eigen/Geometry>

namespace godel_process_planning
{
typedef std::vector<descartes_core::TrajectoryPtPtr> DescartesTraj;

/**
   * @brief Converts a point relative to given pose into a robot tool pose (i.e. flip z)
   * @param ref_pose Reference pose (in world frame) for 'pt'
   * @param pt 3-dimensional offset from pose for the given point
   * @return Tool pose corresponding to this surface point
   */
Eigen::Affine3d createNominalTransform(const geometry_msgs::Pose& ref_pose,
                                       const geometry_msgs::Point& pt);

/**
   * @brief Converts a point relative to given pose into a robot tool pose (i.e. flip z)
   * @param ref_pose Reference pose (in world frame) for 'pt'
   * @param pt 3-dimensional offset from pose for the given point
   * @return Tool pose corresponding to this surface point
   */
Eigen::Affine3d createNominalTransform(const geometry_msgs::Pose& ref_pose, const double z_adjust = 0.0);


Eigen::Affine3d createNominalTransform(const Eigen::Affine3d& ref_pose, const double z_adjust = 0.0);

/**
 * @brief Given a path and robot model, this method creates a descartes planner and attempts to
 * solve the path
 * @param in_path Trajectory to solve
 * @param robot_model Robot model used for IK/FK by planner
 * @param out_path The solution path, if found, otherwise undefined
 * @return True if path was found and result placed inside 'out_path'; False otherwise.
 */
bool descartesSolve(const DescartesTraj& in_path, descartes_core::RobotModelConstPtr robot_model,
                    DescartesTraj& out_path);
/**
 * @brief Extracts joint position values from Descartes trajectory and packs them into a ROS message
 * @param solution The Descartes trajectory used to generate nominal joint trajectory
 * @param model The robot model used in generating the above trajectory
 * @return A ROS joint trajectory with only the 'points' field filled in; you must add header/joint
 * name info
 */
trajectory_msgs::JointTrajectory toROSTrajectory(const DescartesTraj& solution,
                                                 const descartes_core::RobotModel& model);
/**
 * @brief Updates the joint names, frame id, and time stamp of the given trajectory
 * @param joints Joint names; listed in same order as the values they correspond to
 * @param traj The trajectory to modify
 */
void fillTrajectoryHeaders(const std::vector<std::string>& joints,
                           trajectory_msgs::JointTrajectory& traj);

/**
 * @brief Attempts to capture the most recent JointState from the given topic
 * @param topic The joint topic name on which to listen for the robot state
 * @return The joint positions captured from the topic or a std::runtime_error
 */
std::vector<double> getCurrentJointState(const std::string& topic);

/**
 * @brief Creates descartes trajectory consisting of cartesian positions in a linear path between
 * start and stop
 * @param start The start pose of the linear interpolation
 * @param stop The final pose of the linear interpolation
 * @param ds The distance (m) between points in the linear path; defaults to 10cm
 * @return A linear interpolated path from start to stop in Descartes format
 */
DescartesTraj createLinearPath(const Eigen::Affine3d& start, const Eigen::Affine3d& stop,
                               double ds = 0.1);
/**
 * @brief Creates descartes trajectory consisting of joint interpolated positions from start to stop
 * @param start The initial joint configuration
 * @param stop The final joint configuration
 * @param dtheta The maximum angle change (radians) for any joint between sequential points
 * @return A joint-interpolated path from start to stop in Descartes format
 */
DescartesTraj createJointPath(const std::vector<double>& start, const std::vector<double>& stop,
                              double dtheta = M_PI / 180.0);
/**
 * @brief Invokes MoveGroup's 'plan_kinematic_path' service to create a collision free path between
 * joints_start and joints_stop
 * @brief group_name The name of the MoveIt move-group associated with this plan
 * @param joints_start The initial robot configuration
 * @param joints_stop The target robot configuration
 * @param model The MoveIt model containing the move-group 'group_name'
 * @return Joint Trajectory represeting motion from joints_start to joints_stop; or
 * std::runtime_error
 */
trajectory_msgs::JointTrajectory getMoveitPlan(const std::string& group_name,
                                               const std::vector<double>& joints_start,
                                               const std::vector<double>& joints_stop,
                                               moveit::core::RobotModelConstPtr model);
/**
 * @brief A helper function to make getting the nominal joint value out of a Descartes point more
 * concise
 * @param model The associated Descartes robot model
 * @param pt Descartes trajectory point from which to pull the joint values
 * @return The joint values from 'pt' obtained as a result of a call to getNominalJointPose()
 */
static inline std::vector<double> extractJoints(const descartes_core::RobotModel& model,
                                                const descartes_core::TrajectoryPt& pt)
{
  std::vector<double> dummy, result;
  pt.getNominalJointPose(dummy, model, result);
  return result;
}

/**
 * @brief A planning helper function for getting a valid path between start and stop; first attempts
 * a joint interpolated motion
 *        to see if its collision free. If not, it invokes MoveIt as a backup.
 * @param model Associated descartes robot model
 * @param group_name Name of moveit move-group associated with the moveit model
 * @param moveit_model the moveit robot description associated with this item
 * @param start Initial robot configuration
 * @param stop Final robot configuration
 * @return A collision-free path from start to stop
 */
trajectory_msgs::JointTrajectory planFreeMove(descartes_core::RobotModel& model,
                                              const std::string& group_name,
                                              moveit::core::RobotModelConstPtr moveit_model,
                                              const std::vector<double>& start,
                                              const std::vector<double>& stop);

/**
 * @brief Given a list of possible joint solutions, remove those that end in collision. Checking via 'model'.
 */
std::vector<std::vector<double>> filterColliding(descartes_core::RobotModel& model,
                                                 const std::vector<std::vector<double>>& candidates);

/**
 * @brief Given an initial position and a list of candidates, select the 'best' start position for process path.
 * @param start The initial pose of the robot (where you are moving from)
 * @param model The descartes RobotModel used for collision checking
 * @param candidates The possible candidate poses
 * @param cost_func A pointer to function of type "double(vector<double> a, vector<double> b)" used
 *        to order the non-colliding points
 * @return The 'best' pose or an exception if no candidates are valid
 */
template <typename CostFunc>
inline std::vector<double> pickBestStartPose(const std::vector<double>& start,
                                             descartes_core::RobotModel& model,
                                             const std::vector<std::vector<double>>& candidates,
                                             CostFunc cost_func)
{
  // step 1: Filter out candidates that are already in collision
  auto not_colliding = godel_process_planning::filterColliding(model, candidates);

  if (not_colliding.empty())
  {
    ROS_WARN("After filtering %lu possible process start poses, all were found"
             " to be in collision.", candidates.size());
    throw std::runtime_error("No valid starting poses");
  }

  // step 2: sort them by "closeness" cost
  std::sort(not_colliding.begin(), not_colliding.end(), [&start, cost_func]
            (const std::vector<double>& a, const std::vector<double>& b) {
    auto a_cost = cost_func(start, a);
    auto b_cost = cost_func(start, b);
    return a_cost < b_cost;
  });

  return not_colliding.front();
}

/**
 * @brief Computes a 'cost' value for a robot motion between 'source' and 'target'
 * @param source  The joint configuration at start of motion
 * @param target  The joint configuration at end of motion
 * @return cost value
 */
 double freeSpaceCostFunction(const std::vector<double>& source,
                              const std::vector<double>& target);

 EigenSTL::vector_Affine3d linearMoveZ(const Eigen::Affine3d& origin, double step_size, int steps);

 EigenSTL::vector_Affine3d toEigenArray(const geometry_msgs::PoseArray& geom_poses);

 std::vector<EigenSTL::vector_Affine3d> toEigenArrays(const std::vector<geometry_msgs::PoseArray>& poses);

}

#endif // COMMON_UTILS_H
