#include "common_utils.h"
#include <eigen_conversions/eigen_msg.h>
#include <descartes_planner/dense_planner.h>
#include <descartes_planner/sparse_planner.h>
#include <descartes_trajectory/axial_symmetric_pt.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/GetMotionPlan.h>

#include <ros/topic.h>
#include "trajectory_utils.h"

// Constants
const static double DEFAULT_TIME_UNDEFINED_VELOCITY =
    1.0; // When a Descartes trajectory point has no timing info associated
         // with it, this value (in seconds) is used instead
const static std::string DEFAULT_FRAME_ID =
    "world_frame"; // The default frame_id used for trajectories generated
                   // by these helper functions
const static double DEFAULT_ANGLE_DISCRETIZATION =
    M_PI / 12.0; // Default discretization used for axially-symmetric points
                 // in these helper functions
const static double DEFAULT_JOINT_WAIT_TIME = 5.0; // Maximum time allowed to capture a new joint
                                                   // state message
const static double DEFAULT_JOINT_VELOCITY = 0.3; // rad/s

// MoveIt Configuration Constants
const static int DEFAULT_MOVEIT_NUM_PLANNING_ATTEMPTS = 20;
const static double DEFAULT_MOVEIT_PLANNING_TIME = 20.0;   // seconds
const static double DEFAULT_MOVEIT_VELOCITY_SCALING = 0.1; // Slow down the robot
const static std::string DEFAULT_MOVEIT_PLANNER_ID = "RRTConnectkConfigDefault";
const static std::string DEFAULT_MOVEIT_FRAME_ID = "world_frame";
const static std::string DEFAULT_MOVEIT_PLANNING_SERVICE_NAME = "plan_kinematic_path";

Eigen::Affine3d godel_process_planning::createNominalTransform(const geometry_msgs::Pose& ref_pose,
                                                               const geometry_msgs::Point& pt)
{
  Eigen::Affine3d eigen_pose;
  Eigen::Vector3d eigen_pt;

  tf::poseMsgToEigen(ref_pose, eigen_pose);
  tf::pointMsgToEigen(pt, eigen_pt);

  // Translation transform
  Eigen::Affine3d to_point;
  to_point = Eigen::Translation3d(eigen_pt);

  // Reverse the Z axis
  Eigen::Affine3d flip_z;
  flip_z = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());

  return eigen_pose * to_point * flip_z;
}

Eigen::Affine3d godel_process_planning::createNominalTransform(const geometry_msgs::Pose& ref_pose,
                                                               const double z_adjust)
{
  Eigen::Affine3d eigen_pose;

  tf::poseMsgToEigen(ref_pose, eigen_pose);

  return createNominalTransform(eigen_pose, z_adjust);
}

Eigen::Affine3d godel_process_planning::createNominalTransform(const Eigen::Affine3d &ref_pose,
                                                               const double z_adjust)
{
  // Reverse the Z axis
  Eigen::Affine3d flip_z;
  flip_z = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());

  return ref_pose * Eigen::Translation3d(0, 0, z_adjust) * flip_z;
}

bool godel_process_planning::descartesSolve(const godel_process_planning::DescartesTraj& in_path,
                                            descartes_core::RobotModelConstPtr robot_model,
                                            godel_process_planning::DescartesTraj& out_path)
{
  // Create planner
  descartes_core::PathPlannerBasePtr planner(new descartes_planner::DensePlanner);
  if (!planner->initialize(robot_model))
  {
    ROS_ERROR("%s: Failed to initialize planner with robot model", __FUNCTION__);
    return false;
  }

  if (!planner->planPath(in_path))
  {
    ROS_ERROR("%s: Failed to plan for given trajectory.", __FUNCTION__);
    return false;
  }

  if (!planner->getPath(out_path))
  {
    ROS_ERROR("%s: Failed to retrieve path.", __FUNCTION__);
    return false;
  }

  return true;
}


static double calcDefaultTime(const std::vector<double>& a, const std::vector<double>& b,
                              double max_joint_vel)
{
  ROS_ASSERT(a.size() == b.size());
  ROS_ASSERT(a.size() > 0);
  std::vector<double> result (a.size());
  std::transform(a.begin(), a.end(), b.begin(), result.begin(), [] (double a, double b)
  {
    return std::abs(a - b);
  });

  return *std::max_element(result.begin(), result.end()) / max_joint_vel;
}

trajectory_msgs::JointTrajectory
godel_process_planning::toROSTrajectory(const godel_process_planning::DescartesTraj& solution,
                                        const descartes_core::RobotModel& model)
{
  ros::Duration from_start(0.0);
  std::vector<double> joint_point;
  std::vector<double> dummy;
  trajectory_msgs::JointTrajectory ros_trajectory; // result

  for (std::size_t i = 0; i < solution.size(); ++i)
  {
    solution[i]->getNominalJointPose(dummy, model, joint_point);

    trajectory_msgs::JointTrajectoryPoint pt;
    pt.positions = joint_point;
    pt.velocities.resize(joint_point.size(), 0.0);
    pt.accelerations.resize(joint_point.size(), 0.0);
    pt.effort.resize(joint_point.size(), 0.0);

    double time_step = solution[i]->getTiming().upper; // request descartes timing
    if (time_step == 0.0)
    {
      if (i == 0)
        from_start += ros::Duration(DEFAULT_TIME_UNDEFINED_VELOCITY); // default time
      else
      {
        // If we have a previous point then it makes more sense to set the time of the
        // motion based on the largest joint motion required between two points and a
        // default velocity.
        const auto& prev = ros_trajectory.points.back().positions;
        const auto& next = pt.positions;
        const auto td = calcDefaultTime(prev, next, DEFAULT_JOINT_VELOCITY);
        from_start += ros::Duration(td);
      }
    }
    else
      from_start += ros::Duration(time_step);

    pt.time_from_start = from_start;

    ros_trajectory.points.push_back(pt);
  }

  return ros_trajectory;
}

void godel_process_planning::fillTrajectoryHeaders(const std::vector<std::string>& joints,
                                                   trajectory_msgs::JointTrajectory& traj)
{
  traj.joint_names = joints;
  traj.header.frame_id = DEFAULT_FRAME_ID;
  traj.header.stamp = ros::Time::now();
}

std::vector<double> godel_process_planning::getCurrentJointState(const std::string& topic)
{
  sensor_msgs::JointStateConstPtr state = ros::topic::waitForMessage<sensor_msgs::JointState>(
      topic, ros::Duration(DEFAULT_JOINT_WAIT_TIME));
  if (!state)
    throw std::runtime_error("Joint state message capture failed");
  return state->position;
}

godel_process_planning::DescartesTraj
godel_process_planning::createLinearPath(const Eigen::Affine3d& start, const Eigen::Affine3d& stop,
                                         double ds)
{
  using namespace descartes_trajectory;

  PoseVector cart_path = interpolateCartesian(start, stop, ds);
  DescartesTraj result;
  for (std::size_t i = 0; i < cart_path.size(); ++i)
  {
    result.push_back(
        boost::make_shared<AxialSymmetricPt>(cart_path[i], DEFAULT_ANGLE_DISCRETIZATION,
                                             descartes_trajectory::AxialSymmetricPt::Z_AXIS));
  }

  return result;
}

godel_process_planning::DescartesTraj
godel_process_planning::createJointPath(const std::vector<double>& start,
                                        const std::vector<double>& stop, double dtheta)
{
  JointVector path = interpolateJoint(start, stop, dtheta);
  DescartesTraj result;
  for (std::size_t i = 0; i < path.size(); ++i)
  {
    result.push_back(boost::make_shared<descartes_trajectory::JointTrajectoryPt>(path[i]));
  }
  return result;
}

trajectory_msgs::JointTrajectory godel_process_planning::getMoveitPlan(
    const std::string& group_name, const std::vector<double>& joints_start,
    const std::vector<double>& joints_stop, moveit::core::RobotModelConstPtr model)
{
  const moveit::core::JointModelGroup* group = model->getJointModelGroup(group_name);
  robot_state::RobotState goal_state(model);
  goal_state.setJointGroupPositions(group, joints_stop);

  moveit_msgs::GetMotionPlan::Request req;
  req.motion_plan_request.group_name = group_name;
  req.motion_plan_request.num_planning_attempts = DEFAULT_MOVEIT_NUM_PLANNING_ATTEMPTS;
  req.motion_plan_request.max_velocity_scaling_factor = DEFAULT_MOVEIT_VELOCITY_SCALING;
  req.motion_plan_request.allowed_planning_time = DEFAULT_MOVEIT_PLANNING_TIME;
  req.motion_plan_request.planner_id = DEFAULT_MOVEIT_PLANNER_ID;

  req.motion_plan_request.workspace_parameters.header.frame_id = model->getRootLinkName();
  req.motion_plan_request.workspace_parameters.header.stamp = ros::Time::now();

  // Set the start state
  // Will want to add options here to start from a state that's not the start state
  moveit_msgs::RobotState start_state;
  start_state.is_diff = false;
  sensor_msgs::JointState joint_state;
  joint_state.name = group->getActiveJointModelNames();
  joint_state.position = joints_start;
  start_state.joint_state = joint_state;
  req.motion_plan_request.start_state = start_state;

  // Set the goal state
  moveit_msgs::Constraints c = kinematic_constraints::constructGoalConstraints(goal_state, group);
  req.motion_plan_request.goal_constraints.push_back(c);

  // Make connection the planning-service offered by the MoveIt MoveGroup node
  ros::NodeHandle nh;
  ros::ServiceClient client =
      nh.serviceClient<moveit_msgs::GetMotionPlan>(DEFAULT_MOVEIT_PLANNING_SERVICE_NAME);

  trajectory_msgs::JointTrajectory jt;
  moveit_msgs::GetMotionPlan::Response res;
  if (client.call(req, res))
  {
    jt = res.motion_plan_response.trajectory.joint_trajectory;
  }
  else
  {
    ROS_ERROR("%s: Unable to call MoveIt path planning service: '%s' or planning failed",
              __FUNCTION__, DEFAULT_MOVEIT_PLANNING_SERVICE_NAME.c_str());
    throw std::runtime_error("Unable to generate MoveIt path plan");
  }
  return jt;
}

trajectory_msgs::JointTrajectory godel_process_planning::planFreeMove(
    descartes_core::RobotModel& model, const std::string& group_name,
    moveit::core::RobotModelConstPtr moveit_model, const std::vector<double>& start,
    const std::vector<double>& stop)
{
  // Attempt joint interpolated motion
  DescartesTraj joint_approach = createJointPath(start, stop);

  // Check approach for collisions
  bool collision_free = true;
  for (std::size_t i = 0; i < joint_approach.size(); ++i)
  {
    if (!joint_approach[i]->isValid(model))
    {
      collision_free = false;
      break;
    }
  }

  // If the method is collision free, then we use the interpolation
  // otherwise let moveit try
  if (collision_free)
  {
    return toROSTrajectory(joint_approach, model);
  }
  else
  {
    return godel_process_planning::getMoveitPlan(group_name, start, stop, moveit_model);
  }
}

std::vector<std::vector<double>>
godel_process_planning::filterColliding(descartes_core::RobotModel& model,
                                        const std::vector<std::vector<double>>& candidates)
{
  std::vector<std::vector<double>> results;

  for (const auto& c : candidates)
  {
    if (model.isValid(c))
      results.push_back(c);
  }
  return results;
}


double godel_process_planning::freeSpaceCostFunction(const std::vector<double> &source,
                                                     const std::vector<double> &target)
{
  const double FREE_SPACE_MAX_ANGLE_DELTA =
      M_PI; // The maximum angle a joint during a freespace motion
              // from the start to end position without that motion
              // being penalized. Avoids flips.
  const double FREE_SPACE_ANGLE_PENALTY =
      2.0; // The factor by which a joint motion is multiplied if said
           // motion is greater than the max.

  // The cost function here penalizes large single joint motions in an effort to
  // keep the robot from flipping a joint, even if some other joints have to move
  // a bit more.
  double cost = 0.0;
  for (std::size_t i = 0; i < source.size(); ++i)
  {
    double diff = std::abs(source[i] - target[i]);
    if (diff > FREE_SPACE_MAX_ANGLE_DELTA)
      cost += FREE_SPACE_ANGLE_PENALTY * diff;
    else
      cost += diff;
  }
  return cost;
}

EigenSTL::vector_Affine3d godel_process_planning::linearMoveZ(const Eigen::Affine3d &origin, double step_size, int steps)
{
  EigenSTL::vector_Affine3d result (steps);

  for (int i = 0; i < steps; ++i)
  {
    result[i] = origin * Eigen::Translation3d(0, 0, step_size * (i + 1));
  }

  return result;
}

EigenSTL::vector_Affine3d godel_process_planning::toEigenArray(const geometry_msgs::PoseArray &geom_poses)
{
  EigenSTL::vector_Affine3d result (geom_poses.poses.size());
  std::transform(geom_poses.poses.begin(), geom_poses.poses.end(), result.begin(), [](const geometry_msgs::Pose& pose) {
    Eigen::Affine3d e;
    tf::poseMsgToEigen(pose, e);
    return e;
  });
  return result;
}

std::vector<EigenSTL::vector_Affine3d> godel_process_planning::toEigenArrays(const std::vector<geometry_msgs::PoseArray> &poses)
{
  std::vector<EigenSTL::vector_Affine3d> result (poses.size());
  std::transform(poses.begin(), poses.end(), result.begin(), [](const geometry_msgs::PoseArray& p) {
    return toEigenArray(p);
  });
  return result;
}
