#include "godel_path_planning/trajectory_planning.h"

#include "descartes_trajectory/cart_trajectory_pt.h"
#include <godel_path_planning/trajectory_interpolator.h>
#include "descartes_planner/dense_planner.h"

#include <tf_conversions/tf_eigen.h>

#include <descartes_core/robot_model.h>
#include <pluginlib/class_loader.h>

#include "godel_path_planning/trajectory_multipoint.h"
#include "godel_path_planning/trajectory_interpolator.h"

#include <sensor_msgs/JointState.h>

#include <ros/ros.h>

namespace
{
  const static std::string ROBOT_DESCRIPTION = "robot_description";

  /* This will allow us to adapt to a different robot model by dynamically loading its
     interface. */
  static const std::string PLUGIN_NAME = "motoman_sia20d_descartes/MotomanSia20dRobotModel";
  
  static pluginlib::ClassLoader<descartes_core::RobotModel> robot_model_loader("descartes_core",
                                                                     "descartes_core::RobotModel");
  
  // Create a descartes RobotModel for graph planning
  descartes_core::RobotModelPtr createRobotModel(const std::string& group_name,
                                                 const std::string& tool_frame,
                                                 const std::string& world_frame)
  {
    using descartes_core::RobotModelPtr;

    RobotModelPtr ptr = robot_model_loader.createInstance(PLUGIN_NAME);
    ptr->initialize(ROBOT_DESCRIPTION, group_name, world_frame, tool_frame);
    return ptr;
  }

  descartes_core::TrajectoryPtPtr rotationalMultipoint(double x, double y, double z, double rx, double ry, double rz,
                                                       double discretization, const descartes_core::TimingConstraint& timing,
                                                       double drx = 0.0, double dry = 0.0, double drz = 0.3)
  {
    using namespace descartes_core;
    using namespace descartes_trajectory;

    CartTrajectoryPt a(
              TolerancedFrame(utils::toFrame(x,y, z, rx, ry, rz, descartes_core::utils::EulerConventions::XYZ),
               ToleranceBase::zeroTolerance<PositionTolerance>(x, y, z),
               ToleranceBase::createSymmetric<OrientationTolerance>(rx, ry, rz-(M_PI/2.0), drx, dry, drz)),
              0.0, discretization, timing);

    
    CartTrajectoryPt b(
              TolerancedFrame(utils::toFrame(x,y, z, rx, ry, rz, descartes_core::utils::EulerConventions::XYZ),
               ToleranceBase::zeroTolerance<PositionTolerance>(x, y, z),
               ToleranceBase::createSymmetric<OrientationTolerance>(rx, ry, rz+(M_PI/2.0), 0, 0, 0.3)),
              0.0, discretization, timing);

    std::vector<CartTrajectoryPt> pts;
    pts.push_back(a); 
    pts.push_back(b);

    return descartes_core::TrajectoryPtPtr(new godel_path_planning::CartTrajectoryMultiPt(pts, timing));
  }

  // Create an axial trajectory pt from a given tf transform
  descartes_core::TrajectoryPtPtr tfToAxialTrajectoryPt(const tf::Transform& nominal, double discretization, bool blend_path)
  {
    using namespace descartes_core;
    using namespace descartes_trajectory;

    Eigen::Affine3d eigen_pose;
    tf::poseTFToEigen(nominal,eigen_pose);
    Eigen::Vector3d rpy =  eigen_pose.rotation().eulerAngles(0,1,2);

    double rx = rpy(0);
    double ry = rpy(1);
    double rz = rpy(2);
    double x = eigen_pose.translation()(0);
    double y = eigen_pose.translation()(1);
    double z = eigen_pose.translation()(2);

    static const descartes_core::TimingConstraint timing(0.0, 0.15);

    if (blend_path)
    {
      return boost::shared_ptr<TrajectoryPt>(
        new CartTrajectoryPt(
              TolerancedFrame(utils::toFrame(x,y, z, rx, ry, rz, descartes_core::utils::EulerConventions::XYZ),
               ToleranceBase::zeroTolerance<PositionTolerance>(x, y, z),
               ToleranceBase::createSymmetric<OrientationTolerance>(rx, ry, 0, 0, 0, 2*M_PI)),
              0.0, discretization, timing));
    } 
    else
    {
      return rotationalMultipoint(x, y, z, rx, ry, rz, discretization, timing);
    }
  }

  // Translates a point relative to a reference pose to an absolute transformation
  // Also flips the z axis of the orientation to point INTO the plane specified by the
  // reference frame.
  // TODO: use Eigen instead of TF. Descartes uses Eigen, and so should we
  tf::Transform createNominalTransform(const geometry_msgs::Pose& ref_pose, const geometry_msgs::Point& point)
  {
    // To plane
    tf::Transform marker_pose;
    tf::poseMsgToTF(ref_pose, marker_pose);
    
    // From plane to point
    tf::Transform to_point = tf::Transform::getIdentity();
    to_point.setOrigin(tf::Vector3(point.x, point.y, point.z));
    // Reverse orientation of z axis
    tf::Quaternion quat;
    quat.setEuler(0.0, M_PI, 0.0); // yaw, pitch roll
    tf::Transform flip_z (quat);
    // Calculate transform
    tf::Transform in_world = marker_pose * to_point * flip_z;

    return in_world;
  }

  double timeParamiterize(const std::vector<double>& start,
                          const std::vector<double>& stop)
  {
    const static std::vector<double> vel {0.2, 0.2, 0.2, 0.2, 0.3, 0.3, 0.3};

    double max_time = 0.0;
    for (std::size_t i = 0; i < start.size(); ++i)
    {
      max_time = std::max(max_time, std::fabs((stop[i] - start[i]) / vel[i]));
    }
    return max_time;
  }

  void applyTimeParamiterization(std::vector<descartes_core::TrajectoryPtPtr>& traj, 
                                 const descartes_core::RobotModel& robot_model)
  {
    std::vector<double> dummy;
    // Attempt to smooth the velocity of the robot
    for (auto it = traj.begin(); it != traj.end() - 1; ++it)
    {
      std::vector<double> start;
      it->get()->getNominalJointPose(dummy, robot_model, start);
      std::vector<double> stop;
      (it + 1)->get()->getNominalJointPose(dummy, robot_model, stop);
      double dt = timeParamiterize(start, stop);
      descartes_core::TimingConstraint tm (0.0, dt);
      (it + 1)->get()->setTiming(tm);
    }
  }



  // Return the current joint positions from a given topic
  std::vector<double> getCurrentPosition(const std::string& topic)
  {
    sensor_msgs::JointStateConstPtr state = ros::topic::waitForMessage<sensor_msgs::JointState>(topic, ros::Duration(1.0));
    return state->position;
  }

} // end of anon namespace

bool godel_path_planning::generateTrajectory(const godel_msgs::TrajectoryPlanning::Request& req,
                                             trajectory_msgs::JointTrajectory& trajectory,
                                             const moveit::core::RobotModelConstPtr& model)
{
  using descartes_core::TrajectoryPtPtr;

  if (req.path.points.empty())
  {
    ROS_WARN("%s: recieved trajectory planning request with no points.", __FUNCTION__);
    return false;
  }

  descartes_core::RobotModelConstPtr robot_model = createRobotModel(req.group_name, 
                                                                    req.tool_frame, 
                                                                    req.world_frame);

  TrajectoryVec graph_points;
  graph_points.reserve(req.path.points.size());

  // Create cartesian trajectory points out of the tool processing path
  for (size_t i = 0; i < req.path.points.size(); ++i)
  {
    // compute the absolute transform of a given point given its 
    // reference plane and relative position to that plane
    tf::Transform point_tf = createNominalTransform(req.path.reference, req.path.points[i]);
    graph_points.push_back(tfToAxialTrajectoryPt(point_tf, req.angle_discretization, req.is_blending_path));
  }

  size_t nPointsInGoto = 0;

  if (req.plan_from_current_position)
  {
    // Add segment connecting current position with start of trajectory
    std::vector<double> init_state = getCurrentPosition("joint_states");
    // Get FK
    Eigen::Affine3d init_pose;
    if (!robot_model->getFK(init_state, init_pose))
    {
      ROS_ERROR("Could not get FK of initial position");
      return false;
    }
    // Get Pose of first point in process path
    Eigen::Affine3d stop_pose;
    graph_points.front()->getNominalCartPose(std::vector<double>(), *robot_model, stop_pose);
    // Interpolate between the current pos and start of path 
    TrajectoryVec to_process = interpolateCartesian(init_pose, stop_pose, 15.0, 0.05);
    // replace the first trajectorypt with a fixed one
    nPointsInGoto = to_process.size();
    to_process.front() = descartes_core::TrajectoryPtPtr(new descartes_trajectory::JointTrajectoryPt(init_state));
    // Unconstrain the transition from the cartesian motion portion to the tool trajectory
    // which might have much tighter tolerances
    graph_points.front()->setTiming(descartes_core::TimingConstraint(0,0));
    // concatenate paths
    graph_points.insert(graph_points.begin(), to_process.begin(), to_process.end());
  }

  // Create planner
  descartes_core::PathPlannerBasePtr planner (new descartes_planner::DensePlanner);
  planner->initialize(robot_model);

  // Attempt to solve the trajectory
  if (!planner->planPath(graph_points))
  {
    ROS_ERROR("%s: Failed to plan for given trajectory.", __FUNCTION__);
    return false;
  }

  TrajectoryVec result_path;
  if (!planner->getPath(result_path))
  {
    ROS_ERROR("%s: Failed to retrieve path.", __FUNCTION__);
    return false;
  }

  // Post process the resulting path
  // This step looks for jumps in joint positions of more than 45 degrees
  // and then tries to smooth them out using joint interpolated motion
  std::vector<double> dummy;
  TrajectoryVec traj;
  for (auto it = result_path.begin(); it != result_path.end() - 1; ++it)
  {
    std::vector<double> start;
    it->get()->getNominalJointPose(dummy, *robot_model, start);
    std::vector<double> stop;
    (it + 1)->get()->getNominalJointPose(dummy, *robot_model, stop);
    if (hasJointJump(start, stop, M_PI/4))
    {
      ROS_WARN_STREAM("Adding joint interp for jump");
      auto t = jointInterpolate(start, stop, 10.0, 10);
      nPointsInGoto += 10;
      traj.insert(traj.end(), t.begin(), t.end() );
    }
    else
    {
      traj.push_back(*it);
    }
  }

  // Attempt to smooth the velocity of the robot
  for (auto it = traj.begin(); it != traj.end() - 1 && it != traj.begin() + nPointsInGoto; ++it)
  {
    std::vector<double> start;
    it->get()->getNominalJointPose(dummy, *robot_model, start);
    std::vector<double> stop;
    (it + 1)->get()->getNominalJointPose(dummy, *robot_model, stop);
    double dt = timeParamiterize(start, stop);
    descartes_core::TimingConstraint tm (0.0, dt);
    (it + 1)->get()->setTiming(tm);
  }

  // Now translate the solution into ROS trajectory type
  
  // Retrieve active joint names for this planning group
  const std::vector< std::string >& joint_names = 
    model->getJointModelGroup(req.group_name)->getActiveJointModelNames(); 

  // trajectory header: 
  trajectory.header.stamp = ros::Time::now();
  trajectory.header.frame_id = req.world_frame;
  // fill in joint names (order matters) - might need to check
  trajectory.joint_names = joint_names;

  populateTrajectoryMsg(traj, *robot_model, trajectory);

  return true;
}
