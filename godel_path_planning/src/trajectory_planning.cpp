#include "godel_path_planning/trajectory_planning.h"

#include <moveit/robot_trajectory/robot_trajectory.h>

#include "descartes_trajectory/cart_trajectory_pt.h"
#include "descartes_planner/dense_planner.h"

#include <tf_conversions/tf_eigen.h>

#include <descartes_core/robot_model.h>
#include <pluginlib/class_loader.h>

#include "godel_path_planning/trajectory_multipoint.h"

// Anonymous namespace
namespace
{
  const static double TOOL_POINT_DELAY = 0.75;
  const static double SCAN_Z_ORIENTATION_OFFSET = M_PI_2;

  const static std::string robot_description = "robot_description";

  /* This will allow us to adapt to a different robot model by dynamically loading its
     interface. */
  static const std::string PLUGIN_NAME = "motoman_sia20d_descartes/MotomanSia20dRobotModel";
  static pluginlib::ClassLoader<descartes_core::RobotModel> robot_model_loader("descartes_core",
                                                                     "descartes_core::RobotModel");
  
  // Create a descartes RobotModel for graph planning
  descartes_core::RobotModelPtr createRobotModel(const moveit::core::RobotStatePtr robot_state,
                                                 const std::string& group_name,
                                                 const std::string& tool_frame,
                                                 const std::string& world_frame,
                                                 uint8_t iterations)
  {
    using descartes_core::RobotModelPtr;

    RobotModelPtr ptr = robot_model_loader.createInstance(PLUGIN_NAME);
    ptr->initialize(robot_description, group_name, world_frame, tool_frame);
    return ptr;
  }

  descartes_core::TrajectoryPtPtr rotationalMultipoint(double x, double y, double z, double rx, double ry, double rz, double plus_minus,
                                                       double discretization, const descartes_core::TimingConstraint& timing)
  {
    using namespace descartes_core;
    using namespace descartes_trajectory;

    CartTrajectoryPt a(
              TolerancedFrame(utils::toFrame(x,y, z, rx, ry, rz, descartes_core::utils::EulerConventions::XYZ),
               ToleranceBase::zeroTolerance<PositionTolerance>(x, y, z),
               ToleranceBase::createSymmetric<OrientationTolerance>(rx, ry, rz-plus_minus, 0, 0, 0.3)),
              0.0, discretization, timing);

    
    CartTrajectoryPt b(
              TolerancedFrame(utils::toFrame(x,y, z, rx, ry, rz, descartes_core::utils::EulerConventions::XYZ),
               ToleranceBase::zeroTolerance<PositionTolerance>(x, y, z),
               ToleranceBase::createSymmetric<OrientationTolerance>(rx, ry, rz+plus_minus, 0, 0, 0.3)),
              0.0, discretization, timing);

    std::vector<CartTrajectoryPt> pts;
    pts.push_back(a); pts.push_back(b);

    return descartes_core::TrajectoryPtPtr(new godel_path_planning::CartTrajectoryMultiPt(pts, timing));

  }

  // Create an axial trajectory pt from a given tf transform
  descartes_core::TrajectoryPtPtr tfToAxialTrajectoryPt(const tf::Transform& nominal, double discretization, bool free_z)
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

    static const descartes_core::TimingConstraint timing(0.0, 0.3);

    if (free_z)
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
      return rotationalMultipoint(x,y,z,rx,ry,rz,SCAN_Z_ORIENTATION_OFFSET,discretization,timing);
    }
  }

  // Translates a point relative to a reference pose to an absolute transformation
  // Also flips the z axis of the orientation to point INTO the plane specified by the
  // reference frame.
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

  void populateTrajectoryMsg(const std::vector<descartes_core::TrajectoryPtPtr>& solution,
                             const descartes_core::RobotModel& robot_model,
                             double interpoint_delay,
                             double time_offset,
                             trajectory_msgs::JointTrajectory& trajectory)
  {
    typedef std::vector<descartes_core::TrajectoryPtPtr>::const_iterator JointSolutionIterator;
    
    // For calculating the time_from_start field of the trajectoryPoint
    ros::Duration time_from_start(time_offset);
    std::vector<double> dummy_seed (robot_model.getDOF(), 0.0);

    for (JointSolutionIterator it = solution.begin(); it != solution.end(); ++it)
    {
      // Retrieve actual target joint angles from the polymorphic interface function
      std::vector<double> sol;
      it->get()->getNominalJointPose(dummy_seed, robot_model, sol);
      
      trajectory_msgs::JointTrajectoryPoint point;
      point.positions = sol;
      point.velocities.resize(sol.size(), 0.0); // Fill extra fields with zeroes for now
      point.accelerations.resize(sol.size(), 0.0);
      point.effort.resize(sol.size(), 0.0);
      point.time_from_start = time_from_start;

      // add trajectory point to array
      trajectory.points.push_back(point);

      // increment time so far by next duration
      time_from_start += ros::Duration(interpoint_delay);
    }

    return;
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

  // Note that there is both a descartes_core::RobotModel and a moveit RobotModel
  robot_state::RobotStatePtr kinematic_state (new robot_state::RobotState(model));

  descartes_core::RobotModelConstPtr robot_model = createRobotModel(kinematic_state,
                                                                    req.group_name, req.tool_frame, 
                                                                    req.world_frame, req.iterations);

  std::vector<TrajectoryPtPtr> graph_points;
  graph_points.reserve(req.path.points.size());

  // Create cartesian trajectory points out of the tool processing path
  for (size_t i = 0; i < req.path.points.size(); ++i)
  {
    // compute the absolute transform of a given point given its 
    // reference plane and relative position to that plane
    tf::Transform point_tf = createNominalTransform(req.path.reference, req.path.points[i]);
    graph_points.push_back(tfToAxialTrajectoryPt(point_tf, req.angle_discretization, req.free_z_rotation));
  }

  descartes_core::PathPlannerBasePtr planner (new descartes_planner::DensePlanner);
  planner->initialize(robot_model);

  if (!planner->planPath(graph_points))
  {
    ROS_ERROR("%s: Failed to plan for given trajectory.", __FUNCTION__);
    return false;
  }

  std::vector<TrajectoryPtPtr> result_path;
  if (!planner->getPath(result_path))
  {
    ROS_ERROR("%s: Failed to retrieve path.", __FUNCTION__);
    return false;
  }

  // Retrieve active joint names for this planning group
  const std::vector< std::string >& joint_names = 
    model->getJointModelGroup(req.group_name)->getActiveJointModelNames(); 

  // translate the solution to the path planner
  // trajectory header: 
  trajectory.header.stamp = ros::Time::now();
  trajectory.header.frame_id = req.world_frame;
  // fill in joint names (order matters) - might need to check
  trajectory.joint_names = joint_names;

  populateTrajectoryMsg(result_path, *robot_model, 2.0, req.interpoint_delay, trajectory);

  return true;
}