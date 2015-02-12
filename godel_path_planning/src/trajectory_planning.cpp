#include "godel_path_planning/trajectory_planning.h"

#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include "descartes_core/cart_trajectory_pt.h"
#include "descartes_moveit/moveit_state_adapter.h"
#include "descartes_core/planning_graph.h"
#include "descartes_core/sparse_planner.h"

#include <tf_conversions/tf_eigen.h>


// Anonymous namespace
namespace
{
  // Create a descartes RobotModel for graph planning
  descartes_core::RobotModelPtr createRobotModel(const moveit::core::RobotStatePtr robot_state,
                                                 const std::string& group_name,
                                                 const std::string& tool_frame,
                                                 const std::string& world_frame)
  {
    boost::shared_ptr<descartes_core::RobotModel> result(new descartes_moveit::MoveitStateAdapter(*robot_state, group_name, tool_frame, world_frame, 2));
    return result; 
  }

  // Create a CartTrajectoryPt that defines a point & axis with free rotation about z
  descartes_core::TrajectoryPtPtr pointAxisTrajectoryFactory(double x, double y, double z, 
                                                             double rx, double ry, double rz)
  {
    using namespace descartes_core;

    return boost::shared_ptr<TrajectoryPt>(
      new CartTrajectoryPt(
            TolerancedFrame(utils::toFrame(x,y, z, rx, ry, rz, descartes_core::utils::EulerConventions::XYZ),
             ToleranceBase::zeroTolerance<PositionTolerance>(x, y, z),
             ToleranceBase::createSymmetric<OrientationTolerance>(rx, ry, 0, 0, 0, 2.0 * M_PI)),
            0.0, 0.3));
  }

  // Create an axial trajectory pt from a given tf transform
  descartes_core::TrajectoryPtPtr tfToAxialTrajectoryPt(const tf::Transform& nominal)
  {
    Eigen::Affine3d eigen_pose;
    tf::poseTFToEigen(nominal,eigen_pose);
    Eigen::Vector3d rpy =  eigen_pose.rotation().eulerAngles(0,1,2);

    double rx = rpy(0);
    double ry = rpy(1);
    double rz = rpy(2);
    double x = eigen_pose.translation()(0);
    double y = eigen_pose.translation()(1);
    double z = eigen_pose.translation()(2);

    return pointAxisTrajectoryFactory(x, y, z , rx, ry, rz);
  }

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
    quat.setEuler(0.0, 3.14159, 0.0); // yaw, pitch roll
    tf::Transform flip_z (quat);
    // Calculate transform
    tf::Transform in_world = marker_pose * to_point * flip_z;

    return in_world;
  }

} // end of anon namespace

bool godel_path_planning::generateTrajectory(const godel_msgs::TrajectoryPlanning::Request& req,
                                             trajectory_msgs::JointTrajectory& trajectory)
{
  using descartes_core::TrajectoryPtPtr;

  moveit::planning_interface::MoveGroup group(req.group_name);

  descartes_core::RobotModelConstPtr robot_model = createRobotModel(group.getCurrentState(),
                                                                    req.group_name, "tool0", 
                                                                    "world_frame");

  // if (tool_points.markers.empty())
  // {
  //   ROS_ERROR("%s: tool_points was empty.", __FUNCTION__);
  //   return false;
  // }

  // if (tool_points.markers[0].points.empty())
  // {
  //   ROS_ERROR("%s: sub-sequence %d of tool points is empty", __FUNCTION__, 0);
  //   return false;
  // }

  // std::vector<TrajectoryPtPtr> pts;

  // ROS_INFO_STREAM("N POINTS: " << pts.size());

  // for (size_t i = 0; i < tool_points.markers[0].points.size(); ++i)
  // {
  //   tf::Transform tf_pt = createNominalTransform(tool_points.markers[0].pose, tool_points.markers[0].points[i]);
  //   pts.push_back(createTrajectoryPoint(tf_pt));
  // }

  // // descartes_core::PlanningGraph graph(robot_model);
  // descartes_core::SparsePlanner graph(robot_model);
  // ROS_INFO_STREAM("CREATING GRAPH");
  // // graph.insertGraph(&pts);
  // graph.setPoints(pts);
  // ROS_INFO_STREAM("GRAPH_DONE");

  // double cost;
  // std::list<descartes_core::JointTrajectoryPt> joints_sol;
  // ROS_INFO_STREAM("SOLVE GRAPH");
  // graph.getShortestPath(cost, joints_sol);
  // ROS_INFO_STREAM("SOLVE GRAPH DONE");


  return false;
}