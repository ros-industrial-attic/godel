#include <ros/ros.h>
#include <descartes_core/robot_model.h>
#include <pluginlib/class_loader.h>
#include <gtest/gtest.h>

static const std::string PLUGIN_NAME = "motoman_sia20d_descartes/MotomanSia20dRobotModel";
static const std::string GROUP_NAME = "manipulator";
static const std::string WORLD_FRAME = "world_frame";
static const std::string TCP_FRAME = "tool0";
static const std::string ROBOT_DESCRIPTION = "robot_description";
Eigen::Affine3d tcp_pose;

pluginlib::ClassLoader<descartes_core::RobotModel> robot_model_loader("descartes_core",
                                                                      "descartes_core::RobotModel");
descartes_core::RobotModelPtr robot_model_ptr;

TEST(MotomanSia20dRobotModel, createPluginInstance)
{
  bool succeeded = false;
  try
  {
    robot_model_ptr = robot_model_loader.createInstance(PLUGIN_NAME);
    succeeded = true;
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_ERROR_STREAM("Failed to create plugin instance of '" << PLUGIN_NAME << "', " << ex.what());
  }
  EXPECT_TRUE(succeeded);
}

TEST(MotomanSia20dRobotModel, initializePlugin)
{
  EXPECT_TRUE(robot_model_ptr->initialize(ROBOT_DESCRIPTION, GROUP_NAME, WORLD_FRAME, TCP_FRAME));
}

TEST(MotomanSia20dRobotModel, getFK)
{
  std::vector<double> joints = {M_PI / 4, M_PI / 6, M_PI / 5, M_PI / 8, M_PI / 6, 0, M_PI / 6};
  EXPECT_TRUE(robot_model_ptr->getFK(joints, tcp_pose));
}

TEST(MotomanSia20dRobotModel, getAllIK)
{
  std::vector<std::vector<double> > solutions;
  EXPECT_TRUE(robot_model_ptr->getAllIK(tcp_pose, solutions));
  ROS_INFO_STREAM("Found " << solutions.size() << " ik solutions");
}
