
//register IKFastKinematicsPlugin as a KinematicsBase implementation
#include <irb6640_ikfast_manipulator_plugin/abb_irb6640_manipulator_ikfast_moveit_plugin.hpp>
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(irb6640_ikfast_manipulator_plugin::IKFastKinematicsPlugin, kinematics::KinematicsBase);
