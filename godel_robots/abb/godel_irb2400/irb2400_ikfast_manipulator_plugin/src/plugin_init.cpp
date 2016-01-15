
// register IKFastKinematicsPlugin as a KinematicsBase implementation
#include <irb2400_ikfast_manipulator_plugin/abb_irb2400_manipulator_ikfast_moveit_plugin.hpp>
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(irb2400_ikfast_manipulator_plugin::IKFastKinematicsPlugin,
                       kinematics::KinematicsBase);
