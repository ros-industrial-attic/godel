
// register IKFastKinematicsPlugin as a KinematicsBase implementation
#include <motoman_sia20d_ikfast_manipulator/motoman_sia20d_manipulator_ikfast_moveit_plugin.hpp>
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(
    motoman_sia20d_ikfast_manipulator::ikfast_kinematics_plugin::IKFastKinematicsPlugin,
    kinematics::KinematicsBase);
