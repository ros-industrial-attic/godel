/*
 * robot_blending_plugin.cpp
 *
 *  Created on: Feb 10, 2014
 *      Author: ros-industrial
 */

#include <robot_blending_rqt_plugin/rqt_plugins/robot_blending_plugin.h>
#include <pluginlib/class_list_macros.h>

namespace robot_blending_rqt_plugin{ namespace rqt_plugins{

const std::string RobotBlendingPlugin::QOBJECT_NAME = "RobotBlending";

RobotBlendingPlugin::RobotBlendingPlugin():
		rqt_gui_cpp::Plugin(),
		widget_(0)
{
	setObjectName(QString::fromStdString(QOBJECT_NAME));
}

RobotBlendingPlugin::~RobotBlendingPlugin() {
	// TODO Auto-generated destructor stub
}

void RobotBlendingPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
	widget_ = new QWidget();
	ui_.setupUi(widget_);
	context.addWidget(widget_);
}

void RobotBlendingPlugin::shutdownPlugin()
{

}
void RobotBlendingPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings,
		qt_gui_cpp::Settings& instance_settings) const
{

}

void RobotBlendingPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
		const qt_gui_cpp::Settings& instance_settings)
{

}

}}

PLUGINLIB_EXPORT_CLASS(robot_blending_rqt_plugin::rqt_plugins::RobotBlendingPlugin, rqt_gui_cpp::Plugin)

