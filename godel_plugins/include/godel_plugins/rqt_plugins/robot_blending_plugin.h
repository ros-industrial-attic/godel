/*
 * robot_blending_plugin.h
 *
 *  Created on: Feb 10, 2014
 *      Author: ros-industrial
 */

#ifndef ROBOT_BLENDING_PLUGIN_H_
#define ROBOT_BLENDING_PLUGIN_H_

#include <rqt_gui_cpp/plugin.h>
#include <ui_robot_blending_plugin.h>
#include <QWidget>

namespace godel_plugins{ namespace rqt_plugins{

class RobotBlendingPlugin : public rqt_gui_cpp::Plugin
{

public:
	static const std::string QOBJECT_NAME;


	Q_OBJECT
public:
	RobotBlendingPlugin();
	virtual ~RobotBlendingPlugin();

	virtual void initPlugin(qt_gui_cpp::PluginContext& context);
	virtual void shutdownPlugin();
	virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
			qt_gui_cpp::Settings& instance_settings) const;
	virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
			const qt_gui_cpp::Settings& instance_settings);

private:
	Ui::RobotBlendingWidget ui_;
	QWidget* widget_;
};

}}

#endif /* ROBOT_BLENDING_PLUGIN_H_ */
