/*
 * robot_blending_panel.h
 *
 *  Created on: Mar 13, 2014
 *      Author: ros developer 
 */

#ifndef ROBOT_BLENDING_PANEL_H_
#define ROBOT_BLENDING_PANEL_H_

#include <ros/ros.h>
#include <rviz/panel.h>
#include <godel_plugins/widgets/robot_blending_widget.h>

namespace godel_plugins {
namespace rviz_panels {

class RobotBlendingPanel: public rviz::Panel {
Q_OBJECT
public:
	RobotBlendingPanel(QWidget* parent = 0);
	virtual ~RobotBlendingPanel();

	virtual void onInitialize();

protected Q_SLOTS:

	// rviz::Panel virtual functions
	virtual void load(const rviz::Config& config);
	virtual void save(rviz::Config config) const;

protected:

	QWidget *widget_;
};

} /* namespace rviz_panels */
} /* namespace godel_plugins */
#endif /* ROBOT_BLENDING_PANEL_H_ */
