/*
 * robot_blending_widget.h
 *
 *  Created on: Mar 13, 2014
 *      Author: ros developer 
 */

#ifndef ROBOT_BLENDING_WIDGET_H_
#define ROBOT_BLENDING_WIDGET_H_

#include <ui_robot_blending_plugin.h>
#include <godel_surface_detection/detection/surface_detection.h>
#include <godel_surface_detection/interactive/interactive_surface_server.h>
#include <QWidget>

namespace godel_plugins
{
namespace widgets {

class RobotBlendingWidget:  public QWidget
{
Q_OBJECT
public:
	RobotBlendingWidget();
	virtual ~RobotBlendingWidget();

protected:
	void init();

protected Q_SLOTS:

	void acquire_button_handler();
	void update();

protected:
	Ui::RobotBlendingWidget ui_;
	godel_surface_detection::detection::SurfaceDetection surf_detect_;
	godel_surface_detection::interactive::InteractiveSurfaceServer surf_server_;
};

} /* namespace widgets */
}
#endif /* ROBOT_BLENDING_WIDGET_H_ */
