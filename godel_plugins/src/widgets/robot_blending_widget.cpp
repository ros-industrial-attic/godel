/*
	Copyright Mar 13, 2014 Southwest Research Institute

	Licensed under the Apache License, Version 2.0 (the "License");
	you may not use this file except in compliance with the License.
	You may obtain a copy of the License at

		http://www.apache.org/licenses/LICENSE-2.0

	Unless required by applicable law or agreed to in writing, software
	distributed under the License is distributed on an "AS IS" BASIS,
	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
	See the License for the specific language governing permissions and
	limitations under the License.
*/

#include <godel_plugins/widgets/robot_blending_widget.h>
#include <QTimer>

namespace godel_plugins
{
namespace widgets {

RobotBlendingWidget::RobotBlendingWidget()
{
	// TODO Auto-generated constructor stub
	init();
}

RobotBlendingWidget::~RobotBlendingWidget() {
	// TODO Auto-generated destructor stub
}

void RobotBlendingWidget::init()
{
	// initializing surface detector
	if(surf_detect_.init() && surf_server_.init())
	{
		ROS_INFO_STREAM("Parameters for surface detector and server loaded successfully");
	}
	else
	{
		ROS_ERROR_STREAM("Parameters for surface detector or server failed to load, using defaults");
	}

	// start server
	surf_server_.run();

	// initializing gui
	ui_.setupUi(this);
	ui_.TabWidget->setCurrentIndex(0);
	ui_.LineEditSensorTopic->setText(QString::fromStdString(surf_detect_.acquisition_topic_));
	ui_.SpinBoxAcquisitionTime->setValue(static_cast<int>(surf_detect_.acquisition_time_));

	// setting signals and slots
	connect(ui_.PushButtonAcquire,SIGNAL(clicked()),this,SLOT(acquire_button_handler()));

	// setting up timer
	QTimer *timer = new QTimer(this);
	connect(timer,SIGNAL(timeout()),this,SLOT(update()));
	timer->start(1000);
}

void RobotBlendingWidget::update()
{
	//ros::spinOnce();
	if(surf_server_.get_surface_count() == 0)
	{
		// adding markers to server
		visualization_msgs::MarkerArray markers_msg = surf_detect_.get_surface_markers();
		surf_server_.remove_all_surfaces();
		for(int i =0;i < markers_msg.markers.size();i++)
		{
			surf_server_.add_surface(markers_msg.markers[i]);
		}
	}
}

void RobotBlendingWidget::acquire_button_handler()
{
	surf_detect_.acquisition_time_ = ui_.SpinBoxAcquisitionTime->value();
	surf_detect_.acquisition_topic_ = ui_.LineEditSensorTopic->text().toStdString();
	if(surf_detect_.acquire_data())
	{
		ROS_INFO_STREAM("Acquisition succeeded");
		if(surf_detect_.find_surfaces())
		{
			surf_server_.remove_all_surfaces();
		}
	}
	else
	{
		ROS_ERROR_STREAM("Acquisition failed");
	}
}

} /* namespace widgets */

}
