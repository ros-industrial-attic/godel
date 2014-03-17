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

RobotBlendingWidget::RobotBlendingWidget(std::string ns):
		param_ns_(ns)
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
	if(surf_detect_.init(param_ns_) && surf_server_.init(param_ns_))
	{
		ROS_INFO_STREAM("Parameters for surface detector and server loaded successfully");
	}
	else
	{
		ROS_ERROR_STREAM("Parameters for surface detector or server failed to load, using defaults");
	}

	// start server
	godel_surface_detection::interactive::InteractiveSurfaceServer::SelectionCallback f =	boost::bind(
			&RobotBlendingWidget::emit_signal_selection_change,this);
	surf_server_.add_selection_callback(f);
	surf_server_.run();

	// initializing gui
	ui_.setupUi(this);
	ui_.TabWidget->setCurrentIndex(0);
	ui_.LineEditSensorTopic->setText(QString::fromStdString(surf_detect_.acquisition_topic_));
	ui_.SpinBoxAcquisitionTime->setValue(static_cast<int>(surf_detect_.acquisition_time_));

	// setting signals and slots
	connect(ui_.PushButtonAcquire,SIGNAL(clicked()),this,SLOT(acquire_button_handler()));
	connect(ui_.PushButtonNext,SIGNAL(clicked()),this,SLOT(increase_tab_index_handler()));
	connect(ui_.PushButtonBack,SIGNAL(clicked()),this,SLOT(decrease_tab_index_handler()));
	connect(ui_.PushButtonSelectAllSurfaces,SIGNAL(clicked()),this,SLOT(select_all_handler()));
	connect(ui_.PushButtonDeselectAllSurfaces,SIGNAL(clicked()),this,SLOT(deselect_all_handler()));
	connect(this,SIGNAL(selection_changed()),this,SLOT(selection_changed_handler()));


	// setting up timer
	QTimer *timer = new QTimer(this);
	connect(timer,SIGNAL(timeout()),this,SLOT(update_handler()));
	timer->start(1000);
}

void RobotBlendingWidget::select_all_handler()
{
	surf_server_.select_all(true);
}

void RobotBlendingWidget::deselect_all_handler()
{
	surf_server_.select_all(false);
}


void RobotBlendingWidget::update_handler()
{

}

void RobotBlendingWidget::selection_changed_handler()
{
	std::vector<std::string> list;
	surf_server_.get_selected_list(list);

	ui_.ListWidgetSelectedSurfs->clear();
	if(list.size() > 0)
	{
		for(std::vector<std::string>::iterator i = list.begin(); i != list.end();i++)
		{
			QListWidgetItem *item = new QListWidgetItem();
			item->setText(QString::fromStdString(*i));
			ui_.ListWidgetSelectedSurfs->addItem(item);

		}
	}
}

void RobotBlendingWidget::increase_tab_index_handler()
{
	ui_.TabWidgetCreateLib->setCurrentIndex(ui_.TabWidgetCreateLib->currentIndex() + 1);
}

void RobotBlendingWidget::decrease_tab_index_handler()
{
	ui_.TabWidgetCreateLib->setCurrentIndex(ui_.TabWidgetCreateLib->currentIndex() - 1);
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
			// adding markers to server
			surf_server_.remove_all_surfaces();
			visualization_msgs::MarkerArray markers_msg = surf_detect_.get_surface_markers();
			for(int i =0;i < markers_msg.markers.size();i++)
			{
				surf_server_.add_surface(markers_msg.markers[i]);
			}
		}
	}
	else
	{
		ROS_ERROR_STREAM("Acquisition failed");
	}
}

} /* namespace widgets */

}
