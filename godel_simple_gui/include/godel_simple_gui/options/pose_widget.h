#ifndef GODEL_SIMPLE_GUI_POSE_WIDGET
#define GODEL_SIMPLE_GUI_POSE_WIDGET

#include <QWidget>

#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>

namespace Ui
{
class PoseWidget;
}

namespace godel_simple_gui
{

class PoseWidget : public QWidget
{
  Q_OBJECT
public:
  PoseWidget(QWidget* parent = NULL);

  void set_values(const geometry_msgs::Pose& p);
  void set_values(const tf::Transform& t);
  tf::Transform get_values();

protected:
  Ui::PoseWidget* ui_;
};
}

#endif
