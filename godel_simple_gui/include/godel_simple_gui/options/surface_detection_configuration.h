#ifndef SURFACE_DETECTION_CONFIGURATION_H
#define SURFACE_DETECTION_CONFIGURATION_H

#include <godel_simple_gui/parameter_window_base.h>
#include <ui_surface_detection_configuration.h>
#include <godel_msgs/SurfaceDetectionParameters.h>

namespace Ui
{
class SurfaceDetectionConfigWindow;
}

namespace godel_simple_gui
{

class SurfaceDetectionConfigWidget : public ParameterWindowBase
{
  Q_OBJECT
public:
  SurfaceDetectionConfigWidget(godel_msgs::SurfaceDetectionParameters params);

  godel_msgs::SurfaceDetectionParameters& params() { return params_; }
  const godel_msgs::SurfaceDetectionParameters& params() const { return params_; }

  virtual void update_display_fields();

  virtual void update_internal_fields();

protected:
  godel_msgs::SurfaceDetectionParameters params_;
  Ui::SurfaceDetectionConfigWindow* ui_;
};
}

#endif // SURFACE_DETECTION_CONFIGURATION_H
