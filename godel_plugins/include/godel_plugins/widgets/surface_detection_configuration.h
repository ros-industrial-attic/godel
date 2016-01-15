#ifndef SURFACE_DETECTION_CONFIGURATION_H
#define SURFACE_DETECTION_CONFIGURATION_H

#include <godel_plugins/widgets/parameter_window_base.h>
#include <ui_surface_detection_configuration.h>
#include <godel_msgs/SurfaceDetectionParameters.h>

namespace godel_plugins
{

class SurfaceDetectionConfigWidget : public ParameterWindowBase
{
  Q_OBJECT
public:
  SurfaceDetectionConfigWidget(const godel_msgs::SurfaceDetectionParameters& params);

  godel_msgs::SurfaceDetectionParameters& params() { return params_; }

protected:
  virtual void update_gui_fields();

  virtual void update_internal_values();

  godel_msgs::SurfaceDetectionParameters params_;
  Ui::SurfaceDetectionConfigWindow ui_;
};
}

#endif // SURFACE_DETECTION_CONFIGURATION_H
